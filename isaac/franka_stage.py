#!/usr/bin/env python3
"""Isaac Sim standalone smoke: official Franka USD stage and visible scene.

Run with Isaac Sim's ./python.sh (not system Python).
Requires NVIDIA asset access or locally provisioned official asset pack.
"""
import json
import os
from pathlib import Path

from isaacsim import SimulationApp

HEADLESS = os.environ.get("HOMEAGENT_ISAAC_HEADLESS", "0") == "1"
app = SimulationApp({"headless": HEADLESS, "width": 1280, "height": 720})

try:
    import omni.usd
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.prims import is_prim_path_valid
    from isaacsim.storage.native import get_assets_root_path
    from pxr import UsdGeom, UsdLux, Gf

    assets_root = os.environ.get("HOMEAGENT_ISAAC_ASSET_ROOT") or get_assets_root_path()
    if not assets_root:
        raise RuntimeError("NVIDIA Isaac Sim asset root unavailable; provision assets before demo")

    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Cube.Define(stage, "/World/Ground")
    ground = stage.GetPrimAtPath("/World/Ground")
    cube = UsdGeom.Cube(ground)
    cube.GetSizeAttr().Set(1.0)
    UsdGeom.XformCommonAPI(ground).SetScale(Gf.Vec3f(12.0, 12.0, 0.05))
    UsdGeom.XformCommonAPI(ground).SetTranslate(Gf.Vec3d(0, 0, -0.075))
    light = UsdLux.DistantLight.Define(stage, "/World/KeyLight")
    light.CreateIntensityAttr(2500)
    light.CreateAngleAttr(1.0)
    light_xform = UsdGeom.XformCommonAPI(light.GetPrim())
    light_xform.SetRotate((35.0, -35.0, 0.0))

    candidates = [
        "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
        "/Isaac/Robots/Franka/franka.usd",
    ]
    asset = None
    for path in candidates:
        full = str(assets_root).rstrip("/") + path
        if full.startswith("/") and Path(full).is_file():
            asset = full
            break
        if not full.startswith("/") or os.environ.get("HOMEAGENT_ISAAC_ASSET_ROOT"):
            asset = full
            break
    if asset is None:
        raise RuntimeError("Franka USD not found in local root; specify HOMEAGENT_ISAAC_ASSET_ROOT")

    prim_path = "/World/Franka"
    add_reference_to_stage(asset, prim_path)
    for _ in range(120):
        app.update()
    valid = is_prim_path_valid(prim_path)
    output = {"asset": asset, "prim": prim_path, "prim_valid": bool(valid),
              "headless": HEADLESS, "render_verified": False,
              "joint_motion_verified": False}
    print("HOMEAGENT_ISAAC_STAGE", json.dumps(output), flush=True)
    if not valid:
        raise RuntimeError("Franka prim was not created")
    if not HEADLESS:
        while app.is_running():
            app.update()
finally:
    app.close()
