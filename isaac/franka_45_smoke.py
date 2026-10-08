#!/usr/bin/env python3
"""Isaac Sim 4.5 / Isaac Lab 0.41 native Panda minimal runtime smoke.

Single genuine NVIDIA Franka articulation. No fake primitives, no robot
mounting claims. Limited steps allow coexistence with other GPU experiments.
"""
import argparse
import json

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
parser.add_argument("--steps", type=int, default=60)
args = parser.parse_args()
app = AppLauncher(args).app

try:
    import isaaclab.sim as sim_utils
    from isaaclab.assets import Articulation
    from isaaclab_assets import FRANKA_PANDA_CFG
    from isaaclab.sim import SimulationCfg, SimulationContext

    sim = SimulationContext(SimulationCfg(dt=0.01))
    sim.set_camera_view([2.5, 2.3, 2.2], [0.0, 0.0, 0.6])
    sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())
    light = sim_utils.DomeLightCfg(intensity=1800.0)
    light.func("/World/Light", light)
    robot_cfg = FRANKA_PANDA_CFG.replace(prim_path="/World/Panda")
    robot_cfg.init_state.pos = (0.0, 0.0, 0.15)
    panda = Articulation(cfg=robot_cfg)
    sim.reset()
    print("HOMEAGENT_ISAAC_READY", json.dumps({
        "robot": "Franka Panda (official Isaac Lab asset)",
        "num_joints": int(panda.num_joints),
        "num_bodies": int(panda.num_bodies),
        "headless": bool(args.headless),
        "steps_target": args.steps,
    }), flush=True)
    for _ in range(args.steps):
        panda.write_data_to_sim()
        sim.step()
        panda.update(0.01)
    print("HOMEAGENT_ISAAC_SMOKE_PASS", flush=True)
finally:
    app.close()
