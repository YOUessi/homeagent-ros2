# 2026-10-09 — Isaac Sim branch kickoff

## Decision
Migrate high-fidelity rendering, physically simulated manipulation and sensor development to Isaac Sim. Keep Gazebo Classic regression baseline in its original branch, not replace its artifacts or claim equivalent physics.

## Verified machine environment (Tang)
Ubuntu 22.04.5 x86_64; RTX 4090 Laptop GPU (NOT desktop 24 GiB), 16,376 MiB VRAM, NVIDIA driver 580.178.04; 31 GiB RAM (around 16 GiB available at probe), 71 GiB available disk on root filesystem.

## Constraints and risks
- GPU VRAM is at the minimum of Isaac Sim 5.1, with little margin for several RTX sensors and elaborate environments.
- RAM is close to 5.1 minimum. Disk available (71 GB) is enough for a basic application but not unrestricted packs and caches.
- Need check official compatibility and effective runtime before choosing a working version. No Isaac Sim run is claimed yet.
- High-fidelity requirement includes genuine Panda and HomeBot assets, correct kinematic assembly, plausible contact and lighting, not a cosmetic primitive swap.
- Network/cloud asset access needs checking.
- Visual acceptance must include an inspected screenshot plus a time-continuous recording, not merely process logs.

## Artifacts
`isaac/franka_stage.py` is a first standalone scene smoke, not an integrated HomeBot nor grasp success.
`scripts/check_isaac_tang.sh` is a read-only environment check.

## Next tests
1. Confirm Isaac Sim installed or obtain package and run compatibility checker.
2. Verify official Franka USD asset can load, rendered screenshot looks correct, and joint articulation steps.
3. Mount Panda on HomeBot and test articulation stability.
4. Verify gripper friction-only grasp vs fixed-constraint baseline, drop/release.
5. Connect existing ROS2 Agent tools and publish browser-observable continuous replay.
