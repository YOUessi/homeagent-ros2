# Isaac Sim — HomeAgent High-Fidelity Simulation

This is a separate simulation backend. The existing Gazebo acceptance branch remains untouched.

## First acceptance: official Franka Panda

Launch Isaac Sim standalone workstation, then run:

```bash
./python.sh /path/to/homeagent-ros2/isaac/franka_stage.py
```

The script resolves NVIDIA's official USD Franka asset, creates a ground plane and lighting, stages the robot, and verifies the asset prim. An actual rendered frame and the joint test must be checked before reporting PASS.

## Recommended deployment
- Preferred: workstation GUI on Tang; use NVIDIA Isaac Sim's official WebRTC livestream workflow for remote viewing.
- Avoid treating a TCP listening port, app process, or stage validation alone as a visual acceptance.
- 16 GiB GPU VRAM, 31 GiB system RAM on Tang: start with one Franka, ground plane, no extra RTX camera and no Isaac Lab training.
- Source-of-truth is this GitHub branch; local Tang is only for installation and runtime verification.

## Planned integration
1. Official Panda USD visual and joints.
2. Stable physical mounting on a mobile HomeBot chassis.
3. PhysX gripper contact tests with measured holding/slipping under gravity, no forced attach as baseline.
4. ROS 2 bridge, MoveIt/Navigation adapter, and typed sensor messages.
5. Residential environment and remote live demonstration, with timestamps and video evidence.

Do not conflate the generic Isaac Sim USD Panda with already-mounted HomeBot.
