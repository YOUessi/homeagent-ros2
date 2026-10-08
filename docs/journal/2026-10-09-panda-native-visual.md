# 2026-10-09 — Panda 在原生 Gazebo 中的真实连杆视觉恢复

## 问题
网页中的 HomeBot 只有底盘和几段不连续的圆柱体，Panda 七轴连杆的官方 DAE CAD 外观未渲染。此前诊断圆柱体不得作为最终模型展示。

## 排查与修复
- Tang 上的 `homeagent-panda-live` 和 `homeagent-panda-3d` 均运行；`gzclient` 在 DAE 资源场景曾出现严重 CPU 消耗。
- 确认官方 ROS Humble `moveit_resources_panda_description` 同时包含 DAE visual mesh 和 STL collision mesh；STL 是官方提供的实际连杆近似几何，而不是手工拼装。
- 将 URDF 视觉 mesh 定位从 `model://panda_visual/meshes/visual/*.dae` 改为 `model://panda_visual/meshes/collision/*.stl`，不改七轴运动学、惯量或 GazeboSystem 控制接口。
- 删除七个 `panda_link*_diagnostic_body` 圆柱形临时 visual，启动脚本新增对官方 STL 网格的 Gazebo model resource 映射。
- 实际 Tang 截图 `artifacts/panda_live/stl_check3.png` 已看到机械臂整体外形，`artifacts/panda_live/stl_demo_frame.png` 已看到机械臂在执行动作时的另一种位姿。截图是 960×540 的实时 Gazebo 客户端采集。
- 当前 Gazebo Tailscale 浏览器地址 http://100.84.167.3:6082/vnc.html?autoconnect=true&resize=remote 。

## 抓取复验
运行 `scripts/panda_live_contact_carry.py`，Tang 原始输出 `artifacts/panda_live/stl_contact_demo.log`：`passed=true`、MoveIt 15 个轨迹点成功、杯子世界位移 0.223982m、相对手指参考点漂移 0.025778m、`ATTACHED` 与 `DETACHED` 都出现；松开后的杯子 z=0.252337m。

## 验收边界
官方 STL 是简化 CAD 网格，不是带原始材质的高精度 DAE。夹持仍为双指 Gazebo collision-contact 门控后的 fixed constraint，非纯摩擦维持，也非自主物体识别定位。已验证机械臂视觉出现与抓取逻辑测试，但尚未制作全程连续录像、高清末端放大视角及多物体鲁棒性测试。
