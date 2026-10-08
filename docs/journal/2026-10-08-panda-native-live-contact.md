# 2026-10-08 — Panda + HomeBot 原生 Gazebo 3D 与双指接触抓取

## 目标
将通过物理关节验收的同体 `homebot_panda` 机器人长期运行于 Gazebo Classic 三维世界，提供可实时浏览的原生 gzclient/noVNC 页面，并完成双指接触、受重力抓持、携带与释放验收。

## 环境与成果
- Tang：`/home/you/projects/homeagent-ros2`；分支 `feature/panda-gazebo-physics-20261008`。
- 运行 `bash scripts/start_panda_native_gazebo_live.sh`。ROS_DOMAIN_ID=80，GAZEBO_MASTER_URI=http://127.0.0.1:11380，两个常驻容器 `homeagent-panda-live` / `homeagent-panda-3d`。
- 原生 1920×1080 Gazebo Classic gzclient 通过 noVNC 暴露在 Tailscale 内网 http://100.84.167.3:6082/vnc.html?autoconnect=true&resize=remote 。客户端连接 master，HTTP 200，视频界面仍需观看者在自己的浏览器核验体验与帧率；不对外网开放。
- `scripts/generate_panda_physics_urdf.py` 为移动 Panda 添加双侧接触门控插件；`ros2_ws/src/homeagent_gazebo_plugins/src/contact_grasp_plugin.cpp` 在接触后创建 fixed joint，且将目标 gravity_mode 设置为 true。使用 Gazebo 中实际存在的 `panda_leftfinger` link。

## 失败和修复过程
1. 原页面连接 11345 旧 Gazebo；新模型位于 11380。另建独立实时 viewer。
2. 6081 端口被 RViz demo 占用：新 viewer 改用 DISPLAY=:97 / VNC 5907 / noVNC 6082。
3. 初始新容器无法复用另一个容器的 /tmp colcon install：启动时自行 colcon build，并提供 GAZEBO_PLUGIN_PATH。
4. SpawnEntity 不能用 link 作为 reference_frame：通过 `/gazebo/link_states` 获取实际世界坐标并按四元数变换物体偏置。
5. GetEntityState 查询不到 link，GetLinkState 服务不存在：使用 `/gazebo/link_states` 订阅。
6. Gazebo 合并 panda_hand fixed joint 后无法查询/挂接该 link：改使用独立 `panda_leftfinger` link。
7. SetLinkProperties 服务不存在：接触门控插件在 ATTACHED 后直接调用 child_link->SetGravityMode(true)。
8. 重力布尔标记未生效：插件改用 SDF Get<bool> 读取，启动日志显示 gravity_on_attach=1。

## 第七轮通过
使用 `scripts/panda_live_contact_carry.py`：位置已知的杯子先在无重力下放入张开的双指之间，机械闭合触发双方 Gazebo collision contact 并附着 fixed constraint，同时启用重力；MoveIt2 执行 15 点轨迹，夹爪放开后杯子下落。
- 抓取日志顺序：gravity enabled → ATTACHED after bilateral contact → DETACHED。
- 手指开 0.0349995m / 闭 0.0049981m / 释放后开 0.0349980m。
- 杯子世界位移：0.227985m；杯子相对手指参考点位置变化：0.025130m；释放前高度 0.882606m，释放后高度 0.252050m。
- MoveIt success=true，程序 RC=0，report passed=true。
- 完整结构化追踪：`docs/journal/traces/2026-10-08-panda-native-gazebo-grasp-r7.json`，Tang 本地原始日志：`artifacts/panda_live/contact_carry_run7.log` 和 `artifacts/panda_live/launch.log`。

## 不能扩大为的结论
这是 **Gazebo 真正双指接触门控 + fixed constraint 抓持**，不是纯摩擦力抓取；杯子位姿由测试脚本提供而不是视觉自动定位；没有验证多物体鲁棒性、真实硬件、无辅助摆放抓取或完整自主家务任务。
