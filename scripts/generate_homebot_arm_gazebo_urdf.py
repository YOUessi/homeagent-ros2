#!/usr/bin/env python3
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
HOME_BOT = ROOT / "ros2_ws/src/homeagent_description/urdf/homebot.urdf.xacro"
HOME_ARM = ROOT / "ros2_ws/src/homeagent_manipulation/config/homearm.urdf.xacro"
OUTPUT = ROOT / "ros2_ws/src/homeagent_manipulation/config/homebot_arm_gazebo.urdf.xacro"


def inner_robot(text: str) -> str:
    start = text.index(">", text.index("<robot")) + 1
    end = text.rindex("</robot>")
    return text[start:end].strip()


def main() -> None:
    bot = HOME_BOT.read_text(encoding="utf-8")
    arm = HOME_ARM.read_text(encoding="utf-8")

    bot_inner = inner_robot(bot)
    arm_inner = inner_robot(arm)

    arm_inner = arm_inner.replace(
        "<plugin>mock_components/GenericSystem</plugin>\n"
        '      <param name="calculate_dynamics">false</param>',
        "<plugin>gazebo_ros2_control/GazeboSystem</plugin>",
        1,
    )

    # The moving finger must carry inertia when Gazebo physics owns the joints.
    arm_inner = arm_inner.replace(
        '''  <link name="gripper_fixed_finger_link">
    <visual>''',
        '''  <link name="gripper_fixed_finger_link">
    <inertial>
      <origin xyz="0.05 0 0"/>
      <mass value="0.05"/>
      <inertia ixx="0.00002" iyy="0.00005" izz="0.00005" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <visual>''',
        1,
    )
    arm_inner = arm_inner.replace(
        '''  <link name="gripper_finger_link">
    <visual>''',
        '''  <link name="gripper_finger_link">
    <inertial>
      <origin xyz="0.05 0 0"/>
      <mass value="0.05"/>
      <inertia ixx="0.00002" iyy="0.00005" izz="0.00005" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <visual>''',
        1,
    )

    # Physical mount: rear-top of HomeBot, clear of the LiDAR column.
    mount_joint = '''
  <joint name="homearm_mount_joint" type="fixed">
    <parent link="base_link"/>
    <child link="arm_base_footprint"/>
    <origin xyz="-0.12 0 0.10" rpy="0 0 0"/>
  </joint>
'''

    preserve_gripper_links = '''
  <gazebo reference="tool_joint">
    <preserveFixedJoint>true</preserveFixedJoint>
  </gazebo>
  <gazebo reference="gripper_fixed_finger_joint">
    <preserveFixedJoint>true</preserveFixedJoint>
  </gazebo>
'''

    # Gazebo owns HomeArm's ros2_control hardware in this combined model.
    gazebo_control = '''
  <gazebo reference="gripper_fixed_finger_link">
    <mu1>10.0</mu1>
    <mu2>10.0</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
  </gazebo>
  <gazebo reference="gripper_finger_link">
    <mu1>10.0</mu1>
    <mu2>10.0</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
  </gazebo>

  <gazebo>
    <plugin name="homearm_gazebo_ros2_control" filename="libgazebo_ros2_control.so">
      <parameters>$(arg controllers_file)</parameters>
    </plugin>

    <plugin name="homebot_wheel_joint_state" filename="libgazebo_ros_joint_state_publisher.so">
      <ros>
        <remapping>~/out:=joint_states</remapping>
      </ros>
      <update_rate>30</update_rate>
      <joint_name>left_wheel_joint</joint_name>
      <joint_name>right_wheel_joint</joint_name>
    </plugin>

    <plugin name="homeagent_contact_grasp" filename="libhomeagent_contact_grasp_plugin.so">
      <target_model>physical_cup</target_model>
      <target_link>link</target_link>
      <attach_link>tool_link</attach_link>
      <fixed_finger_token>gripper_fixed_finger_link</fixed_finger_token>
      <moving_finger_token>gripper_finger_link</moving_finger_token>
      <gripper_joint>gripper_joint</gripper_joint>
      <close_threshold>0.008</close_threshold>
      <open_threshold>0.020</open_threshold>
      <contact_window_sec>0.20</contact_window_sec>
    </plugin>
  </gazebo>
'''

    # Avoid carrying the standalone root twice; arm_base_footprint remains a link,
    # but now it is physically fixed to HomeBot base_link.
    arm_inner = arm_inner.replace(
        '  <link name="arm_base_footprint"/>',
        '  <link name="arm_base_footprint"/>\n' + mount_joint,
        1,
    )

    combined = f'''<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="homebot_arm">
  <xacro:arg name="controllers_file" default=""/>
{bot_inner}

{arm_inner}

{preserve_gripper_links}

{gazebo_control}
</robot>
'''

    OUTPUT.write_text(combined, encoding="utf-8")
    print(OUTPUT)


if __name__ == "__main__":
    main()
