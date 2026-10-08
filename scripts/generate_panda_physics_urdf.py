#!/usr/bin/env python3
"""Generate a Panda Gazebo Classic physics URDF from installed ROS packages.

Retain official MoveIt Panda visual CAD meshes and joint kinematics.
Franka FER inertials supply a physics starting point; simplified collision
proxies are explicitly simulation approximations, NOT measured collision CAD.

Generated URDF is an artifact; this generator is the versioned source.
"""
import argparse
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

PANDA = Path(
    "/opt/ros/humble/share/moveit_resources_panda_description/urdf/panda.urdf"
)
INERTIALS = Path(
    "/opt/ros/humble/share/franka_description/robots/fer/inertials.yaml"
)
COLLISION_RADIUS = [0.105, 0.082, 0.077, 0.072, 0.068, 0.065, 0.057, 0.048]
COLLISION_LENGTH = [0.105, 0.175, 0.195, 0.185, 0.18, 0.17, 0.145, 0.100]
FINGERS = {"panda_leftfinger", "panda_rightfinger"}


def sub(parent, tag, **attributes):
    return ET.SubElement(parent, tag, {k: str(v) for k, v in attributes.items()})


def add_inertial(link, data):
    inertial = sub(link, "inertial")
    origin = data["origin"]
    sub(inertial, "origin", xyz=origin["xyz"], rpy=origin.get("rpy", "0 0 0"))
    sub(inertial, "mass", value=data["mass"])
    i = data["inertia"]
    sub(
        inertial, "inertia",
        ixx=i["xx"], ixy=i["xy"], ixz=i["xz"],
        iyy=i["yy"], iyz=i["yz"], izz=i["zz"],
    )


def add_collision_proxy(link, radius, height, *, finger=False, center="0 0 0"):
    # Proxies prioritize stable contact and rapid Gazebo testing.
    for old in list(link.findall("collision")):
        link.remove(old)
    coll = sub(link, "collision")
    if finger:
        sub(coll, "origin", xyz="0 0 0.030", rpy="0 0 0")
        geom = sub(coll, "geometry")
        sub(geom, "box", size="0.018 0.016 0.060")
    else:
        sub(coll, "origin", xyz=center, rpy="0 0 0")
        geom = sub(coll, "geometry")
        sub(geom, "cylinder", radius=radius, length=height)


def default_inertial(link_name):
    if link_name == "panda_hand":
        mass = 0.70
    elif link_name == "panda_link8":
        mass = 0.12
    else:
        mass = 0.07
    return {
        "origin": {"xyz": "0 0 0", "rpy": "0 0 0"},
        "mass": mass,
        "inertia": {
            "xx": 0.001, "xy": 0.0, "xz": 0.0,
            "yy": 0.001, "yz": 0.0, "zz": 0.001,
        },
    }


def add_control(root, controllers_yaml):
    ctrl = sub(root, "ros2_control", name="PandaGazeboSystem", type="system")
    hw = sub(ctrl, "hardware")
    sub(hw, "plugin").text = "gazebo_ros2_control/GazeboSystem"

    initial = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    for index in range(1, 8):
        joint = sub(ctrl, "joint", name=f"panda_joint{index}")
        sub(joint, "command_interface", name="position")
        sub(joint, "state_interface", name="position")
        sub(joint, "state_interface", name="velocity")

    finger1 = sub(ctrl, "joint", name="panda_finger_joint1")
    sub(finger1, "command_interface", name="position")
    sub(finger1, "state_interface", name="position")
    sub(finger1, "state_interface", name="velocity")

    finger2 = sub(ctrl, "joint", name="panda_finger_joint2")
    sub(finger2, "param", name="mimic").text = "panda_finger_joint1"
    sub(finger2, "param", name="multiplier").text = "1"
    sub(finger2, "state_interface", name="position")
    sub(finger2, "state_interface", name="velocity")

    gazebo = sub(root, "gazebo")
    plugin = sub(
        gazebo, "plugin", name="panda_gazebo_ros2_control",
        filename="libgazebo_ros2_control.so",
    )
    sub(plugin, "parameters").text = str(controllers_yaml)
    sub(plugin, "robot_param").text = "robot_description"
    sub(plugin, "robot_param_node").text = "robot_state_publisher"

    # Do not add a second /joint_states publisher: the controller manager
    # already publishes the authoritative joint state topic. Physical finger
    # separation is validated via Gazebo GetEntityState instead.


def configure_mobile_chassis(homebot):
    """Panda-class demo chassis with a stable forward/rear support polygon.

    The original lightweight HomeBot model remains unchanged. These are
    explicit simulation prototype dimensions, not a fabricated production BOM.
    """
    chassis = homebot.find("./link[@name='base_link']")
    inertia = chassis.find("inertial")
    inertia.find("mass").set("value", "44.0")
    moments = inertia.find("inertia")
    moments.set("ixx", "0.99")
    moments.set("iyy", "1.63")
    moments.set("izz", "2.47")
    for visual_or_collision in ("visual", "collision"):
        body_size = chassis.find(f"./{visual_or_collision}/geometry/box")
        if body_size is None:
            raise ValueError("HomeBot body box geometry missing")
        body_size.set("size", "0.62 0.49 0.16")

    # Increase drive wheel separation, maintaining the original wheel radius.
    for name, sign in (("left_wheel_joint", 1), ("right_wheel_joint", -1)):
        joint = homebot.find(f"./joint[@name='{name}']")
        joint.find("origin").set("xyz", f"0 {0.23*sign:.3f} -0.07")
    for plugin in homebot.findall(".//plugin"):
        if plugin.attrib.get("name") == "homebot_diff_drive":
            wheel_separation = plugin.find("wheel_separation")
            if wheel_separation is None:
                raise ValueError("homebot_diff_drive wheel separation missing")
            wheel_separation.text = "0.46"
            plugin.find("max_wheel_acceleration").text = "1.2"

    # Keep the existing low-friction rear caster, but shift it rearward.
    rear = homebot.find("./joint[@name='caster_joint']")
    rear.find("origin").set("xyz", "-0.26 0 -0.09")

    # A second front contact point stops the front-heavy Panda from pitching
    # into the floor when accelerating/stopping.
    front = sub(homebot, "link", name="front_caster_link")
    inertial = sub(front, "inertial")
    sub(inertial, "mass", value="0.20")
    sub(
        inertial, "inertia",
        ixx="0.0002", ixy="0", ixz="0",
        iyy="0.0002", iyz="0", izz="0.0002",
    )
    visual = sub(front, "visual")
    sub(sub(visual, "geometry"), "sphere", radius="0.035")
    collision = sub(front, "collision")
    sub(sub(collision, "geometry"), "sphere", radius="0.035")
    joint = sub(homebot, "joint", name="front_caster_joint", type="fixed")
    sub(joint, "parent", link="base_link")
    sub(joint, "child", link="front_caster_link")
    sub(joint, "origin", xyz="0.26 0 -0.09", rpy="0 0 0")
    friction = sub(homebot, "gazebo", reference="front_caster_link")
    sub(friction, "mu1").text = "0.05"
    sub(friction, "mu2").text = "0.05"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--controllers", required=True)
    parser.add_argument(
        "--mount", choices=["fixed_base", "homebot"], default="fixed_base",
        help="fixed Panda base, or dynamically attach it to HomeBot chassis",
    )
    parser.add_argument(
        "--homebot-xacro",
        default="/workspace/ros2_ws/src/homeagent_description/urdf/homebot.urdf.xacro",
    )
    args = parser.parse_args()

    inertials = yaml.safe_load(INERTIALS.read_text())
    tree = ET.parse(PANDA)
    root = tree.getroot()
    root.attrib.pop("{http://www.ros.org/wiki/xacro}unused", None)

    for index in range(8):
        link = root.find(f"./link[@name='panda_link{index}']")
        if link is None:
            raise ValueError(f"missing panda_link{index}")
        add_inertial(link, inertials[f"link{index}"])
        if index == 7:
            # The distal Panda wrist has tightly nested links.
            # A centered cylinder falsely intersects link5 in the ready pose.
            # Keep CAD visual/inertia; omit this particular *proxy* until a
            # non-overlapping measured collision model is integrated.
            for collision in list(link.findall("collision")):
                link.remove(collision)
        else:
            add_collision_proxy(
                link,
                (0.042 if index == 5 else COLLISION_RADIUS[index]),
                (0.10 if index == 5 else COLLISION_LENGTH[index]),
                center=inertials[f"link{index}"]["origin"]["xyz"],
            )

    for name in ("panda_link8", "panda_hand", *sorted(FINGERS)):
        link = root.find(f"./link[@name='{name}']")
        if link is None:
            raise ValueError(f"missing {name}")
        add_inertial(link, default_inertial(name))
        if name == "panda_link8":
            # Fixed tool flange: visual-only; its approximate cylinder would
            # overlap the adjacent hand collision body and falsely block MoveIt.
            for collision in list(link.findall("collision")):
                link.remove(collision)
        else:
            add_collision_proxy(
                link, 0.028 if name == "panda_hand" else 0.05,
                0.045 if name == "panda_hand" else 0.075,
                finger=name in FINGERS,
            )

    if args.mount == "fixed_base":
        sub(root, "link", name="world")
        anchor = sub(root, "joint", name="panda_world_fixed", type="fixed")
        sub(anchor, "parent", link="world")
        sub(anchor, "child", link="panda_link0")
        sub(anchor, "origin", xyz="0 0 0.45", rpy="0 0 0")

    add_control(root, args.controllers)

    if args.mount == "homebot":
        # The full mobile robot is one URDF and one Gazebo physics model.
        # The original HomeBot sensor, diff-drive and odometry plugins remain.
        xacro_xml = subprocess.check_output(
            ["xacro", args.homebot_xacro], stderr=subprocess.PIPE
        )
        homebot = ET.fromstring(xacro_xml)
        # Preserve Panda's robot name so the official Panda SRDF remains
        # usable by MoveIt (its active planning group is panda_arm).
        homebot.set("name", "panda")
        for element in list(root):
            homebot.append(element)

        # Only the two passive wheel joints are mirrored to /joint_states.
        # Panda arm/finger state remains owned exclusively by ros2_control;
        # never publish a competing second source for Panda joints.
        wheel_gz = sub(homebot, "gazebo")
        wheel_plugin = sub(
            wheel_gz, "plugin", name="homebot_wheel_state",
            filename="libgazebo_ros_joint_state_publisher.so",
        )
        wheel_ros = sub(wheel_plugin, "ros")
        sub(wheel_ros, "remapping").text = "~/out:=joint_states"
        sub(wheel_plugin, "update_rate").text = "30"
        sub(wheel_plugin, "joint_name").text = "left_wheel_joint"
        sub(wheel_plugin, "joint_name").text = "right_wheel_joint"

        mount = sub(
            homebot, "joint", name="homebot_panda_mount", type="fixed"
        )
        sub(mount, "parent", link="base_link")
        sub(mount, "child", link="panda_link0")
        sub(mount, "origin", xyz="-0.14 0 0.18", rpy="0 0 0")

        # Stabilize the mobile base for a Panda-class manipulator using
        # a wider support polygon and mass-matched inertial tensor.
        configure_mobile_chassis(homebot)

        root = homebot
        tree = ET.ElementTree(root)

    # Gazebo-side grasp is gated by two distinct physical finger contacts.
    # It is a fixed-constraint simulation grasp, NOT a friction-only grasp.
    if args.mount == "homebot":
        grasp_gz = sub(root, "gazebo")
        grasp = sub(
            grasp_gz, "plugin", name="panda_bilateral_contact_grasp",
            filename="libhomeagent_contact_grasp_plugin.so",
        )
        for key, value in (
            ("target_model", "panda_grasp_cup"),
            ("target_link", "link"),
            ("attach_link", "panda_hand"),
            ("fixed_finger_token", "panda_leftfinger"),
            ("moving_finger_token", "panda_rightfinger"),
            ("gripper_joint", "panda_finger_joint1"),
            ("close_threshold", "0.012"),
            ("open_threshold", "0.030"),
            ("contact_window_sec", "0.5"),
        ):
            sub(grasp, key).text = value

    target = Path(args.output)
    target.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(target, encoding="unicode", xml_declaration=False)
    print("generated", target, "bytes", target.stat().st_size, flush=True)
    print("panda_revolute_joints", 7, "finger_joints", 2, flush=True)
    print("official_visual_meshes", sum(
        len(link.findall(".//visual//mesh"))
        for link in root.findall("link")
    ), flush=True)


if __name__ == "__main__":
    main()
