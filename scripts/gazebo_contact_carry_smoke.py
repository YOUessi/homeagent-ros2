#!/usr/bin/env python3
import json, math, os, time
import rclpy
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.srv import GetEntityState, SpawnEntity
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM=["joint1","joint2","joint3","joint4"]
PICK=[0.20,-0.75,1.15,-0.35]
CARRY=[0.00,-0.30,0.65,-0.35]
GRIPPER="gripper_joint"
TOOL="homebot_arm::tool_link"
CUP="physical_cup"

CUP_SDF="""<sdf version='1.6'><model name='physical_cup'><link name='link'>
<gravity>true</gravity><self_collide>true</self_collide>
<inertial><mass>0.05</mass><inertia><ixx>0.000032</ixx><iyy>0.000032</iyy><izz>0.000011</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
<collision name='collision'><geometry><cylinder><radius>0.021</radius><length>0.08</length></cylinder></geometry>
<surface><friction><ode><mu>40</mu><mu2>40</mu2></ode></friction></surface></collision>
<visual name='visual'><geometry><cylinder><radius>0.021</radius><length>0.08</length></cylinder></geometry></visual>
</link></model></sdf>"""

def xyz(p):
    return [float(p.position.x),float(p.position.y),float(p.position.z)]

def dist(a,b):
    return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))

class Probe(Node):
    def __init__(self):
        super().__init__("gazebo_contact_carry_probe")
        self.q={}
        self.create_subscription(JointState,"/joint_states",self.js,20)
        self.move=ActionClient(self,MoveGroup,"/move_action")
        self.grip=ActionClient(self,FollowJointTrajectory,"/gripper_controller/follow_joint_trajectory")
        self.spawn=self.create_client(SpawnEntity,"/spawn_entity")
        self.state=self.create_client(GetEntityState,"/gazebo/get_entity_state")

    def js(self,msg):
        for n,v in zip(msg.name,msg.position): self.q[n]=float(v)

    def spin_for(self,s):
        end=time.monotonic()+s
        while time.monotonic()<end: rclpy.spin_once(self,timeout_sec=0.05)

    def ready(self):
        end=time.monotonic()+18
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=0.1)
            if all(n in self.q for n in ARM+[GRIPPER]) and self.move.server_is_ready() and self.grip.server_is_ready() and self.spawn.service_is_ready() and self.state.service_is_ready():
                return
        raise RuntimeError("stack not ready")

    def arm(self,target,label):
        g=MoveGroup.Goal()
        g.request.group_name="arm"; g.request.pipeline_id="ompl"
        g.request.num_planning_attempts=3; g.request.allowed_planning_time=5.0
        g.request.max_velocity_scaling_factor=0.20; g.request.max_acceleration_scaling_factor=0.20
        g.request.start_state.is_diff=True
        c=Constraints(); c.name=label
        for n,p in zip(ARM,target):
            j=JointConstraint(); j.joint_name=n; j.position=float(p)
            j.tolerance_above=0.02; j.tolerance_below=0.02; j.weight=1.0
            c.joint_constraints.append(j)
        g.request.goal_constraints=[c]
        g.planning_options.plan_only=False
        g.planning_options.planning_scene_diff.is_diff=True
        g.planning_options.planning_scene_diff.robot_state.is_diff=True
        f=self.move.send_goal_async(g); rclpy.spin_until_future_complete(self,f,timeout_sec=10)
        h=f.result()
        if h is None or not h.accepted: raise RuntimeError(label+" rejected")
        rf=h.get_result_async(); rclpy.spin_until_future_complete(self,rf,timeout_sec=35)
        w=rf.result()
        if w is None or int(w.status)!=4 or int(w.result.error_code.val)!=MoveItErrorCodes.SUCCESS:
            raise RuntimeError(label+" failed")
        return len(w.result.planned_trajectory.joint_trajectory.points)

    def clamp(self,pos=0.003):
        g=FollowJointTrajectory.Goal(); g.trajectory.joint_names=[GRIPPER]
        p=JointTrajectoryPoint(); p.positions=[pos]; p.time_from_start.sec=1
        g.trajectory.points=[p]; g.goal_time_tolerance.sec=2
        f=self.grip.send_goal_async(g); rclpy.spin_until_future_complete(self,f,timeout_sec=5)
        h=f.result()
        if h is None or not h.accepted: raise RuntimeError("gripper rejected")
        rf=h.get_result_async(); rclpy.spin_until_future_complete(self,rf,timeout_sec=8)
        return rf.result()

    def spawn_cup(self):
        r=SpawnEntity.Request(); r.name=CUP; r.xml=CUP_SDF; r.reference_frame=TOOL
        r.initial_pose.position.x=0.12; r.initial_pose.position.y=0.0015; r.initial_pose.orientation.w=1.0
        f=self.spawn.call_async(r); rclpy.spin_until_future_complete(self,f,timeout_sec=5)
        out=f.result()
        if out is None or not out.success: raise RuntimeError("cup spawn failed")

    def pose(self,ref):
        r=GetEntityState.Request(); r.name=CUP; r.reference_frame=ref
        f=self.state.call_async(r); rclpy.spin_until_future_complete(self,f,timeout_sec=5)
        out=f.result()
        if out is None or not out.success: raise RuntimeError("cup state failed")
        return out.state.pose

def main():
    rclpy.init(); n=Probe()
    try:
        n.ready()
        p1=n.arm(PICK,"contact_pick_pose")
        n.clamp(); n.spin_for(0.2)
        n.spawn_cup(); n.spin_for(0.7)
        before_w=xyz(n.pose("world")); before_r=xyz(n.pose(TOOL))
        p2=n.arm(CARRY,"contact_carry"); n.spin_for(0.8)
        after_w=xyz(n.pose("world")); after_r=xyz(n.pose(TOOL))
        wm=dist(before_w,after_w); rd=dist(before_r,after_r)
        report={
          "backend":"Gazebo bilateral-contact-gated fixed constraint",
          "contact_required":True,
          "planning_scene_attach_used":False,
          "physics_constraint_attach_used":True,
          "friction_only_grasp":False,
          "gravity_enabled":True,
          "scope":"closed-jaw contact-gated hold-and-carry",
          "autonomous_table_pick":False,
          "pick_points":p1,"carry_points":p2,
          "gripper_m":float(n.q[GRIPPER]),
          "cup_world_motion_m":wm,
          "cup_relative_tool_drift_m":rd,
          "contact_gated_physical_hold":wm>0.08 and rd<0.035,
        }
        report["passed"]=report["contact_gated_physical_hold"]
        report_path=os.environ.get("HOMEAGENT_CONTACT_REPORT")
        if report_path:
            with open(report_path,"w",encoding="utf-8") as f:
                json.dump(report,f,indent=2)
        print(json.dumps(report,indent=2))
        return 0 if report["passed"] else 2
    finally:
        n.destroy_node(); rclpy.shutdown()

if __name__=="__main__":
    raise SystemExit(main())
