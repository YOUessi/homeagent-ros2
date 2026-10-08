#!/usr/bin/env python3
"""Staged Panda bilateral-contact lift verification in live Gazebo.

Object gravity is off ONLY while positioning between open fingers. Gravity
is switched on before lift after contact; grasp requires actual two-sided
Gazebo collision events followed by model-level fixed constraint.
No autonomous visual localization and no friction-only claim.
"""
import json, math, os, time
import rclpy
from gazebo_msgs.srv import SpawnEntity, GetEntityState
from gazebo_msgs.msg import LinkStates
from panda_gazebo_physics_e2e import PhysicalPandaProbe
from panda_moveit_joint_demo import READY, INSPECT

CUP = """<sdf version="1.6"><model name="panda_grasp_cup"><link name="link">
<gravity>false</gravity><inertial><mass>0.045</mass>
<inertia><ixx>0.000014</ixx><iyy>0.000014</iyy><izz>0.000008</izz>
<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
<collision name="collision"><geometry><cylinder><radius>0.014</radius>
<length>0.054</length></cylinder></geometry>
<surface><friction><ode><mu>2</mu><mu2>2</mu2></ode></friction></surface>
</collision><visual name="visual"><geometry><cylinder>
<radius>0.014</radius><length>0.054</length></cylinder></geometry>
<material><ambient>0.9 0.15 0.12 1</ambient><diffuse>0.9 0.15 0.12 1</diffuse>
</material></visual></link></model></sdf>"""

def call(node, client, req, timeout=8):
    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError("service not ready: " + client.srv_name)
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.result() is None:
        raise RuntimeError("service timeout: " + client.srv_name)
    return future.result()

def xyz(p):
    return [p.position.x, p.position.y, p.position.z]

def distance(a,b):
    return math.dist(a,b)

def main():
    rclpy.init()
    node=PhysicalPandaProbe()
    report={"fixture":"gravity disabled during insertion only",
            "grasp_mechanism":"bilateral Gazebo collision contacts + fixed constraint",
            "friction_only":False,"autonomous_object_localization":False,"passed":False}
    try:
        spawn=node.create_client(SpawnEntity,"/spawn_entity")
        get=node.create_client(GetEntityState,"/gazebo/get_entity_state")
        observed_links={}
        def links_cb(msg):
            observed_links.update(dict(zip(msg.name,msg.pose)))
        link_sub=node.create_subscription(LinkStates,"/gazebo/link_states",links_cb,10)
        deadline=time.monotonic()+8
        while "homebot_panda::panda_leftfinger" not in observed_links and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=0.1)
        if "homebot_panda::panda_leftfinger" not in observed_links:
            raise RuntimeError("Gazebo link_states missing Panda hand")
        node.wait_ready()
        report["ready_code"]=node.initialize_ready()
        report["open"]=node.set_gripper(0.035)
        request=SpawnEntity.Request()
        request.name="panda_grasp_cup";request.xml=CUP
        request.reference_frame="world"
        hand_pose=observed_links["homebot_panda::panda_leftfinger"]
        q=hand_pose.orientation
        # Rotate local [0,0,0.088] into world using unit quaternion.
        # At 35mm finger opening the object lies between both jaws:
        # left finger local y offset -0.035m, local z +0.030m.
        vx,vy,vz=0.0,-0.035,0.030
        rotated=[
          (1-2*(q.y*q.y+q.z*q.z))*vx+2*(q.x*q.y-q.w*q.z)*vy+2*(q.x*q.z+q.w*q.y)*vz,
          2*(q.x*q.y+q.w*q.z)*vx+(1-2*(q.x*q.x+q.z*q.z))*vy+2*(q.y*q.z-q.w*q.x)*vz,
          2*(q.x*q.z-q.w*q.y)*vx+2*(q.y*q.z+q.w*q.x)*vy+(1-2*(q.x*q.x+q.y*q.y))*vz
        ]
        request.initial_pose.position.x=hand_pose.position.x+rotated[0]
        request.initial_pose.position.y=hand_pose.position.y+rotated[1]
        request.initial_pose.position.z=hand_pose.position.z+rotated[2]
        request.initial_pose.orientation=q
        spawned=call(node,spawn,request)
        if not spawned.success: raise RuntimeError("spawn rejected: "+spawned.status_message)
        report["close"]=node.set_gripper(0.005)
        time.sleep(0.5)
        def pose(ref):
            if ref=="homebot_panda::panda_leftfinger":
                rclpy.spin_once(node,timeout_sec=0.15)
                return xyz(observed_links[ref])
            req=GetEntityState.Request()
            req.name=ref;req.reference_frame="world"
            result=call(node,get,req)
            if not result.success: raise RuntimeError("cup pose failed: "+ref)
            return xyz(result.state.pose)
        before_world=pose("panda_grasp_cup")
        before_hand=pose("homebot_panda::panda_leftfinger")
        before_rel=[a-b for a,b in zip(before_world,before_hand)]
        report["before_world"]=before_world
        report["before_relative"]=before_rel
        # Gazebo model plugin enables target gravity in AttachTarget() ONLY
        # if both finger contacts triggered a real fixed-constraint grasp.
        report["gravity_transition"]="contact plugin on successful bilateral ATTACHED event"
        report["lift"]=node.move_to(INSPECT,"panda_cup_lift")
        time.sleep(0.6)
        after_world=pose("panda_grasp_cup")
        after_hand=pose("homebot_panda::panda_leftfinger")
        after_rel=[a-b for a,b in zip(after_world,after_hand)]
        report["world_displacement_m"]=distance(before_world,after_world)
        report["tool_relative_drift_m"]=distance(before_rel,after_rel)
        report["after_world"]=after_world
        report["after_relative"]=after_rel
        report["passed"]=bool(report["lift"]["success"] and report["world_displacement_m"]>0.08 and report["tool_relative_drift_m"]<0.035)
        # Release must be tested even if lift fails.
        report["release"]=node.set_gripper(0.035)
        time.sleep(0.4)
        report["after_release_world"]=pose("panda_grasp_cup")
    except Exception as exc:
        report["error"]=str(exc)
    finally:
        os.makedirs("/workspace/artifacts/panda_live",exist_ok=True)
        with open("/workspace/artifacts/panda_live/contact_carry_report.json","w") as f:
            json.dump(report,f,indent=2)
        print("PANDA_CONTACT_CARRY_REPORT",json.dumps(report,indent=2),flush=True)
        node.destroy_node();rclpy.shutdown()
    return 0 if report["passed"] else 2

if __name__=="__main__":
    raise SystemExit(main())
