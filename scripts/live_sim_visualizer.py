#!/usr/bin/env python3
import math
import os
import re
import struct
import time
import zlib
from pathlib import Path

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from sensor_msgs.msg import JointState


ROOT = Path("/workspace")
MAP_PGM = ROOT / "ros2_ws/src/homeagent_navigation/maps/home_room.pgm"
MAP_YAML = ROOT / "ros2_ws/src/homeagent_navigation/maps/home_room.yaml"
OUT = ROOT / "artifacts/live_sim"
SCALE = 5


def read_pgm(path):
    data = path.read_bytes()
    i = 0
    def token():
        nonlocal i
        while i < len(data):
            if data[i:i+1] == b"#":
                while i < len(data) and data[i:i+1] not in (b"\n", b"\r"):
                    i += 1
            elif data[i:i+1].isspace():
                i += 1
            else:
                break
        start = i
        while i < len(data) and not data[i:i+1].isspace():
            i += 1
        return data[start:i].decode("ascii")
    if token() != "P5":
        raise RuntimeError("expected P5 PGM")
    w, h, maxv = int(token()), int(token()), int(token())
    if maxv != 255:
        raise RuntimeError("expected maxval 255")
    while i < len(data) and data[i:i+1].isspace():
        i += 1
    return w, h, data[i:i+w*h]


def map_meta(path):
    text = path.read_text()
    res = float(re.search(r"^resolution:\s*([^\s]+)", text, re.M).group(1))
    m = re.search(r"^origin:\s*\[([^\]]+)\]", text, re.M)
    origin = [float(x.strip()) for x in m.group(1).split(",")]
    return res, origin


def yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def setpx(img, w, h, x, y, c):
    if 0 <= x < w and 0 <= y < h:
        k = (y*w+x)*3
        img[k:k+3] = bytes(c)


def dot(img, w, h, x, y, r, c):
    for yy in range(y-r, y+r+1):
        for xx in range(x-r, x+r+1):
            if (xx-x)*(xx-x)+(yy-y)*(yy-y) <= r*r:
                setpx(img,w,h,xx,yy,c)


def line(img,w,h,x0,y0,x1,y1,c,t=1):
    dx=abs(x1-x0); sx=1 if x0<x1 else -1
    dy=-abs(y1-y0); sy=1 if y0<y1 else -1
    err=dx+dy
    while True:
        dot(img,w,h,x0,y0,t,c)
        if x0==x1 and y0==y1:
            break
        e2=2*err
        if e2>=dy:
            err+=dy; x0+=sx
        if e2<=dx:
            err+=dx; y0+=sy


def write_png(path,w,h,rgb):
    raw=bytearray()
    stride=w*3
    for y in range(h):
        raw.append(0)
        raw.extend(rgb[y*stride:(y+1)*stride])
    def chunk(k,p):
        return struct.pack(">I",len(p))+k+p+struct.pack(">I",zlib.crc32(k+p)&0xffffffff)
    out=bytearray(b"\x89PNG\r\n\x1a\n")
    out+=chunk(b"IHDR",struct.pack(">IIBBBBB",w,h,8,2,0,0,0))
    out+=chunk(b"IDAT",zlib.compress(bytes(raw),6))
    out+=chunk(b"IEND",b"")
    path.write_bytes(out)


class Viz(Node):
    def __init__(self):
        super().__init__("homeagent_live_sim_visualizer")
        self.amcl=None; self.odom=None; self.models=None; self.plan=None
        self.joints={}
        self.create_subscription(PoseWithCovarianceStamped,"/amcl_pose",self._amcl,10)
        self.create_subscription(Odometry,"/odom",self._odom,10)
        self.create_subscription(ModelStates,"/gazebo/model_states",self._models,10)
        self.create_subscription(NavPath,"/plan",self._plan,10)
        self.create_subscription(JointState,"/joint_states",self._joints,20)
    def _amcl(self,m): self.amcl=m
    def _odom(self,m): self.odom=m
    def _models(self,m): self.models=m
    def _plan(self,m): self.plan=m
    def _joints(self,m):
        for n,v in zip(m.name,m.position): self.joints[n]=float(v)


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    mw,mh,gray=read_pgm(MAP_PGM)
    res,origin=map_meta(MAP_YAML)
    w,h=mw*SCALE,mh*SCALE
    base=bytearray(w*h*3)
    for y in range(mh):
        for x in range(mw):
            v=gray[y*mw+x]
            for sy in range(SCALE):
                for sx in range(SCALE):
                    k=((y*SCALE+sy)*w+x*SCALE+sx)*3
                    base[k:k+3]=bytes((v,v,v))
    def wp(x,y):
        gx=int(round((x-origin[0])/res)); gy=int(round((y-origin[1])/res))
        return gx*SCALE+SCALE//2,(mh-1-gy)*SCALE+SCALE//2

    rclpy.init()
    n=Viz()
    start=time.monotonic()
    frame=0
    try:
        while time.monotonic()-start < 90:
            rclpy.spin_once(n,timeout_sec=0.1)
            if frame % 5 != 0:
                frame += 1
                continue
            img=bytearray(base)

            if n.plan and len(n.plan.poses)>1:
                pts=[wp(p.pose.position.x,p.pose.position.y) for p in n.plan.poses]
                for a,b in zip(pts,pts[1:]):
                    line(img,w,h,a[0],a[1],b[0],b[1],(0,180,255),1)

            # Dynamic pregrasp target from current cup observation/calibrated offset.
            cup_xy=None
            if n.models and "physical_cup" in n.models.name:
                i=n.models.name.index("physical_cup")
                cp=n.models.pose[i]
                cup_xy=(float(cp.position.x),float(cp.position.y))
                cx,cy=wp(*cup_xy)
                dot(img,w,h,cx,cy,8,(230,40,40))
                dot(img,w,h,cx,cy,3,(255,220,220))
                # Current calibrated target for near-zero cup yaw.
                tx=cup_xy[0]-0.5480166
                ty=cup_xy[1]-0.1403520
                px,py=wp(tx,ty)
                line(img,w,h,px-8,py,px+8,py,(40,200,40),2)
                line(img,w,h,px,py-8,px,py+8,(40,200,40),2)

            pose=None
            if n.amcl:
                pose=n.amcl.pose.pose
            elif n.odom:
                pose=n.odom.pose.pose
            if pose:
                x,y=float(pose.position.x),float(pose.position.y)
                a=yaw(pose.orientation)
                rx,ry=wp(x,y)
                dot(img,w,h,rx,ry,10,(30,90,230))
                hx=rx+int(22*math.cos(a)); hy=ry-int(22*math.sin(a))
                line(img,w,h,rx,ry,hx,hy,(20,20,20),2)

                # Simple arm top-view ray using joint1; length scaled for visibility.
                q1=n.joints.get("joint1",0.0)
                ax=rx+int(35*math.cos(a+q1))
                ay=ry-int(35*math.sin(a+q1))
                line(img,w,h,rx,ry,ax,ay,(255,145,0),3)
                dot(img,w,h,ax,ay,5,(255,145,0))

            latest=OUT/"sim_latest.png"
            write_png(latest,w,h,img)
            write_png(OUT/f"sim_{frame:04d}.png",w,h,img)
            frame += 1
            time.sleep(0.35)
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    main()
