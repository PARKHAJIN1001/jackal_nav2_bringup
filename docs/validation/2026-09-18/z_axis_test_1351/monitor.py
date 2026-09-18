#!/usr/bin/env python3
"""Read-only compact TF/pose and input continuity recording; no cloud/image payloads."""
import json, time, math, signal
from pathlib import Path
from collections import defaultdict
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import PointCloud2, Imu, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from diagnostic_msgs.msg import DiagnosticArray
from tf2_ros import Buffer
from rclpy.duration import Duration

ROOT=Path(__file__).resolve().parent
alive=True
def stop(*_):
    global alive
    alive=False

def stamp(s): return s.sec+s.nanosec*1e-9
def pose(p,q):
    sinp=max(-1.,min(1.,2*(q.w*q.y-q.z*q.x)))
    return {'xyz':[p.x,p.y,p.z], 'xyzw':[q.x,q.y,q.z,q.w],
            'rpy':[math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y)),
                   math.asin(sinp),math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]}

class Monitor(Node):
    def __init__(self):
        super().__init__('z_axis_observer')
        self.out=open(ROOT/'samples.jsonl','a',buffering=1)
        self.count=defaultdict(int); self.prev={}; self.stats={}; self.buf=Buffer(cache_time=Duration(seconds=15))
        self.subs=[]; self.started=time.time(); self.last_graph=0
        for topic in ['/tf','/tf_static','/j100_0519/tf','/j100_0519/tf_static']:
            qos=QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL) if 'static' in topic else qos_profile_sensor_data
            self.subs.append(self.create_subscription(TFMessage,topic,lambda m,t=topic:self.tf(t,m),qos))
        for topic,typ in [('/livox/lidar',PointCloud2),('/livox/lidar_local',PointCloud2),('/livox/imu',Imu),('/scan',LaserScan),('/aft_mapped_to_init',Odometry),('/odom',Odometry),('/amcl_pose',PoseWithCovarianceStamped),('/initialpose',PoseWithCovarianceStamped)]:
            self.subs.append(self.create_subscription(typ,topic,lambda m,t=topic:self.msg(t,m),qos_profile_sensor_data))
        self.subs.append(self.create_subscription(DiagnosticArray,'/diagnostics',self.diag,qos_profile_sensor_data))
        self.subs.append(self.create_subscription(DiagnosticArray,'/nav2/lidar_relay_diagnostics',self.diag,qos_profile_sensor_data))
        self.create_timer(1.,self.tick)
        self.write('start',{})
    def write(self,kind,data):
        self.out.write(json.dumps({'wall':time.time(),'kind':kind,**data},allow_nan=False)+'\n')
    def tf(self,topic,msg):
        for t in msg.transforms:
            if topic == '/tf_static': self.buf.set_transform_static(t,'unattributed')
            elif topic == '/tf': self.buf.set_transform(t,'unattributed')
            self.write('tf',{'topic':topic,'parent':t.header.frame_id,'child':t.child_frame_id,'stamp':stamp(t.header.stamp),**pose(t.transform.translation,t.transform.rotation)})
    def msg(self,topic,msg):
        now=time.time(); s=stamp(msg.header.stamp); self.count[topic]+=1
        st=self.stats.setdefault(topic,{'n':0,'max_receipt_gap':0.,'min_age':1e9,'max_age':-1e9,'regressions':0})
        st['n']+=1; st['min_age']=min(st['min_age'],now-s); st['max_age']=max(st['max_age'],now-s)
        if topic in self.prev:
            old,old_s=self.prev[topic]; st['max_receipt_gap']=max(st['max_receipt_gap'],now-old); st['regressions']+=int(s<old_s)
        self.prev[topic]=(now,s)
        if isinstance(msg,(Odometry,PoseWithCovarianceStamped)):
            self.write('pose',{'topic':topic,'frame':msg.header.frame_id,'stamp':s,**pose(msg.pose.pose.position,msg.pose.pose.orientation),'covariance':list(msg.pose.covariance)})
        elif isinstance(msg,Imu) and self.count[topic]%5==0:
            self.write('imu',{'stamp':s,'a':[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z],'w':[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z]})
    def diag(self,msg):
        for s in msg.status:
            level=int.from_bytes(s.level,'little') if isinstance(s.level,bytes) else int(s.level)
            if 'relay' in s.name.lower() or level>=1:
                self.write('diagnostic',{'name':s.name,'level':level,'message':s.message,'values':{x.key:x.value for x in s.values}})
    def tick(self):
        counters={}
        lines=Path('/proc/net/snmp').read_text().splitlines()
        for h,v in zip(lines[::2],lines[1::2]):
            keys=h.split(); vals=v.split()
            counters[keys[0][:-1]]={k:int(x) for k,x in zip(keys[1:],vals[1:]) if k in ['ReasmReqds','ReasmOKs','ReasmFails','RcvbufErrors','InErrors']}
        self.write('health',{'topics':self.stats,'since_last':{k:time.time()-v[0] for k,v in self.prev.items()},'kernel':counters,'sockstat':Path('/proc/net/sockstat').read_text(),'ipfrag_high_thresh':Path('/proc/sys/net/ipv4/ipfrag_high_thresh').read_text().strip()})
        self.stats={}
        try:
            t=self.buf.lookup_transform('map','base_link',rclpy.time.Time())
            self.write('composed',{'parent':'map','child':'base_link','stamp':stamp(t.header.stamp),**pose(t.transform.translation,t.transform.rotation)})
        except Exception: pass
        if time.time()-self.last_graph>30:
            self.last_graph=time.time()
            self.write('graph',{'nodes':self.get_node_names_and_namespaces(),'topics':self.get_topic_names_and_types()})

def main():
    rclpy.init(); node=Monitor()
    signal.signal(signal.SIGINT,stop); signal.signal(signal.SIGTERM,stop)
    try:
        while alive and rclpy.ok() and not (ROOT/'stop.monitor').exists():
            rclpy.spin_once(node,timeout_sec=.2)
    finally:
        node.write('stop',{}); node.out.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__=='__main__': main()
