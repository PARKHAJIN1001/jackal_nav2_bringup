import rclpy,time,json,signal
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from rcl_interfaces.srv import GetParameters
from pathlib import Path
from collections import defaultdict
root=Path(__file__).resolve().parent
rclpy.init(); n=Node('perception_flow_observer'); f=open(root/'perception_flow.jsonl','a',buffering=1)
counts=defaultdict(int); latest={}; subscriptions=[]; active=True
def cb(t,m):
 counts[t]+=1
 latest[t]={'wall':time.time(),'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'objects':len(getattr(m,'detections',getattr(m,'tracks',getattr(m,'traces',[]))))}
for topic,typ in [('/ped_yolo/detections2d','vision_msgs/msg/Detection2DArray'),('/ped_detection','moai_nav_msgs/msg/Detections'),('/ped_tracking','moai_nav_msgs/msg/Tracks'),('/ped_traces','moai_nav_msgs/msg/Traces')]:
 subscriptions.append(n.create_subscription(get_message(typ),topic,lambda m,t=topic:cb(t,m),qos_profile_sensor_data))
client=n.create_client(GetParameters,'/cmd_vel_safety_bridge/get_parameters'); future=None; saved=False
started=time.time(); last=0
def stop(*_):
 global active
 active=False
signal.signal(signal.SIGINT,stop); signal.signal(signal.SIGTERM,stop)
try:
 while active and rclpy.ok() and not (root/'stop.monitor').exists():
  rclpy.spin_once(n,timeout_sec=.1); now=time.time()
  if future is None and client.service_is_ready():
   req=GetParameters.Request();req.names=['forward_cmd_vel'];future=client.call_async(req)
  if future is not None and future.done() and not saved:
   result=future.result();f.write(json.dumps({'wall':now,'kind':'motion_forwarding','values':[{'type':v.type,'bool_value':v.bool_value} for v in result.values]})+'\n');saved=True
  if now-last>=1:
   f.write(json.dumps({'wall':now,'kind':'flow','counts':dict(counts),'latest':latest})+'\n');last=now;counts.clear()
finally:
 f.close();n.destroy_node()
 if rclpy.ok():rclpy.shutdown()
