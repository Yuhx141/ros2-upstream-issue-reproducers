#!/usr/bin/env python3
import argparse,hashlib,json,math,os,signal,subprocess,threading,time,traceback
from pathlib import Path
import yaml

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point32,PolygonStamped,PoseStamped,PoseWithCovarianceStamped,TransformStamped,TwistStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState,GetState
from nav2_msgs.action import BackUp,ComputePathToPose,DriveOnHeading,FollowWaypoints,NavigateToPose,Spin,Wait
from nav2_msgs.msg import Costmap
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid,Odometry
from rclpy.action import ActionClient,ActionServer,CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy,QoSProfile,ReliabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty,Trigger
from tf2_ros import StaticTransformBroadcaster,TransformBroadcaster

def atomic_json(path,value):
 tmp=path.with_suffix(path.suffix+'.tmp');tmp.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n');tmp.replace(path)
def dur(x):return Duration(sec=int(x),nanosec=round((x%1)*1e9))
def close(a,b,t=.05):return abs(a-b)<=t
def check(name,passed,actual,expected):return {'name':name,'pass':bool(passed),'actual':actual,'expected':expected}
def result(item,title,checks,observations=None):
 passed=all(x['pass'] for x in checks)
 return {'item':item,'title':title,'status':'observed','pass':passed,'classification':'pass' if passed else 'violation','checks':checks,'observations':observations or {}}
def wait(f,timeout=8):
 end=time.monotonic()+timeout
 while not f.done() and time.monotonic()<end:time.sleep(.01)
 if not f.done():raise TimeoutError('ROS call timed out')
 return f.result()
def qos():return QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
def stop(proc,log=None):
 if proc and proc.poll() is None:
  os.killpg(proc.pid,signal.SIGTERM)
  try:proc.wait(5)
  except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
 if log:log.close()
def transition(node,name,code):
 client=node.create_client(ChangeState,f'/{name}/change_state');node.__dict__.setdefault('_p3_clients',[]).append(client);assert client.wait_for_service(6)
 req=ChangeState.Request();req.transition.id=code
 return wait(client.call_async(req),12).success
def launch(exe,root,name,config,remaps=()):
 root.mkdir(parents=True,exist_ok=True);path=root/f'{name}.yaml';path.write_text(yaml.safe_dump(config));log=(root/f'{name}.log').open('w')
 args=[exe,'--ros-args']
 for src,dst in remaps:args+=['-r',f'{src}:={dst}']
 args+=['--params-file',str(path)]
 return subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True),log
def package_version(name):return subprocess.check_output(['ros2','pkg','xml','-t','version',name],text=True).strip()

def item17(root,domain):
 os.environ['ROS_DOMAIN_ID']=str(domain);rclpy.init();node=rclpy.create_node(f'behavior_probe_{domain}');ex=MultiThreadedExecutor(3);ex.add_node(node);thread=threading.Thread(target=ex.spin);thread.start()
 config={'behavior_server':{'ros__parameters':{'cycle_frequency':20.,'behavior_plugins':['spin','backup','drive_on_heading','wait'],'spin':{'plugin':'nav2_behaviors::Spin'},'backup':{'plugin':'nav2_behaviors::BackUp'},'drive_on_heading':{'plugin':'nav2_behaviors::DriveOnHeading'},'wait':{'plugin':'nav2_behaviors::Wait'},'local_frame':'odom','global_frame':'map','robot_base_frame':'base_link','transform_tolerance':.2,'local_costmap_topic':'/behavior_costmap','local_footprint_topic':'/behavior_footprint','simulate_ahead_time':.5,'max_rotational_vel':1.,'min_rotational_vel':.2,'rotational_acc_lim':3.2,'enable_stamped_cmd_vel':True,'bond_heartbeat_period':0.0}}}
 proc,log=launch('/opt/ros/jazzy/lib/nav2_behaviors/behavior_server',root,'behavior',config)
 state={'x':0.,'y':0.,'yaw':0.,'v':0.,'w':0.,'commands':[],'last':time.monotonic()};lock=threading.Lock();tf=TransformBroadcaster(node)
 def cmd(m):
  with lock:state['v']=m.twist.linear.x;state['w']=m.twist.angular.z;state['commands'].append((state['v'],state['w']))
 node.create_subscription(TwistStamped,'/cmd_vel',cmd,10)
 def tick():
  with lock:
   now=time.monotonic();dt=now-state['last'];state['last']=now;state['x']+=state['v']*math.cos(state['yaw'])*dt;state['y']+=state['v']*math.sin(state['yaw'])*dt;state['yaw']+=state['w']*dt;x,y,a=state['x'],state['y'],state['yaw']
  t=TransformStamped();t.header.stamp=node.get_clock().now().to_msg();t.header.frame_id='odom';t.child_frame_id='base_link';t.transform.translation.x=x;t.transform.translation.y=y;t.transform.rotation.z=math.sin(a/2);t.transform.rotation.w=math.cos(a/2);tf.sendTransform(t)
 node.create_timer(.01,tick);costpub=node.create_publisher(Costmap,'/behavior_costmap',qos());footpub=node.create_publisher(PolygonStamped,'/behavior_footprint',qos())
 def publish_cost(value):
  m=Costmap();m.header.frame_id='odom';m.header.stamp=node.get_clock().now().to_msg();m.metadata.resolution=.1;m.metadata.size_x=100;m.metadata.size_y=100;m.metadata.origin.position.x=-5.;m.metadata.origin.position.y=-5.;m.metadata.origin.orientation.w=1.;m.data=[value]*10000;costpub.publish(m)
  f=PolygonStamped();f.header.frame_id='base_link';f.header.stamp=node.get_clock().now().to_msg();f.polygon.points=[Point32(x=x,y=y) for x,y in [(-.1,-.1),(-.1,.1),(.1,.1),(.1,-.1)]];footpub.publish(f)
 def run(kind,goal,feedback=None,timeout=8):
  name={Wait:'wait',Spin:'spin',BackUp:'backup',DriveOnHeading:'drive_on_heading'}[kind];a=ActionClient(node,kind,'/'+name);assert a.wait_for_server(5);h=wait(a.send_goal_async(goal,feedback_callback=feedback));assert h.accepted;return wait(h.get_result_async(),timeout)
 try:
  publish_cost(0);assert transition(node,'behavior_server',Transition.TRANSITION_CONFIGURE);publish_cost(0);assert transition(node,'behavior_server',Transition.TRANSITION_ACTIVATE);publish_cost(0)
  feedback=[];start=time.monotonic();wr=run(Wait,Wait.Goal(time=dur(.25)),lambda m:feedback.append(m.feedback.time_left.sec+m.feedback.time_left.nanosec/1e9));wait_elapsed=time.monotonic()-start
  a=ActionClient(node,Wait,'/wait');assert a.wait_for_server(3);h=wait(a.send_goal_async(Wait.Goal(time=dur(2.))));time.sleep(.15);wait(h.cancel_goal_async());cancel=wait(h.get_result_async())
  with lock:state['commands'].clear();x0=state['x']
  g=DriveOnHeading.Goal();g.target.x=.3;g.speed=.2;g.time_allowance=dur(3);drive=run(DriveOnHeading,g);drive_delta=state['x']-x0;drive_cmd=list(state['commands'])
  with lock:state['commands'].clear();x0=state['x']
  g=BackUp.Goal();g.target.x=.2;g.speed=.15;g.time_allowance=dur(3);backup=run(BackUp,g);backup_delta=state['x']-x0;backup_cmd=list(state['commands'])
  with lock:state['commands'].clear();a0=state['yaw']
  spin=run(Spin,Spin.Goal(target_yaw=.5,time_allowance=dur(3)));spin_delta=state['yaw']-a0;spin_cmd=list(state['commands'])
  publish_cost(254);time.sleep(.15);g=DriveOnHeading.Goal();g.target.x=.3;g.speed=.2;g.time_allowance=dur(3);collision=run(DriveOnHeading,g)
  publish_cost(0);time.sleep(.1);g=DriveOnHeading.Goal();g.target.x=1.;g.speed=.01;g.time_allowance=dur(.2);timeout=run(DriveOnHeading,g)
 finally:
  ex.shutdown();thread.join();node.destroy_node();rclpy.shutdown();stop(proc,log)
 observations={'wait_elapsed':wait_elapsed,'wait_feedback':feedback,'drive_delta':drive_delta,'backup_delta':backup_delta,'spin_delta':spin_delta,'drive_commands':drive_cmd,'backup_commands':backup_cmd,'spin_commands':spin_cmd}
 checks=[
  check('wait_duration_feedback',wr.status==GoalStatus.STATUS_SUCCEEDED and close(wait_elapsed,.25,.12) and feedback and feedback[0]>feedback[-1],{'status':wr.status,'elapsed':wait_elapsed,'feedback':feedback},'succeeded, 0.25s ±0.12, decreasing feedback'),
  check('wait_cancel',cancel.status==GoalStatus.STATUS_CANCELED,cancel.status,GoalStatus.STATUS_CANCELED),
  check('drive_distance_direction',drive.status==GoalStatus.STATUS_SUCCEEDED and close(drive_delta,.3,.06) and any(v>0 for v,_ in drive_cmd),{'status':drive.status,'delta':drive_delta},'success, +0.3m ±0.06'),
  check('backup_distance_direction',backup.status==GoalStatus.STATUS_SUCCEEDED and close(backup_delta,-.2,.06) and any(v<0 for v,_ in backup_cmd),{'status':backup.status,'delta':backup_delta},'success, -0.2m ±0.06'),
  check('spin_angle_direction',spin.status==GoalStatus.STATUS_SUCCEEDED and close(spin_delta,.5,.08) and any(w>0 for _,w in spin_cmd),{'status':spin.status,'delta':spin_delta},'success, +0.5rad ±0.08'),
  check('collision_abort',collision.status==GoalStatus.STATUS_ABORTED and collision.result.error_code==723,{'status':collision.status,'error':collision.result.error_code},'aborted, COLLISION_AHEAD=723'),
  check('time_allowance_abort',timeout.status==GoalStatus.STATUS_ABORTED and timeout.result.error_code==721,{'status':timeout.status,'error':timeout.result.error_code},'aborted, TIMEOUT=721')]
 return result(17,'Behavior Server action semantics',checks,observations)

def item18(root,domain):
 os.environ['ROS_DOMAIN_ID']=str(domain);rclpy.init();fake=rclpy.create_node(f'waypoint_fake_{domain}');client=rclpy.create_node(f'waypoint_probe_{domain}');received=[];nav_done={}
 def pose(x):
  p=PoseStamped();p.header.frame_id='map';p.pose.position.x=float(x);p.pose.orientation.w=1.;return p
 def execute(g):
  x=g.request.pose.pose.position.x;received.append(x);r=NavigateToPose.Result()
  if x==2.:r.error_code=42;g.abort()
  else:g.succeed()
  nav_done[x]=time.monotonic();return r
 server=ActionServer(fake,NavigateToPose,'navigate_to_pose',execute_callback=execute);ex=MultiThreadedExecutor(4);ex.add_node(fake);ex.add_node(client);thread=threading.Thread(target=ex.spin);thread.start();processes=[]
 def stack(name,stop_on_failure,plugin,extra):
  config={name:{'ros__parameters':{'stop_on_failure':stop_on_failure,'loop_rate':20,'waypoint_task_executor_plugin':plugin,plugin:{'plugin':f'nav2_waypoint_follower::{"WaitAtWaypoint" if plugin=="wait" else "InputAtWaypoint"}',**extra},'bond_heartbeat_period':0.0}}}
  proc,log=launch('/opt/ros/jazzy/lib/nav2_waypoint_follower/waypoint_follower',root,name,config,[('__node',name),('follow_waypoints',f'/{name}/follow_waypoints')]);processes.append((proc,log));time.sleep(.3);assert transition(client,name,Transition.TRANSITION_CONFIGURE);assert transition(client,name,Transition.TRANSITION_ACTIVATE)
  action=ActionClient(client,FollowWaypoints,f'/{name}/follow_waypoints');assert action.wait_for_server(5);return action
 def run(action,xs):
  goal=FollowWaypoints.Goal();goal.poses=[pose(x) for x in xs];start=time.monotonic();h=wait(action.send_goal_async(goal));assert h.accepted;r=wait(h.get_result_async(),8);return r,time.monotonic()-start
 try:
  wait_action=stack('wf_wait',False,'wait',{'enabled':True,'waypoint_pause_duration':0});received.clear();continued,_=run(wait_action,[1.,2.,3.]);continued_received=list(received)
  params=AsyncParameterClient(client,'/wf_wait');assert params.wait_for_services(5);responses=wait(params.set_parameters([Parameter('stop_on_failure',value=True)])).results
  received.clear();stopped,_=run(wait_action,[1.,2.,3.]);stopped_received=list(received)
  proc,log=processes.pop();stop(proc,log);timeout_rows={};input_success=None
  for suffix,value,x in [('02',.2,20.),('10',1.,21.),('12',1.2,22.)]:
   name='wf_input'+suffix;action=stack(name,True,'input',{'enabled':True,'timeout':value,'input_topic':f'/{name}/input'});received.clear();r,total=run(action,[x]);timeout_rows[str(value)]={'status':r.status,'missed':[(m.index,m.error_code) for m in r.result.missed_waypoints],'total_elapsed':total,'post_nav_elapsed':time.monotonic()-nav_done[x]}
   if value==1.2:
    input_topic=f'/{name}/input';pub=client.create_publisher(EmptyMsg,input_topic,10);end=time.monotonic()+3
    while client.count_subscribers(input_topic)==0 and time.monotonic()<end:time.sleep(.02)
    sending=[True]
    def send_input():
     while sending[0]:pub.publish(EmptyMsg());time.sleep(.02)
    input_thread=threading.Thread(target=send_input);input_thread.start();received.clear();input_success,_=run(action,[23.]);sending[0]=False;input_thread.join()
   proc,log=processes.pop();stop(proc,log)
 finally:
  for proc,log in processes:stop(proc,log)
  ex.shutdown();thread.join();server.destroy();fake.destroy_node();client.destroy_node();rclpy.shutdown()
 t02=timeout_rows['0.2']['post_nav_elapsed'];t10=timeout_rows['1.0']['post_nav_elapsed'];t12=timeout_rows['1.2']['post_nav_elapsed']
 checks=[
  check('continue_after_navigation_failure',continued.status==GoalStatus.STATUS_SUCCEEDED and continued_received==[1.,2.,3.] and [(m.index,m.error_code) for m in continued.result.missed_waypoints]==[(1,42)],{'status':continued.status,'received':continued_received,'missed':[(m.index,m.error_code) for m in continued.result.missed_waypoints]},'succeeded; visits all; missed [(1,42)]'),
  check('dynamic_stop_on_failure',all(x.successful for x in responses) and stopped.status==GoalStatus.STATUS_ABORTED and stopped_received==[1.,2.] and [(m.index,m.error_code) for m in stopped.result.missed_waypoints]==[(1,42)],{'parameter':[(x.successful,x.reason) for x in responses],'status':stopped.status,'received':stopped_received},'parameter accepted; aborts at second waypoint'),
  check('input_task_success',input_success.status==GoalStatus.STATUS_SUCCEEDED and not input_success.result.missed_waypoints,{'status':input_success.status,'missed':[(m.index,m.error_code) for m in input_success.result.missed_waypoints]},'succeeded with no missed waypoint'),
  check('input_timeout_error_code',all(row['status']==GoalStatus.STATUS_ABORTED and row['missed']==[(0,601)] for row in timeout_rows.values()),timeout_rows,'all aborted with task executor failed=601'),
  check('fractional_timeout_0_2',.15<=t02<=.55,t02,'0.2 seconds within scheduler tolerance [.15,.55]'),
  check('fractional_timeout_1_0',.85<=t10<=1.35,t10,'1.0 seconds within scheduler tolerance [.85,1.35]'),
  check('fractional_timeout_1_2',1.05<=t12<=1.55 and t12-t10>=.12,{'elapsed':t12,'delta_from_1.0':t12-t10},'1.2 seconds and at least 0.12s longer than 1.0')]
 return result(18,'Waypoint Follower sequence and task semantics',checks,{'timeouts':timeout_rows})

def lifecycle_config(nodes):
 return {
  'behavior_server':{'behavior_server':{'ros__parameters':{'behavior_plugins':['wait'],'wait':{'plugin':'nav2_behaviors::Wait'},'cycle_frequency':20.,'bond_heartbeat_period':.1}}},
  'waypoint_follower':{'waypoint_follower':{'ros__parameters':{'stop_on_failure':False,'waypoint_task_executor_plugin':'wait','wait':{'plugin':'nav2_waypoint_follower::WaitAtWaypoint','enabled':False},'bond_heartbeat_period':.1}}},
  'manager':{'lifecycle_manager':{'ros__parameters':{'node_names':nodes,'autostart':False,'bond_timeout':.5,'attempt_respawn_reconnection':False}}}}

def lifecycle_phase(root,domain,crash=False,settle_delay=0.,observe_wait=24.):
 os.environ['ROS_DOMAIN_ID']=str(domain);rclpy.init();node=rclpy.create_node(f'lifecycle_probe_{domain}');ex=MultiThreadedExecutor(2);ex.add_node(node);thread=threading.Thread(target=ex.spin);thread.start();cfg=lifecycle_config(['behavior_server','waypoint_follower']);procs={};logs={}
 for name,exe in [('behavior_server','/opt/ros/jazzy/lib/nav2_behaviors/behavior_server'),('waypoint_follower','/opt/ros/jazzy/lib/nav2_waypoint_follower/waypoint_follower'),('manager','/opt/ros/jazzy/lib/nav2_lifecycle_manager/lifecycle_manager')]:procs[name],logs[name]=launch(exe,root,name,cfg[name])
 manage=node.create_client(ManageLifecycleNodes,'/lifecycle_manager/manage_nodes');states={name:node.create_client(GetState,f'/{name}/get_state') for name in ['behavior_server','waypoint_follower']};active=node.create_client(Trigger,'/lifecycle_manager/is_active')
 def command(code):assert manage.wait_for_service(6);return wait(manage.call_async(ManageLifecycleNodes.Request(command=code)),12).success
 def state(name,timeout=4):assert states[name].wait_for_service(6);return wait(states[name].call_async(GetState.Request()),timeout).current_state.label
 observation={}
 try:
  assert all(c.wait_for_service(6) for c in states.values());time.sleep(.6)
  observation['startup']=[command(0),state('behavior_server'),state('waypoint_follower')]
  if not crash:
   observation['pause']=[command(1),state('behavior_server'),state('waypoint_follower')]
   observation['resume']=[command(2),state('behavior_server'),state('waypoint_follower')]
   observation['reset']=[command(3),state('behavior_server'),state('waypoint_follower')]
  else:
   time.sleep(settle_delay);killed=procs['waypoint_follower'].pid;os.killpg(killed,signal.SIGKILL);procs['waypoint_follower'].wait();started=time.monotonic();time.sleep(observe_wait);observation['pre_cleanup_manager_log']=(root/'manager.log').read_text();survivor=state('behavior_server',3.)
   observation['crash']={'killed_pid':killed,'process_dead':procs['waypoint_follower'].poll() is not None,'survivor_state':survivor,'elapsed':time.monotonic()-started}
   assert active.wait_for_service(3);future=active.call_async(Trigger.Request())
   try:observation['manager_active']=wait(future,.5).success
   except TimeoutError:observation['manager_active']=None
 finally:
  for name,proc in procs.items():stop(proc,logs[name])
  ex.shutdown();thread.join();node.destroy_node();rclpy.shutdown()
 observation['manager_log']=(root/'manager.log').read_text()[-6000:]
 return observation

def bt_phase(root,domain):
 os.environ['ROS_DOMAIN_ID']=str(domain);rclpy.init();fake=rclpy.create_node(f'bt_fake_{domain}');client=rclpy.create_node(f'bt_probe_{domain}');root.mkdir(parents=True,exist_ok=True)
 def pose(x):
  p=PoseStamped();p.header.frame_id='map';p.header.stamp=client.get_clock().now().to_msg();p.pose.position.x=float(x);p.pose.orientation.w=1.;return p
 def tree(name,body):
  path=root/name;path.write_text(f'<root BTCPP_format="4" main_tree_to_execute="MainTree"><BehaviorTree ID="MainTree">{body}</BehaviorTree></root>');return str(path)
 trees={'success':tree('success.xml','<AlwaysSuccess/>'),'failure':tree('failure.xml','<AlwaysFailure/>'),'compute':tree('compute.xml','<ComputePathToPose goal="{goal}" path="{path}" planner_id="" error_code_id="{compute_path_error_code}"/>'),'recovery':tree('recovery.xml','<RecoveryNode number_of_retries="1"><ComputePathToPose goal="{goal}" path="{path}" planner_id="" error_code_id="{compute_path_error_code}"/><Wait wait_duration="0.1"/></RecoveryNode>'),'wait':tree('wait.xml','<Wait wait_duration="0.8"/>')}
 counts={};wait_calls=[]
 def compute_cb(g):
  x=g.request.goal.pose.position.x;counts[x]=counts.get(x,0)+1;time.sleep(.05);r=ComputePathToPose.Result()
  if x==5. or (x==6. and counts[x]==1):r.error_code=ComputePathToPose.Result.TIMEOUT;r.error_msg='synthetic timeout';g.abort()
  else:r.path.header.frame_id='map';r.path.poses=[pose(0),pose(x)];g.succeed()
  return r
 def wait_cb(g):
  seconds=g.request.time.sec+g.request.time.nanosec/1e9;wait_calls.append(seconds);end=time.monotonic()+seconds
  while time.monotonic()<end:
   if g.is_cancel_requested:g.canceled();return Wait.Result()
   time.sleep(.01)
  g.succeed();return Wait.Result()
 compute_server=ActionServer(fake,ComputePathToPose,'compute_path_to_pose',execute_callback=compute_cb);wait_server=ActionServer(fake,Wait,'wait',execute_callback=wait_cb,cancel_callback=lambda _:CancelResponse.ACCEPT);ex=MultiThreadedExecutor(5);ex.add_node(fake);ex.add_node(client);thread=threading.Thread(target=ex.spin);thread.start()
 static=StaticTransformBroadcaster(client);t=TransformStamped();t.header.stamp=client.get_clock().now().to_msg();t.header.frame_id='map';t.child_frame_id='base_link';t.transform.rotation.w=1.;static.sendTransform(t);odom=client.create_publisher(Odometry,'/odom',10);client.create_timer(.05,lambda:odom.publish(Odometry()))
 config={'bt_navigator':{'ros__parameters':{'navigators':['navigate_to_pose'],'navigate_to_pose':{'plugin':'nav2_bt_navigator::NavigateToPoseNavigator'},'global_frame':'map','robot_base_frame':'base_link','odom_topic':'/odom','bt_loop_duration':10,'default_server_timeout':200,'wait_for_service_timeout':2000,'error_code_names':['compute_path_error_code','wait_error_code'],'default_nav_to_pose_bt_xml':trees['success'],'bond_heartbeat_period':0.0}}};proc,log=launch('/opt/ros/jazzy/lib/nav2_bt_navigator/bt_navigator',root,'bt',config)
 try:
  assert transition(client,'bt_navigator',Transition.TRANSITION_CONFIGURE);assert transition(client,'bt_navigator',Transition.TRANSITION_ACTIVATE);action=ActionClient(client,NavigateToPose,'navigate_to_pose');assert action.wait_for_server(5)
  def run(x,xml,feedback=None,timeout=8):
   g=NavigateToPose.Goal();g.pose=pose(x);g.behavior_tree=xml;h=wait(action.send_goal_async(g,feedback_callback=feedback));assert h.accepted;return h,wait(h.get_result_async(),timeout)
  success=run(1.,trees['success'])[1];failure=run(2.,trees['failure'])[1];error=run(5.,trees['compute'])[1];feedback=[];recovery=run(6.,trees['recovery'],lambda m:feedback.append(m.feedback.number_of_recoveries))[1]
  g=NavigateToPose.Goal();g.pose=pose(3.);g.behavior_tree=trees['wait'];h=wait(action.send_goal_async(g));time.sleep(.15);wait(h.cancel_goal_async());cancel=wait(h.get_result_async())
  g1=NavigateToPose.Goal();g1.pose=pose(3.);g1.behavior_tree=trees['wait'];h1=wait(action.send_goal_async(g1));time.sleep(.15);g2=NavigateToPose.Goal();g2.pose=pose(4.);g2.behavior_tree=trees['wait'];h2=wait(action.send_goal_async(g2));old=wait(h1.get_result_async(),3);new=wait(h2.get_result_async(),3)
 finally:
  stop(proc,log);ex.shutdown();thread.join();compute_server.destroy();wait_server.destroy();fake.destroy_node();client.destroy_node();rclpy.shutdown()
 return {'success':[success.status,success.result.error_code],'failure':[failure.status,failure.result.error_code],'error':[error.status,error.result.error_code],'recovery':[recovery.status,recovery.result.error_code],'counts':counts,'feedback':feedback,'wait_calls':wait_calls,'cancel':[cancel.status,cancel.result.error_code],'preempt_old':[old.status,old.result.error_code],'preempt_new':[new.status,new.result.error_code]}

def item19(root,domain):
 early=lifecycle_phase(root/'lifecycle-early-crash',domain,True,0.,2.);crash=lifecycle_phase(root/'lifecycle-settled-crash',domain+1,True,1.,12.);sequence=lifecycle_phase(root/'lifecycle-sequence',domain+2,False);bt=bt_phase(root/'bt',domain+3)
 checks=[
  check('lifecycle_startup',sequence['startup']==[True,'active','active'],sequence['startup'],[True,'active','active']),
  check('lifecycle_pause',sequence['pause']==[True,'inactive','inactive'],sequence['pause'],[True,'inactive','inactive']),
  check('lifecycle_resume',sequence['resume']==[True,'active','active'],sequence['resume'],[True,'active','active']),
  check('lifecycle_reset',sequence['reset']==[True,'unconfigured','unconfigured'],sequence['reset'],[True,'unconfigured','unconfigured']),
  check('bond_early_crash_detection',early['crash']['process_dead'] and 'SERVER waypoint_follower IS DOWN' in early['pre_cleanup_manager_log'],early,'crash immediately after formation is reported within 2s'),
  check('bond_settled_crash_detection',crash['crash']['process_dead'] and 'SERVER waypoint_follower IS DOWN' in crash['pre_cleanup_manager_log'],crash,'after one-second settling, crash is reported within the observation window'),
  check('bond_crash_reset_completion',crash['crash']['survivor_state']=='unconfigured' and crash['manager_active'] is False,{'survivor_state':crash['crash']['survivor_state'],'manager_active_response':crash['manager_active'],'elapsed':crash['crash']['elapsed']},'surviving node unconfigured and manager inactive within 12s'),
  check('bt_success_failure',bt['success']==[GoalStatus.STATUS_SUCCEEDED,0] and bt['failure']==[GoalStatus.STATUS_ABORTED,0],{'success':bt['success'],'failure':bt['failure']},'success [4,0], failure [6,0]'),
  check('bt_error_propagation',bt['error']==[GoalStatus.STATUS_ABORTED,ComputePathToPose.Result.TIMEOUT],bt['error'],[GoalStatus.STATUS_ABORTED,ComputePathToPose.Result.TIMEOUT]),
  check('bt_recovery_retry',bt['recovery']==[GoalStatus.STATUS_SUCCEEDED,0] and bt['counts'].get(6.0)==2 and bt['wait_calls'].count(.1)>=1 and max(bt['feedback'] or [0])==1,bt,'two planner calls, one recovery, succeeds'),
  check('bt_cancel',bt['cancel'][0]==GoalStatus.STATUS_CANCELED,bt['cancel'],[GoalStatus.STATUS_CANCELED,0]),
  check('bt_same_tree_preemption',bt['preempt_old'][0]==GoalStatus.STATUS_ABORTED and bt['preempt_new'][0]==GoalStatus.STATUS_SUCCEEDED,{'old':bt['preempt_old'],'new':bt['preempt_new']},'old aborted, replacement succeeds')]
 return result(19,'Lifecycle Manager and BT Navigator semantics',checks,{'sequence':sequence,'early_crash':early,'settled_crash':crash,'bt':bt})

def item20(root,domain):
 os.environ['ROS_DOMAIN_ID']=str(domain);rclpy.init();node=rclpy.create_node(f'amcl_probe_{domain}');ex=MultiThreadedExecutor(2);ex.add_node(node);thread=threading.Thread(target=ex.spin);thread.start()
 config={'amcl':{'ros__parameters':{'alpha1':0.,'alpha2':0.,'alpha3':0.,'alpha4':0.,'alpha5':0.,'base_frame_id':'base_link','odom_frame_id':'odom','global_frame_id':'map','scan_topic':'scan','map_topic':'map','tf_broadcast':False,'min_particles':100,'max_particles':100,'resample_interval':1,'update_min_d':.25,'update_min_a':.2,'set_initial_pose':False,'laser_model_type':'likelihood_field','max_beams':20,'bond_heartbeat_period':0.0}}};proc,log=launch('/opt/ros/jazzy/lib/nav2_amcl/amcl',root,'amcl',config)
 change=node.create_client(ChangeState,'/amcl/change_state');nomotion=node.create_client(Empty,'/request_nomotion_update');map_pub=node.create_publisher(OccupancyGrid,'/map',qos());initial_pub=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10);scan_pub=node.create_publisher(LaserScan,'/scan',qos_profile_sensor_data);tf=TransformBroadcaster(node);static=StaticTransformBroadcaster(node);poses=[];pose_sub=node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',poses.append,10);odom_x=0.
 def stamp():return node.get_clock().now().to_msg()
 def transform(parent,child,x=0.,at=None):
  t=TransformStamped();t.header.stamp=at or stamp();t.header.frame_id=parent;t.child_frame_id=child;t.transform.translation.x=x;t.transform.rotation.w=1.;return t
 def send_tf(at=None):tf.sendTransform(transform('odom','base_link',odom_x,at))
 def map_msg(mark=False):
  m=OccupancyGrid();m.header.stamp=stamp();m.header.frame_id='map';m.info.resolution=.25;m.info.width=40;m.info.height=40;m.info.origin.position.x=-5.;m.info.origin.position.y=-5.;m.info.origin.orientation.w=1.;m.data=[0]*1600
  for x in range(40):m.data[x]=m.data[1560+x]=100
  for y in range(40):m.data[y*40]=m.data[y*40+39]=100
  if mark:m.data[20*40+24]=100
  return m
 def initial(frame,x,y=1.,yaw=.2):
  p=PoseWithCovarianceStamped();p.header.stamp=stamp();p.header.frame_id=frame;p.pose.pose.position.x=x;p.pose.pose.position.y=y;p.pose.pose.orientation.z=math.sin(yaw/2);p.pose.pose.orientation.w=math.cos(yaw/2);initial_pub.publish(p)
 def scan():
  at=stamp();send_tf(at);time.sleep(.05);s=LaserScan();s.header.stamp=at;s.header.frame_id='laser';s.angle_min=-1.;s.angle_max=1.;s.angle_increment=.1;s.range_min=.05;s.range_max=10.;s.ranges=[4.]*21;scan_pub.publish(s);time.sleep(.45)
 def xy(message):return [message.pose.pose.position.x,message.pose.pose.position.y]
 try:
  assert change.wait_for_service(6);assert transition(node,'amcl',Transition.TRANSITION_CONFIGURE);static.sendTransform(transform('base_link','laser'))
  for _ in range(3):map_pub.publish(map_msg());send_tf();time.sleep(.2)
  initial('map',1.);time.sleep(.2);assert transition(node,'amcl',Transition.TRANSITION_ACTIVATE);scan();initial_pose=xy(poses[-1]);n=len(poses);scan();stationary=[n,len(poses)]
  initial('odom',3.);time.sleep(.2);assert nomotion.wait_for_service(3);wait(nomotion.call_async(Empty.Request()));scan();wrong_frame=xy(poses[-1])
  initial('map',2.);time.sleep(.2);scan();reset_pose=xy(poses[-1]);n=len(poses);odom_x=.1;scan();below=[n,len(poses)];odom_x=.4;scan();above_pose=xy(poses[-1]);above_count=len(poses)
  n=len(poses);map_pub.publish(map_msg(True));time.sleep(.3);scan();map_update=[n,len(poses),xy(poses[-1])]
 finally:
  ex.shutdown();thread.join();node.destroy_node();rclpy.shutdown();stop(proc,log)
 expected_above=[2.+.4*math.cos(.2),1.+.4*math.sin(.2)]
 observations={'initial_pose':initial_pose,'stationary':stationary,'wrong_frame':wrong_frame,'reset_pose':reset_pose,'below_gate':below,'above_gate_count':above_count,'above_pose':above_pose,'expected_above':expected_above,'map_update':map_update}
 checks=[
  check('inactive_initial_pose_applied_on_activate',close(initial_pose[0],1.,1e-6) and close(initial_pose[1],1.,1e-6),initial_pose,[1.,1.]),
  check('stationary_scan_gated',stationary[0]==stationary[1],stationary,'pose publication count unchanged'),
  check('non_global_initial_pose_rejected',close(wrong_frame[0],1.,1e-6) and close(wrong_frame[1],1.,1e-6),wrong_frame,'unchanged [1,1] after odom-frame request'),
  check('active_global_initial_pose_applied',close(reset_pose[0],2.,1e-6) and close(reset_pose[1],1.,1e-6),reset_pose,[2.,1.]),
  check('translation_below_gate',below[0]==below[1],below,'pose publication count unchanged for 0.1m < 0.25m'),
  check('translation_above_gate_kinematics',above_count==below[1]+1 and close(above_pose[0],expected_above[0],.01) and close(above_pose[1],expected_above[1],.01),{'count':above_count,'pose':above_pose}, {'count':below[1]+1,'pose':expected_above}),
  check('map_update_rebuilds_sensor_path',map_update[1]==map_update[0]+1 and all(close(a,b,.01) for a,b in zip(map_update[2],above_pose)),map_update,'one no-motion update on replacement map, pose preserved')]
 return result(20,'AMCL pose, frame, update gate, and map semantics',checks,observations)

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--repetitions',type=int,default=3);parser.add_argument('--domain-base',type=int,default=180);args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True);script_hash=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
 manifest={'schema':'p3-nav2-upper-semantic-v1','started_at':time.strftime('%Y-%m-%dT%H:%M:%S%z'),'script':str(Path(__file__).resolve()),'script_sha256':script_hash,'repetitions':args.repetitions,'domain_base':args.domain_base,'rmw_implementation':os.environ.get('RMW_IMPLEMENTATION','rmw_fastrtps_cpp'),'packages':{name:package_version(name) for name in ['nav2_behaviors','nav2_waypoint_follower','nav2_lifecycle_manager','nav2_bt_navigator','nav2_amcl','bondcpp']}}
 atomic_json(args.output/'manifest.json',manifest);records=[];runners=[item17,item18,item19,item20]
 for repetition in range(args.repetitions):
  repeat_root=args.output/f'repeat-{repetition:02d}';repeat_root.mkdir(exist_ok=True)
  for offset,runner in enumerate(runners):
   item=17+offset
   try:record=runner(repeat_root/f'item-{item}',args.domain_base+repetition*16+offset*4)
   except Exception as error:record={'item':item,'title':runner.__name__,'status':'error','pass':False,'classification':'execution_error','error':f'{type(error).__name__}: {error}','traceback':traceback.format_exc()}
   record['repetition']=repetition;records.append(record);atomic_json(repeat_root/f'item-{item}.json',record);atomic_json(args.output/'progress.json',{'completed':len(records),'total':args.repetitions*len(runners),'pass':sum(x['pass'] for x in records),'violations':sum(x['classification']=='violation' for x in records),'errors':sum(x['classification']=='execution_error' for x in records)});print(json.dumps({'repetition':repetition,'item':item,'status':record['status'],'pass':record['pass']}),flush=True)
 summary={'schema':manifest['schema'],'records':records,'counts':{'total':len(records),'pass':sum(x['pass'] for x in records),'violation':sum(x['classification']=='violation' for x in records),'execution_error':sum(x['classification']=='execution_error' for x in records)},'completed_at':time.strftime('%Y-%m-%dT%H:%M:%S%z')};atomic_json(args.output/'summary.json',summary);print(json.dumps(summary['counts'],ensure_ascii=False));return 0 if summary['counts']['execution_error']==0 else 1

if __name__=='__main__':raise SystemExit(main())
