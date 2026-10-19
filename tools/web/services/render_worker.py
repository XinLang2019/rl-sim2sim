"""Lazily attach a non-colliding Gazebo camera; keep image IO off the control path."""
import math
from pathlib import Path
import threading
import time
from render_config import camera_defaults, validate_camera, camera_pose
from render_frames import LatestFrame


class RenderBridge:
    def __init__(self,directory,resolved,shutdown,emit):
        import rospy
        import cv2
        from cv_bridge import CvBridge
        from gazebo_msgs.srv import SpawnModel
        from geometry_msgs.msg import Pose
        from sensor_msgs.msg import Image
        self.rospy,self.cv2,self.cv,self.Image=rospy,cv2,CvBridge(),Image
        self.SpawnModel=SpawnModel
        self.pose_publisher=rospy.Publisher('/rlsim_viewer/camera_pose',Pose,queue_size=1,tcp_nodelay=True)
        self.directory=Path(directory)/'view';self.directory.mkdir(exist_ok=True)
        self.resolved,self.shutdown,self.emit=resolved,shutdown,emit
        self.lock=threading.Lock();self.frames=LatestFrame();self.epoch=0
        self.config=camera_defaults(resolved['experiment']['spawn']['position']);self.config['enabled']=False
        self.position=None;self.seq=0;self.subscriber=None;self.spawned=False;self.last_pose=None
        self.last_frame=0.;self.last_request=0.;self.frame_stamp=None;self.error='';self.last_status=0.;self.encode_ms=None
        self.encoder=threading.Thread(target=self.encode,daemon=True);self.encoder.start()
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()

    def request(self,value):
        config=validate_camera(value)
        with self.lock:
            if config['enabled']!=self.config['enabled']:self.epoch+=1
            self.config=config;self.last_request=time.monotonic()
        if not config['enabled']:self.frames.clear()

    def model(self,position):
        with self.lock:self.position=list(position)

    def status(self,state,error=''):
        with self.lock:
            self.error=error
            packet={'schema_version':1,'type':'render_status','state':state,'error':error,
                   'sequence':self.seq,'frame_received_at':self.last_frame,'simulation_time':self.frame_stamp,'width':960,'height':540,
                    'encode_ms':self.encode_ms,'target_fps':30,'transport':'gazebo-camera-jpeg','wall_time':time.time()}
            self.last_status=time.monotonic()
        self.emit(packet)

    def image(self,message):
        # ROS callbacks only replace one pending frame, never wait for JPEG or IO.
        with self.lock:
            if not self.config['enabled'] or time.monotonic()-self.last_request>=5:return
            epoch=self.epoch
        self.frames.put((epoch,message))

    def encode(self):
        while not self.shutdown.is_set():
            pending=self.frames.take()
            if pending is None:continue
            epoch,message=pending
            try:
                start=time.perf_counter()
                image=self.cv.imgmsg_to_cv2(message,desired_encoding='bgr8')
                ok,encoded=self.cv2.imencode('.jpg',image,[self.cv2.IMWRITE_JPEG_QUALITY,82])
                if not ok:raise ValueError('JPEG encoding failed')
                elapsed=(time.perf_counter()-start)*1000
                with self.lock:
                    if epoch!=self.epoch or not self.config['enabled'] or time.monotonic()-self.last_request>=5:continue
                    tmp=self.directory/'frame.tmp';tmp.write_bytes(encoded.tobytes());tmp.replace(self.directory/'frame.jpg')
                    self.last_frame=time.time();self.frame_stamp=message.header.stamp.to_sec();self.seq+=1;self.encode_ms=elapsed
                self.status('streaming')
            except Exception as error:self.status('failed','Camera image: '+str(error))

    def pose(self,config,position):
        from gazebo_msgs.msg import ModelState
        xyz,rpy=camera_pose(config,position)
        state=ModelState();state.model_name='rlsim_web_camera';state.reference_frame='world'
        state.pose.position.x,state.pose.position.y,state.pose.position.z=xyz
        pitch,yaw=rpy[1],rpy[2]
        state.pose.orientation.w=math.cos(pitch/2)*math.cos(yaw/2)
        state.pose.orientation.x=-math.sin(pitch/2)*math.sin(yaw/2)
        state.pose.orientation.y=math.sin(pitch/2)*math.cos(yaw/2)
        state.pose.orientation.z=math.cos(pitch/2)*math.sin(yaw/2)
        return state

    def run(self):
        try:
            while not self.shutdown.wait(1/30) and not self.rospy.is_shutdown():
                with self.lock:config=dict(self.config);position=self.position;last_request=self.last_request
                # A dropped browser cannot leave its camera rendering indefinitely.
                active=config['enabled'] and time.monotonic()-last_request<5
                if not active:
                    if self.subscriber:
                        self.subscriber.unregister();self.subscriber=None;self.frames.clear()
                        with self.lock:self.epoch+=1
                    if time.monotonic()-self.last_status>1:self.status('idle')
                    continue
                try:
                    if not self.spawned:
                        self.status('loading')
                        self.rospy.wait_for_service('/gazebo/spawn_sdf_model',timeout=3)
                        file=Path(self.resolved['workspace_root'])/'rlsim/interfaces/gazebo/rlsim_gazebo/models/web_camera.sdf'
                        sdf=file.read_text()
                        response=self.rospy.ServiceProxy('/gazebo/spawn_sdf_model',self.SpawnModel)(
                            model_name='rlsim_web_camera',model_xml=sdf,robot_namespace='/rlsim_viewer',
                            initial_pose=self.pose(config,position).pose,reference_frame='world')
                        if not response.success:raise RuntimeError(response.status_message)
                        self.spawned=True;self.spawned_at=time.monotonic()
                    if not self.pose_publisher.get_num_connections():
                        if time.monotonic()-self.spawned_at>3:
                            self.status('failed','观察相机位姿插件未连接，请重新构建 ROS 工作区。')
                            self.shutdown.wait(.5)
                        continue
                    if self.subscriber is None:
                        self.subscriber=self.rospy.Subscriber('/rlsim_viewer/view/image_raw',self.Image,self.image,queue_size=1,buff_size=4*1024*1024,tcp_nodelay=True)
                        self.status('loading')
                    state=self.pose(config,position)
                    pose=(state.pose.position.x,state.pose.position.y,state.pose.position.z,state.pose.orientation.w,state.pose.orientation.x,state.pose.orientation.y,state.pose.orientation.z)
                    if pose!=self.last_pose and self.pose_publisher.get_num_connections():
                        self.pose_publisher.publish(state.pose)
                        self.last_pose=pose
                    if time.monotonic()-self.last_status>1:
                        self.status('streaming' if time.time()-self.last_frame<3 else 'waiting',
                                    '' if self.last_frame else '等待 Gazebo 相机图像；检查 DISPLAY 和 OpenGL 图形环境')
                except Exception as error:
                    self.status('failed',str(error));self.shutdown.wait(1)
        finally:
            if self.subscriber:self.subscriber.unregister()
            self.frames.clear();self.encoder.join(timeout=1);self.pose_publisher.unregister()
