#!/usr/bin/env python3
"""Latest-scan worker; M8-only 3D background/floor residual evidence."""
import os,sys,json,math,time,threading
from collections import deque
sys.path.insert(0,os.path.dirname(os.path.realpath(__file__)))
# Limit small BLAS plane fits before NumPy is imported; this is one node only.
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2,PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from mission_supervisor.msg import SectionState
from tunnel_map_geometry import StaticMap3D,ScanConfirmation,fields_array,classify_scan,registered_lookup_pose
from tunnel_region import read_region, contains
from obstacle_geometry import pose_at,regional_plane_heights

class Filter:
    def __init__(self):
        self.tunnel_region=read_region(rospy.get_param)
        self.enabled=rospy.get_param('~enabled',True) and rospy.get_param('/tunnel_recovery/enabled',True)
        self.map=None
        try:
            if self.enabled:self.map=StaticMap3D(rospy.get_param('~map'),rospy.get_param('~background_distance_m',.12),rospy.get_param('~floor_distance_m',.055))
        except Exception as e:rospy.logerr('[tunnel_map_filter] map unavailable: %s; UNKNOWN retained',e)
        self.poses=deque(maxlen=500);self.registration=None;self.active=False;self.section_time=-1.;self.pending=None
        self.cv=threading.Condition();self.confirm=ScanConfirmation(rospy.get_param("~min_points",3),rospy.get_param("~confirm_scans",2),rospy.get_param("~observation_memory_sec",.35))
        self.min_height=float(rospy.get_param("~min_height_m",.08));self.max_age=float(rospy.get_param("~max_scan_age_sec",.35))
        self.sensor_height=float(rospy.get_param('/lidar_extrinsic/z_from_ground',1.765))
        self.map_version=os.path.basename(rospy.get_param('~map','unknown'))
        self.sensor=np.array([rospy.get_param('~sensor_x',1.1),0.,rospy.get_param('~sensor_z',1.33)])
        self.pub=rospy.Publisher('/tunnel/map_classified',PointCloud2,queue_size=1)
        self.foreground=rospy.Publisher('/tunnel/map_foreground',PointCloud2,queue_size=1)
        self.background=rospy.Publisher('/tunnel/map_background',PointCloud2,queue_size=1)
        self.state=rospy.Publisher('/tunnel/map_state',String,queue_size=1)
        rospy.Subscriber('/tunnel/xicp/yaw_measurement',PoseStamped,self.registration_cb,queue_size=1)
        rospy.Subscriber('/odom',Odometry,self.odom,queue_size=100)
        rospy.Subscriber('/mission/active_section',SectionState,self.section,queue_size=1)
        rospy.Subscriber('/perception/points_gravity',PointCloud2,self.cloud,queue_size=1,buff_size=2**24)
        threading.Thread(target=self.worker,daemon=True).start()
    def odom(self,m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        h=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.poses.append((m.header.stamp.to_sec(),p.x,p.y,h,p.z))
    def registration_cb(self,m):
        stamp=m.header.stamp.to_sec();now=rospy.Time.now().to_sec()
        if not -.05<=now-stamp<=.3:return
        p=pose_at([(a[0],tuple(a[1:])) for a in list(self.poses)],stamp,.08)
        if p is None:return
        delta=np.array([m.pose.position.x-p[0],m.pose.position.y-p[1]])
        if np.isfinite(delta).all() and np.linalg.norm(delta)<=.6:
            # Lateral observable alignment is used only for map lookup. The
            # original pose/raw frame still owns obstacle coordinates. Yaw is
            # corrected through the existing adapter, never forced to the tunnel.
            self.registration=(stamp,delta)

    def section(self,m):self.active=m.index_valid and contains(self.tunnel_region,m.csv,m.index,prepare=True);self.section_time=rospy.Time.now().to_sec()
    def cloud(self,m):
        if self.enabled and self.active and rospy.Time.now().to_sec()-self.section_time<=1.:
            with self.cv:self.pending=m;self.cv.notify()
    def worker(self):
        while not rospy.is_shutdown():
            with self.cv:
                self.cv.wait_for(lambda:self.pending is not None,timeout=.2)
                if self.pending is None:continue
                msg=self.pending;self.pending=None
            try:self.process(msg)
            except Exception as e:rospy.logwarn_throttle(2.,'[tunnel_map_filter] %s',e)
    def process(self,msg):
        begin=time.monotonic();stamp=msg.header.stamp.to_sec();now=rospy.Time.now().to_sec()
        if not -.05<=now-stamp<=self.max_age or msg.header.frame_id!='velodyne_ground':return
        history=list(self.poses);pose=pose_at([(p[0],tuple(p[1:])) for p in history],stamp,.08)
        if pose is None:return
        # pose_at returns x/y/yaw, with optional z trailing in our history.
        px,py,h=pose[:3];z=float(np.interp(stamp,[p[0] for p in history],[p[4] for p in history]))
        xyz=fields_array(msg,['x','y','z']);xyz=xyz[np.isfinite(xyz).all(axis=1)]
        lookup_x,lookup_y=px,py
        registration=self.registration
        if registration is not None and -.05<=stamp-registration[0]<=.3:
            aligned=registered_lookup_pose(xyz,(px,py,z,h),self.map,registration[1],self.sensor)
            lookup_x,lookup_y=aligned[:2]
        world,bg,height,valid,inside,candidate=classify_scan(xyz,(lookup_x,lookup_y,z,h),self.map,self.confirm,self.sensor,self.min_height,sensor_height=self.sensor_height)
        confirmed=self.confirm.update(world,candidate,stamp)
        # Unsupported positive residual remains UNKNOWN and is never erased.
        label=np.where(bg,1.,np.where(confirmed,np.where(valid,3.,4.),np.where(candidate,2.,0.)))
        ground_sensor=xyz[:,2]-height
        a=np.column_stack([xyz,ground_sensor,label,np.where(valid,height,xyz[:,2]+self.sensor_height)]).astype('<f4')
        def publish(pub,mask=None):
            v=a if mask is None else a[mask];out=PointCloud2();out.header=msg.header;out.height=1;out.width=len(v)
            out.fields=[PointField(name=k,offset=4*i,datatype=PointField.FLOAT32,count=1) for i,k in enumerate(['x','y','z','map_ground','map_label','ground_relative'])]
            out.point_step=24;out.row_step=24*len(v);out.is_dense=False;out.data=v.tobytes();pub.publish(out)
        # Publish only fresh output. No receive-time stamp substitution.
        if rospy.Time.now().to_sec()-stamp>self.max_age:return
        publish(self.pub);publish(self.foreground,~bg&~inside&((valid&(height>=.08))|~valid));publish(self.background,bg)
        self.state.publish(String(data=json.dumps(dict(map_version=self.map_version,stamp=stamp,background=int(bg.sum()),candidate=int(candidate.sum()),confirmed=int(confirmed.sum()),confirmed_ground_unknown=int((confirmed&~valid).sum()),unknown=int((~bg&~valid).sum()),compute_ms=(time.monotonic()-begin)*1000,map_valid=self.map is not None,registration_age_sec=(stamp-registration[0] if registration is not None else None),lookup_delta_xy=[lookup_x-px,lookup_y-py]))))
if __name__=='__main__':
    rospy.init_node('tunnel_map_filter');Filter();rospy.spin()
