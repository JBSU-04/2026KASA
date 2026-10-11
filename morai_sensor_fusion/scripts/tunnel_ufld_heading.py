#!/usr/bin/env python3
"""M8-only UFLD observation worker. Never publishes steering or position."""
import bisect
import json
import math
import threading
from collections import deque

import cv2
import numpy as np
import rospy
import rospkg
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import CompressedImage, Imu
from std_msgs.msg import String
from morai_sensor_fusion.lane_heading import ground_points, fit_heading, path_tangent, sample_rows


class Detector:
    def __init__(self):
        self.device = rospy.get_param('~device', 'cuda')
        lidar_z = rospy.get_param('/lidar_extrinsic/z', None)
        lidar_height = rospy.get_param('/lidar_extrinsic/z_from_ground', None)
        self.camera_height = rospy.get_param('~camera_height_from_ground_m')
        if lidar_z is not None and lidar_height is not None:
            self.camera_height = rospy.get_param('~camera_z_vehicle_m')+float(lidar_height)-float(lidar_z)
        rospy.loginfo('UFLD: camera ground height %.3fm; input %s',self.camera_height,rospy.get_param('~image_topic'))
        self.net = None  # Lazy-load only when M8 observations are requested.
        self.model_path = rospy.get_param('~weights', rospkg.RosPack().get_path(
            'morai_sensor_fusion')+'/models/tunnel_ufld_res18.pth')
        self.lock = threading.Lock()
        self.pending = None
        self.poses, self.attitudes = deque(maxlen=150), deque(maxlen=150)
        self.path = None
        self.mode, self.selection = '', None
        self.frames, self.previous = 0, None
        self.last_debug_stamp = 0.
        self.measurement = rospy.Publisher('/tunnel/lane_yaw_measurement', PoseStamped, queue_size=1)
        self.debug = rospy.Publisher('/tunnel/lane_heading/state', String, queue_size=1)
        self.image_pub = rospy.Publisher('/tunnel/lane_heading/image/compressed', CompressedImage, queue_size=1)
        rospy.Subscriber(rospy.get_param('~image_topic'), CompressedImage, self.image, queue_size=1, buff_size=2**22)
        rospy.Subscriber('/odom', Odometry, self.odom, queue_size=1)
        rospy.Subscriber('/imu', Imu, self.imu, queue_size=1)
        rospy.Subscriber('/global_path', Path, self.route, queue_size=1)
        rospy.Subscriber('/localization/gps_mode', String, self.gps, queue_size=1)
        rospy.Subscriber('/obstacle_avoidance/selection_state', String, self.plan, queue_size=1)
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()

    def gps(self, msg):
        with self.lock: self.mode = msg.data.lower()

    def plan(self, msg):
        try: value = json.loads(msg.data)
        except ValueError: value = None
        with self.lock: self.selection = (rospy.Time.now().to_sec(), value)

    def route(self, msg):
        with self.lock:
            self.path = np.array([(p.pose.position.x, p.pose.position.y) for p in msg.poses])

    def odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])[2]
        p = msg.pose.pose.position
        with self.lock: self.poses.append((msg.header.stamp.to_sec(), p.x,p.y,yaw,msg.twist.twist.linear.x))

    def imu(self, msg):
        q = msg.orientation
        r,p,_ = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
        with self.lock: self.attitudes.append((msg.header.stamp.to_sec(),r,p,msg.angular_velocity.z))

    def image(self, msg):
        with self.lock: self.pending = msg

    @staticmethod
    def sample(rows, stamp, angle_indices=()):
        return sample_rows(rows,stamp,angle_indices)

    def pixels(self, image):
        import torch
        if self.net is None:
            from morai_sensor_fusion.ufld_vendor.model import parsingNet
            torch.set_num_threads(1)
            net = parsingNet(pretrained=False, backbone='18', cls_dim=(201,24,4), use_aux=False)
            net.load_state_dict(torch.load(self.model_path,map_location='cpu',weights_only=True),strict=True)
            self.net = net.to(self.device).eval()
        rgb = cv2.cvtColor(cv2.resize(image,(800,288)),cv2.COLOR_BGR2RGB)
        tensor = torch.from_numpy(rgb.transpose(2,0,1).copy()).float()/255
        tensor = (tensor-torch.tensor([.485,.456,.406])[:,None,None])/torch.tensor([.229,.224,.225])[:,None,None]
        with torch.inference_mode(): out = self.net(tensor[None].to(self.device))[0].flip(1)
        bins = torch.arange(1,201,device=self.device)[:,None,None]
        loc = (out[:200].softmax(0)*bins).sum(0)
        loc[out.argmax(0)==200] = 0
        loc = loc.cpu().numpy()
        h,w = image.shape[:2]
        anchors = np.arange(144,283,6)[::-1]
        lanes = []
        for j in range(4):
            mask = loc[:,j]>0
            lanes.append(np.column_stack((loc[mask,j]*(799/199)*w/800-1,anchors[mask]*h/288-1)))
        self.detected_lanes = lanes
        # Ignore hallucinated continuations painted over the hood.
        cutoff = rospy.get_param("~max_lane_image_fraction", .78)*h
        lanes = [q[q[:,1]<=cutoff] for q in lanes]
        left,right = [],[]
        for points in lanes:
            if len(points)<6: continue
            bottom = float(np.median(points[np.argsort(points[:,1])[-3:],0]))
            (left if bottom<w/2 else right).append((bottom,points))
        return (max(left,key=lambda x:x[0])[1],min(right,key=lambda x:x[0])[1]) if left and right else None

    def process(self, msg, poses, attitudes, path, mode, selection):
        stamp = msg.header.stamp.to_sec()
        now = rospy.Time.now().to_sec()
        if not stamp>0 or not 0<=now-stamp<=rospy.get_param('~max_image_age_s',.3): return 'image_expired',None
        # Visualization is independent of localization correction acceptance.
        plan = selection[1] if selection is not None and selection[1] else {}
        if plan.get('mission_mode') != 'tunnel': return 'outside_m8',None
        image = cv2.imdecode(np.frombuffer(msg.data,np.uint8),cv2.IMREAD_COLOR)
        if image is None: return 'invalid_image',None
        self.visual_image, self.visual_header = image, msg.header
        lanes = self.pixels(image)
        self.visual_pair = lanes
        pose = self.sample(poses,stamp,(3,))
        attitude = self.sample(attitudes,stamp,(1,2))
        if pose is None: return 'odom_time_unavailable',None
        if attitude is None: return 'imu_time_unavailable',None
        if path is None or len(path)<2: return 'reference_path_unavailable',None
        if mode not in ('frozen','degraded','outage'): return 'gps_normal',None
        if selection is None or now-selection[0]>.3: return 'planner_expired',None
        if not plan.get('valid',False): return 'invalid_path',None
        if plan.get('trigger_count',1)>0 or plan.get('returning',False) or pose[3]<0: return 'avoidance_or_reverse',None
        if abs(attitude[2])>rospy.get_param('~max_yaw_rate_rad_s',.12): return 'turning',None
        ref = path_tangent(path,pose[0],pose[1],rospy.get_param('~preview_m',4.))
        if ref is None or ref[1]>rospy.get_param('~max_path_distance_m',.25): return 'off_reference_path',None
        if lanes is None: return 'missing_lane_pair',None
        if not rospy.get_param('~calibration_confirmed',False): return 'calibration_required',None
        h,w = image.shape[:2]
        ground = [ground_points(q,w,h,rospy.get_param('~camera_hfov_deg'),
                  self.camera_height,math.radians(rospy.get_param('~camera_pitch_down_deg')),
                  attitude[0],attitude[1],rospy.get_param('~camera_x_m')) for q in lanes]
        fit = fit_heading(*ground,preview=rospy.get_param('~preview_m'),
                          min_width=rospy.get_param('~min_lane_width_m'),max_width=rospy.get_param('~max_lane_width_m'),
                          max_rms=rospy.get_param('~max_fit_rms_m'))
        if fit is None: return 'geometry_rejected',None
        yaw = ref[0]-fit[0]
        error = math.atan2(math.sin(pose[2]-yaw),math.cos(pose[2]-yaw))
        if abs(error)>math.radians(3): return 'innovation_rejected',None
        if self.previous is not None:
            old_stamp,old_error = self.previous
            if stamp-old_stamp>.3 or abs(error-old_error)>math.radians(1): self.frames=0
        self.previous = stamp,error
        self.frames += 1
        if self.frames<rospy.get_param('~confirm_frames',3): return 'confirming',None
        observation = PoseStamped(header=msg.header)
        observation.header.frame_id='map'
        observation.pose.orientation.x,observation.pose.orientation.y,observation.pose.orientation.z,observation.pose.orientation.w = tf.transformations.quaternion_from_euler(0,0,yaw)
        self.measurement.publish(observation)
        return 'accepted',dict(width_m=fit[1],fit_rms_m=fit[2],bias_deg=math.degrees(error))

    def publish_visualization(self, reason):
        if (self.visual_image is None or not rospy.get_param('~publish_debug_image',False)
                or self.image_pub.get_num_connections() == 0): return
        stamp = self.visual_header.stamp.to_sec()
        if stamp-self.last_debug_stamp < .5: return
        self.last_debug_stamp = stamp
        image = self.visual_image.copy()
        # Thin blue: all model lanes. Yellow/cyan: selected ego boundaries.
        for points in self.detected_lanes:
            if len(points)>1: cv2.polylines(image,[points.astype(np.int32)],False,(255,120,40),2)
        if self.visual_pair is not None:
            for points,color in zip(self.visual_pair,[(0,255,255),(255,255,0)]):
                cv2.polylines(image,[points.astype(np.int32)],False,color,3)
                for u,v in points: cv2.circle(image,(int(u),int(v)),3,color,-1)
        color = (0,255,0) if reason=='accepted' else (0,180,255)
        cv2.rectangle(image,(0,0),(image.shape[1],70),(20,20,20),-1)
        cv2.putText(image,'M8 UFLD | heading: '+reason,(12,26),cv2.FONT_HERSHEY_SIMPLEX,.65,color,2)
        cv2.putText(image,'Blue: all lanes | Yellow: ego left | Cyan: ego right',(12,55),cv2.FONT_HERSHEY_SIMPLEX,.6,(240,240,240),1)
        encoded = cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,80])[1]
        self.image_pub.publish(CompressedImage(header=self.visual_header,format='jpeg',data=encoded.tobytes()))

    def run(self):
        rate = rospy.Rate(rospy.get_param('~max_rate_hz',10.))
        while not rospy.is_shutdown():
            with self.lock:
                msg,self.pending = self.pending,None
                snapshot = list(self.poses),list(self.attitudes),self.path,self.mode,self.selection
            if msg is not None:
                self.visual_image, self.visual_pair, self.detected_lanes = None,None,[]
                try: reason,extra = self.process(msg,*snapshot)
                except Exception as exc:
                    reason,extra = 'processing_error',{'error':str(exc)}
                    rospy.logerr_throttle(5.,'UFLD heading: %s',exc)
                if reason not in ('accepted','confirming'): self.frames,self.previous=0,None
                self.publish_visualization(reason)
                self.debug.publish(json.dumps(dict(reason=reason,stamp=msg.header.stamp.to_sec(),**(extra or {}))))
            rate.sleep()


if __name__=='__main__':
    rospy.init_node('tunnel_ufld_heading')
    Detector()
    rospy.spin()
