#!/usr/bin/env python3
import os
import sys
import threading

import cv2
import numpy as np
import rospy
import torch
from sensor_msgs.msg import CompressedImage, Image


class TwinLiteNetMoraiNode:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/image_jpeg/compressed")
        self.compressed = rospy.get_param("~compressed", True)
        self.repo_root = rospy.get_param("~repo_root", os.getcwd())
        self.weights = rospy.get_param(
            "~weights", os.path.join(self.repo_root, "pretrained", "best.pth")
        )
        self.output_topic = rospy.get_param("~output_topic", "/twinlitenet/overlay")
        self.lane_mask_topic = rospy.get_param(
            "~lane_mask_topic", "/twinlitenet/lane_mask"
        )
        self.show = rospy.get_param("~show", False)
        requested_device = rospy.get_param("~device", "cuda")

        self.device = self._select_device(requested_device)
        self.model = self._load_model()
        self.inference_lock = threading.Lock()

        self.pub = rospy.Publisher(self.output_topic, Image, queue_size=1)
        self.lane_mask_pub = rospy.Publisher(
            self.lane_mask_topic, Image, queue_size=1
        )
        msg_type = CompressedImage if self.compressed else Image
        self.sub = rospy.Subscriber(
            self.image_topic, msg_type, self.image_callback, queue_size=1, buff_size=2**24
        )

        rospy.loginfo("TwinLiteNet MORAI node ready")
        rospy.loginfo("subscribed: %s", self.image_topic)
        rospy.loginfo("publishing: %s", self.output_topic)
        rospy.loginfo("publishing lane mask: %s", self.lane_mask_topic)
        rospy.loginfo("device: %s", self.device)

    def _select_device(self, requested_device):
        requested_device = str(requested_device).lower()
        if requested_device == "cpu":
            return torch.device("cpu")
        if torch.cuda.is_available():
            return torch.device("cuda:0")
        rospy.logwarn("CUDA is not available. Falling back to CPU.")
        return torch.device("cpu")

    def _load_model(self):
        if self.repo_root not in sys.path:
            sys.path.insert(0, self.repo_root)

        from model.TwinLite import TwinLiteNet

        model = TwinLiteNet()
        checkpoint = torch.load(self.weights, map_location=self.device)

        state_dict = checkpoint
        if isinstance(checkpoint, dict) and "state_dict" in checkpoint:
            state_dict = checkpoint["state_dict"]

        if any(k.startswith("module.") for k in state_dict.keys()):
            state_dict = {
                k.replace("module.", "", 1): v for k, v in state_dict.items()
            }

        model.load_state_dict(state_dict)
        model.to(self.device)
        model.eval()
        rospy.loginfo("loaded weights: %s", self.weights)
        return model

    def image_callback(self, msg):
        if not self.inference_lock.acquire(False):
            return
        try:
            frame = self._msg_to_cv2(msg)
            overlay, lane_mask = self.run_inference(frame)
            out_msg = self._cv2_to_imgmsg(overlay)
            out_msg.header = msg.header
            self.pub.publish(out_msg)

            lane_mask_msg = self._mono_to_imgmsg(lane_mask)
            lane_mask_msg.header = msg.header
            self.lane_mask_pub.publish(lane_mask_msg)

            if self.show:
                cv2.imshow("TwinLiteNet MORAI", overlay)
                cv2.waitKey(1)
        except Exception as exc:
            rospy.logerr_throttle(1.0, "TwinLiteNet inference failed: %s", exc)
        finally:
            self.inference_lock.release()

    def _msg_to_cv2(self, msg):
        if self.compressed:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError("failed to decode compressed image")
            return frame

        dtype = np.uint8
        channels = 1
        if msg.encoding in ("bgr8", "rgb8"):
            channels = 3
        elif msg.encoding not in ("mono8", "8UC1"):
            raise ValueError("unsupported raw image encoding: {}".format(msg.encoding))

        frame = np.frombuffer(msg.data, dtype=dtype)
        if channels == 3:
            frame = frame.reshape(msg.height, msg.width, 3)
            if msg.encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            return frame
        frame = frame.reshape(msg.height, msg.width)
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

    def _cv2_to_imgmsg(self, frame):
        msg = Image()
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = msg.width * 3
        msg.data = np.ascontiguousarray(frame).tobytes()
        return msg

    def _mono_to_imgmsg(self, frame):
        msg = Image()
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = "mono8"
        msg.is_bigendian = False
        msg.step = msg.width
        msg.data = np.ascontiguousarray(frame).tobytes()
        return msg

    def run_inference(self, frame):
        img = cv2.resize(frame, (640, 360))
        overlay = img.copy()

        tensor = img[:, :, ::-1].transpose(2, 0, 1)
        tensor = np.ascontiguousarray(tensor)
        tensor = torch.from_numpy(tensor).unsqueeze(0).to(self.device).float() / 255.0

        with torch.no_grad():
            da_logits, ll_logits = self.model(tensor)

        _, da_predict = torch.max(da_logits, 1)
        _, ll_predict = torch.max(ll_logits, 1)

        da_mask = da_predict.byte().cpu().numpy()[0] > 0
        ll_mask = ll_predict.byte().cpu().numpy()[0] > 0

        overlay[da_mask] = [255, 0, 0]
        overlay[ll_mask] = [0, 255, 0]
        lane_mask = np.zeros(ll_mask.shape, dtype=np.uint8)
        lane_mask[ll_mask] = 255
        lane_mask = cv2.resize(lane_mask, (640, 360), interpolation=cv2.INTER_NEAREST)
        overlay = cv2.resize(overlay, (640, 360))
        return overlay, lane_mask


def main():
    rospy.init_node("twinlitenet_morai_node")
    TwinLiteNetMoraiNode()
    rospy.spin()


if __name__ == "__main__":
    main()
