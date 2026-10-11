#!/usr/bin/env python3
import os
import sys

import cv2
import numpy as np
import rospy
import torch
from cv_bridge import CvBridge
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
        self.show = rospy.get_param("~show", False)
        requested_device = rospy.get_param("~device", "cuda")

        self.bridge = CvBridge()
        self.device = self._select_device(requested_device)
        self.model = self._load_model()

        self.pub = rospy.Publisher(self.output_topic, Image, queue_size=1)
        msg_type = CompressedImage if self.compressed else Image
        self.sub = rospy.Subscriber(
            self.image_topic, msg_type, self.image_callback, queue_size=1, buff_size=2**24
        )

        rospy.loginfo("TwinLiteNet MORAI node ready")
        rospy.loginfo("subscribed: %s", self.image_topic)
        rospy.loginfo("publishing: %s", self.output_topic)
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

        has_module_prefix = any(k.startswith("module.") for k in state_dict.keys())
        if has_module_prefix:
            model = torch.nn.DataParallel(model)

        model.load_state_dict(state_dict)
        model.to(self.device)
        model.eval()
        rospy.loginfo("loaded weights: %s", self.weights)
        return model

    def image_callback(self, msg):
        try:
            if self.compressed:
                frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            overlay = self.run_inference(frame)
            out_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            out_msg.header = msg.header
            self.pub.publish(out_msg)

            if self.show:
                cv2.imshow("TwinLiteNet MORAI", overlay)
                cv2.waitKey(1)
        except Exception as exc:
            rospy.logerr_throttle(1.0, "TwinLiteNet inference failed: %s", exc)

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
        return overlay


def main():
    rospy.init_node("twinlitenet_morai_node")
    TwinLiteNetMoraiNode()
    rospy.spin()


if __name__ == "__main__":
    main()
