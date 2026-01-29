#!/usr/bin/env python3
import rospy
import torch
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from z_se_msgs.msg import SemanticObject, SemanticCloud
from sam3_wrapper import Sam3Predictor # 假设这是你的模型封装库

class SemanticExplorerNode:
    def __init__(self):
        rospy.init_node('z_se_perception')
        
        # 1. Load SAM 3 Model (Frozen Foundation Model)
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.predictor = Sam3Predictor(checkpoint="sam3_h.pth", device=self.device)
        
        # 2. Encode Target Prompt (Zero-Shot)
        # 获取用户指令，例如 "Find the fire extinguisher"
        self.target_text = rospy.get_param("~target_prompt", "fire extinguisher")
        self.target_embedding = self.predictor.encode_text(self.target_text)
        
        # 3. ROS Interfaces
        self.bridge = CvBridge()
        self.sub_rgb = rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_cb)
        self.sub_depth = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_cb)
        self.pub_objects = rospy.Publisher("/z_se/semantic_objects", SemanticCloud, queue_size=10)
        
        self.depth_img = None
        self.confidence_thresh = 0.4

    def depth_cb(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def rgb_cb(self, msg):
        if self.depth_img is None: return
        rgb_img = self.bridge.imgmsg_to_cv2(msg, "rgb8")

        # --- Core Logic: Zero-Shot Inference ---
        with torch.no_grad():
            # SAM 3 generates masks and visual embeddings
            masks, iou_preds, vis_embeddings = self.predictor.predict(rgb_img)
            
            # Compute Semantic Similarity (Eq. 3 in Paper)
            # Cosine similarity between Visual Embedding and Text Embedding
            sim_scores = torch.cosine_similarity(vis_embeddings, self.target_embedding)
            
            relevant_objects = SemanticCloud()
            relevant_objects.header = msg.header
            
            for i, score in enumerate(sim_scores):
                if score > self.confidence_thresh:
                    # Deproject mask centroid to 3D world frame
                    mask_binary = masks[i].cpu().numpy()
                    p3d = self.deproject_centroid(mask_binary, self.depth_img)
                    
                    if p3d is not None:
                        obj = SemanticObject()
                        obj.class_name = self.target_text
                        obj.score = score.item()
                        obj.position.x, obj.position.y, obj.position.z = p3d
                        relevant_objects.objects.append(obj)

            self.pub_objects.publish(relevant_objects)

    def deproject_centroid(self, mask, depth):
        # ... (Standard Pinhole Camera Model projection logic) ...
        pass

if __name__ == '__main__':
    node = SemanticExplorerNode()
    rospy.spin()