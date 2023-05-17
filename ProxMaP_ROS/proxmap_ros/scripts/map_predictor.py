#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
#  Copyright (c) 2018, Benjamin Narin
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the OSU Personal Robotics Group. nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
# Author: Benjamin Narin

import torch
import numpy as np
import cv2
import rospy
import rospkg
import os
import sys
from sensor_msgs.msg import LaserScan, CompressedImage
# from cv_bridge import CvBridge, CvBridgeError

rospack = rospkg.RosPack()
package_path = rospack.get_path('proxmap_ros')

model_path = package_path + '/model'
sys.path.append(model_path)

model_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'model')

# Discretization Size
disc_size = .08
# Discretization Factor
disc_factor = 1 / disc_size
# Max Lidar Range
max_lidar_range = 10
# Create Image Size Using Range and Discretization Factor
image_size = int(max_lidar_range * 2 * disc_factor)


class map_predictor:
    def __init__(self):
        self.device = torch.device('cpu')
        # self.device = torch.device('cuda:0')

        pred_model = torch.load(
            model_path + f'/clasfn_crossent_v1.pth',
            map_location=self.device)
        self.clasfn_model = pred_model
        self.clasfn_model = self.clasfn_model.to(self.device)
        self.clasfn_model.eval()
        # self.rate = rospy.Rate(2)
        # Laser Scan To Subscribe to
        self.img_sub = rospy.Subscriber(
            "/scan_to_image/compressed",
            CompressedImage,
            self.image_callback, queue_size=1)
        # Publisher for Image
        self.pred_pub = rospy.Publisher(
            "/scan_to_pred/compressed",
            CompressedImage,
            queue_size=1)
        # CvBridge Setup
        # self.bridge = CvBridge()

    def image_callback(self, img):
        np_arr = np.frombuffer(img.data, np.uint8)
        # print(len(np_arr))
        # np_arr.reshape(256, 256, 3)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        occ_map_class = (image_np > 125).astype(float)
        occ_map_class2 = 0 * occ_map_class.copy()
        occ_map_class2[:, :10, 1] = 1
        occ_map_class2[:, 10:, :] = occ_map_class[:, :256 - 10, :].copy()
        input_tensor = torch.FloatTensor(
            (occ_map_class2[None, ...]).transpose(0, 3, 1, 2)).to(
            self.device)
        # print(input_tensor.dtype)
        clasfn_output = self.clasfn_model(input_tensor)

        final_output = clasfn_output[0].cpu().data.numpy().argmax(0)
        final_output2 = np.ones(final_output.shape)
        final_output2[:, :-10] = final_output[:, 10:]
        # print(final_output.shape)
        # print(final_output)
        pub_img = np.zeros((256, 256, 3), dtype=np.uint8)
        pub_img[final_output2 == 0, 0] = 255
        pub_img[final_output2 == 1, 1] = 255
        pub_img[final_output2 == 2, 2] = 255

        # # print(pub_img)

        pred_img = CompressedImage()
        pred_img.header.stamp = rospy.Time.now()
        pred_img.format = 'jpeg'
        # pred_img.data =
        # pub_img.tobytes()#self.bridge.cv2_to_imgmsg(blank_image,
        # encoding="bgr8")
        pred_img.data = np.array(cv2.imencode('.jpg', pub_img)[1]).tobytes()
        # Publish image
        self.pred_pub.publish(pred_img)
        # self.rate.sleep()
        # Use CV to show image
        # cv2.imshow('result', blank_image), cv2.waitKey(3)
        # blank_image = np.zeros((image_size,image_size,3))


if __name__ == '__main__':
    rospy.init_node('map_predictor')
    map_predictor = map_predictor()
    rospy.spin()
    # rospy.spin()
