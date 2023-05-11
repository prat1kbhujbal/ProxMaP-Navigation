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

import bresenham
import roslib
# roslib.load_manifest('laser_to_image')
import numpy as np
import cv2
import math
import rospy
from sensor_msgs.msg import LaserScan, CompressedImage
# from cv_bridge import CvBridge, CvBridgeError
import torch

# Discretization Size
disc_size = .08
# Discretization Factor
disc_factor = 1 / disc_size
# Max Lidar Range
max_lidar_range = 10
# Create Image Size Using Range and Discretization Factor
image_size = int(max_lidar_range * 2 * disc_factor)


class laser_to_image:
    def __init__(self):
        # Laser Scan To Subscribe to
        self.joy_sub = rospy.Subscriber(
            '/scan', LaserScan, self.cloud_to_image_callback,queue_size=1)
        # Publisher for Image
        self.pub = rospy.Publisher(
            "/scan_to_image/compressed",
            CompressedImage,
            queue_size=1)
        self.rate = rospy.Rate(3)
        # self.pred_pub = rospy.Publisher("/scan_to_pred/compressed",CompressedImage, queue_size = 10)
        # # CvBridge Setup
        # # self.bridge = CvBridge()

        # self.device = torch.device('cpu')
        # pred_model = torch.load(f'/home/vishnu/Downloads/occmap_pred_models/clasfn_crossent_v1.pth', map_location=torch.device('cpu'))
        # self.clasfn_model = pred_model
        # self.clasfn_model.eval();

    def cloud_to_image_callback(self, scan):
        # Store maxAngle of lidar
        # print(scan.angle_max, scan.angle_min, np.radians(45))
        # image_size: 250
        # maxAngle: 0.7853981852531433
        # minAngle: -0.789761483669281
        # angleInc: 0.004363323096185923
        # maxLength: 30.0
        # num_pts: 362
        # image_size: 250

        # maxAngle = 0.7853981852531433# scan.angle_max
        # # Store minAngle of lidar
        # minAngle = -0.789761483669281# scan.angle_min
        # # Store angleInc of lidar
        # angleInc =  0.004363323096185923 #scan.angle_increment
        # # Store maxLength in lidar distances
        # maxLength = scan.range_max
        # # Store array of ranges
        # ranges = scan.ranges[332:-331]#scan.ranges
        # # Calculate the number of points in array of ranges
        # num_pts = len(ranges)
        # # Create Array for extracting X,Y points of each data point
        # xy_scan = np.zeros((num_pts,2))
        # # Create 3 Channel Blank Image
        # blank_image = np.zeros((image_size,image_size,3),dtype=np.uint8)

        maxAngle = scan.angle_max
        # Store minAngle of lidar
        minAngle = scan.angle_min
        # Store angleInc of lidar
        angleInc = scan.angle_increment
        # Store maxLength in lidar distances
        maxLength = 5.  # scan.range_max
        # Store array of ranges
        ranges = scan.ranges
        # Calculate the number of points in array of ranges
        num_pts = len(ranges)
        # Create Array for extracting X,Y points of each data point
        xy_scan = np.zeros((num_pts, 2))
        # Create 3 Channel Blank Image
        blank_image = np.zeros((image_size, image_size, 3), dtype=np.uint8)

        # print(f'maxAngle: {maxAngle}')
        # print(f'minAngle: {minAngle}')
        # print(f'angleInc: {angleInc}')
        # print(f'maxLength: {maxLength}')
        # print(f'maxLength: {maxLength}')
        # print(f'num_pts: {num_pts}')
        # print(f'image_size: {image_size}')

        blank_image[:, :, 1] = 255
        # blank_image[:,:,2] = 255
        # Loop through all points converting distance and angle to X,Y point
        for i in range(num_pts):
            # Check that distance is not longer than it should be
            if (ranges[i] > 10) or (math.isnan(ranges[i])):
                pass
            else:
                # Calculate angle of point and calculate X,Y position
                angle = minAngle + float(i) * angleInc
                xy_scan[i][0] = 5 * float(ranges[i] * math.cos(angle))
                xy_scan[i][1] = 5 * float(ranges[i] * math.sin(angle))

        # Loop through all points plot in blank_image
        for i in range(num_pts):
            pt_x = xy_scan[i, 0]
            pt_y = xy_scan[i, 1]
            if (pt_x < max_lidar_range) or (pt_x > -1 * (max_lidar_range - disc_size)
                                            ) or (pt_y < max_lidar_range) or (pt_y > -1 * (max_lidar_range - disc_size)):
                pix_x = int(math.floor((pt_x + max_lidar_range) * disc_factor))
                pix_y = int(math.floor((max_lidar_range - pt_y) * disc_factor))
                # print(f'IN: ({pt_x}, {pt_y})')
                l = bresenham.bresenham(
                    [image_size // 2, image_size // 2],
                    [pix_x, pix_y])
                for j in range(len(l.path)):
                    lpx = l.path[j][0]
                    lpy = l.path[j][1]
                    if (0 < lpx < image_size and 0 < lpy < image_size):
                        blank_image[lpy, lpx] = [255, 0, 0]

                # if (pix_x > image_size) or (pix_y > image_size):
                #     print ("Error")
                # else:
                if (0 < pix_y < image_size and 0 < pix_x < image_size):
                    blank_image[pix_y, pix_x] = [0, 0, 255]

        blank_img_np = np.asarray(blank_image)
        img_2_pub = np.zeros((256, 256, 3), dtype=np.uint8)
        img_2_pub[:, :, 1] = 255
        # print(blank_img_np.shape)
        half_size = image_size // 2
        img_2_pub[128 - half_size:128 + half_size,
                  :half_size, :] = blank_img_np[:, half_size:, :]

        # Convert CV2 Image to ROS Message
        # set ros rate

        img = CompressedImage()
        img.header.stamp = rospy.Time.now()
        img.format = 'jpeg'
        # img.data = np.array(cv2.imencode('.jpg',
        # blank_image)[1]).tobytes()#self.bridge.cv2_to_imgmsg(blank_image,
        # encoding="bgr8")
        # self.bridge.cv2_to_imgmsg(blank_image, encoding="bgr8")
        img.data = np.array(cv2.imencode('.jpg', img_2_pub)[1]).tobytes()
        # Publish image
        self.pub.publish(img)
        # self.rate.sleep()

        # occ_map_class = (img_2_pub > 125).astype(float)
        # occ_map_class2 = 0*occ_map_class.copy()
        # occ_map_class2[:,:10,1] = 1
        # occ_map_class2[:,10:,:] = occ_map_class[:,:256-10,:].copy()
        # clasfn_output = self.clasfn_model(torch.FloatTensor( (occ_map_class2[None,...]).transpose(0,3,1,2)).to(self.device))

        # final_output = clasfn_output[0].cpu().data.numpy().argmax(0)
        # final_output2 = np.ones(final_output.shape)
        # final_output2[:,:-10] = final_output[:,10:]
        # # print(final_output.shape)
        # # print(final_output)
        # pub_img = np.zeros((256, 256, 3), dtype=np.uint8)
        # pub_img[final_output2 == 0, 0] = 255
        # pub_img[final_output2 == 1, 1] = 255
        # pub_img[final_output2 == 2, 2] = 255

        # # print(pub_img)

        # pred_img = CompressedImage()
        # pred_img.header.stamp = rospy.Time.now()
        # pred_img.format = 'jpeg';
        # # pred_img.data = pub_img.tobytes()#self.bridge.cv2_to_imgmsg(blank_image, encoding="bgr8")
        # pred_img.data = np.array(cv2.imencode('.jpg', pub_img.astype(np.uint8))[1]).tobytes()
        # # Publish image
        # self.pred_pub.publish(pred_img)

        # Use CV to show image
        # cv2.imshow('result', blank_image), cv2.waitKey(3)
        # blank_image = np.zeros((image_size,image_size,3))


if __name__ == '__main__':
    rospy.init_node('laser_to_image')

    laser_to_image = laser_to_image()
    rospy.spin()
