#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo as CamInfoMSG

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import numpy as np
import image_geometry
import tf

class FindEnemy:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/red_bot/image_raw",Image, self.image_cb)
    self.camerainfo_sub = rospy.Subscriber("/red_bot/camera_info",CamInfoMSG, self.camerainfo_cb)
    self.pub = rospy.Publisher("enemy_maker", Marker, queue_size = 10)

    self.listener = tf.TransformListener()


  def camerainfo_cb(self,data):
    self.camera_model = image_geometry.PinholeCameraModel()
    self.camera_model.fromCameraInfo(data)
        
    
  def find_red_ball(self, img):
    # define range of blue color in HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # define range of blue color in HSV
    lower_red = np.array([0,70,70])
    upper_red = np.array([10,255,255])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_red, upper_red)
    kernel = np.ones((9,9),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
               
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
       
    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # only proceed if the radius meets a minimum size. Correct this value for your obect's size
        if radius > 0.5:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(img, (int(x), int(y)), int(radius), (186,255,255), 2)
            cv2.imshow("Frame", img)
            cv2.waitKey(1)
        print("We found emeny!")
        return ((x, y), radius)
    # show the frame to our screen
    print("No emeny yet!")
    return ((0, 0), 0)

  def get_world_enmey_pose(self, u, v, radius):
      ray = self.camera_model.projectPixelTo3dRay((u, v))
      fy = self.camera_model.fy()
      d= 0.025 *fy/ radius
      enemy_pose_in_camera = np.array( [ray[2],-ray[0],-ray[1]])
      
      enemy_pose_in_camera = enemy_pose_in_camera/enemy_pose_in_camera[0] * d
      point_msg = PoseStamped()
      point_msg.pose.position.x= enemy_pose_in_camera[0]
      point_msg.pose.position.y= enemy_pose_in_camera[1]
      point_msg.pose.position.z= enemy_pose_in_camera[2]
      point_msg.pose.orientation.x=0
      point_msg.pose.orientation.y=0
      point_msg.pose.orientation.z=0
      point_msg.pose.orientation.w=1
      point_msg.header.stamp = rospy.Time.now()
      point_msg.header.frame_id = "red_bot/base_footprint"
      self.listener.waitForTransform("red_bot/base_footprint", "map", rospy.Time.now(), rospy.Duration(3.0))
      enemy_pose = self.listener.transformPose("map", point_msg)
      return (enemy_pose.pose.position.x, enemy_pose.pose.position.y, enemy_pose.pose.position.z)

  def pub_maker(self, pose):
      marker_data = Marker()
      marker_data.header.frame_id = "map"
      marker_data.header.stamp = rospy.Time.now()
      marker_data.type = marker_data.CUBE
      marker_data.ns = "basic_shapes"
      marker_data.id = 0

      marker_data.action = Marker.ADD

      marker_data.pose.position.x = pose[0]
      marker_data.pose.position.y = pose[1]
      marker_data.pose.position.z = pose[2]

      marker_data.pose.orientation.x=0.0
      marker_data.pose.orientation.y=0.0
      marker_data.pose.orientation.z=1.0
      marker_data.pose.orientation.w=0.0

      marker_data.color.r = 0.0
      marker_data.color.g = 1.0
      marker_data.color.b = 0.0
      marker_data.color.a = 1.0

      marker_data.scale.x = 0.1
      marker_data.scale.y = 0.1
      marker_data.scale.z = 0.1

      marker_data.lifetime = rospy.Duration()

      self.pub.publish(marker_data)

    
  def image_cb(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      ((u, v), radius) = self.find_red_ball(cv_image)
      if(radius == 0):
        return
      enemy_pose = self.get_world_enmey_pose(u, v, radius)
      self.pub_maker(enemy_pose)

    except CvBridgeError as e:
      import traceback
      traceback.print_exc()


def main(args):
  ic = FindEnemy()
  try:
    rospy.spin()
  except:
    print("Shutting down")

if __name__ == '__main__':
    rospy.init_node('get_enemy_pose', anonymous=True)
    main(sys.argv)