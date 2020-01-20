#!/usr/bin/env python
import numpy as np
import cv2
import rospy
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Empty, EmptyResponse

cnt = 0
image_rgb = None
image_depth = None
image_thermal = None
info_rgb = None
info_thermal = None
lock = False
br = CvBridge()

def callback(rgb_image, rgb_info, depth_image, thermal_image, thermal_info):
	global image_rgb, image_depth, image_thermal, info_rgb, info_thermal
	if not lock:
		image_rgb = br.imgmsg_to_cv2(rgb_image)
		image_rgb = cv2.cvtColor(image_rgb, cv2.COLOR_BGR2RGB)
		image_depth = br.imgmsg_to_cv2(depth_image)
		image_thermal = br.imgmsg_to_cv2(thermal_image)
		info_rgb = rgb_info
		info_thermal = thermal_info
		
def save_imgs_cb(req):
	global cnt
	lock = True
	cv2.imwrite("color_{:04d}.jpg".format(cnt), image_rgb)
	print image_depth.dtype
	cv2.imwrite("depth_{:04d}.png".format(cnt), image_depth)
	cv2.imwrite("thermal_{:04d}.jpg".format(cnt), image_thermal)
	fs = open("rgb_intrinsic_file.txt", "w")
	info_str = "rgb_intrinsic: [" + str(info_rgb.K[0]) + ", " + str(info_rgb.K[4]) + ", " + str(info_rgb.K[2]) + ", " + str(info_rgb.K[5]) + "]\n"
	fs.write(info_str)
	fs.close()
	fs = open("thermal_intrinsic_file.txt", "w")
	info_str = "thermal_intrinsic: [" + str(info_thermal.K[0]) + ", " + str(info_thermal.K[4]) + ", " + str(info_thermal.K[2]) + ", " + str(info_thermal.K[5]) + "]\n"
	fs.write(info_str)
	info_str = "thermal_distortion: [" + str(info_thermal.D[0]) + ", " + str(info_thermal.D[1]) + ", " + str(info_thermal.D[2]) + ", " + str(info_thermal.D[3]) + ", " + str(info_thermal.D[4]) + "]\n"
	fs.write(info_str)
	fs.close()
	cnt += 1
	lock = False
	print "Images saved"
	return EmptyResponse()

rgb_image_sub     = message_filters.Subscriber("camera/color/image_raw", Image)
rgb_info_sub      = message_filters.Subscriber("camera/color/camera_info", CameraInfo)
depth_image_sub   = message_filters.Subscriber("camera/aligned_depth_to_color/image_raw", Image)
thermal_image_sub = message_filters.Subscriber("/flir_boson/image_raw", Image)
thermal_info_sub  = message_filters.Subscriber("/flir_boson/camera_info", CameraInfo)

rospy.init_node("save_images_server")
ts = message_filters.ApproximateTimeSynchronizer([rgb_image_sub, rgb_info_sub, depth_image_sub, thermal_image_sub, thermal_info_sub], 10, 0.2)
ts.registerCallback(callback)

s = rospy.Service("save_image", Empty, save_imgs_cb)

while not rospy.is_shutdown():
	rospy.spin()
