#!/usr/bin/env python

import numpy as np
import mxnet as mx
import gluoncv as gcv
from mxnet import gluon
# from gluoncv.data.transforms.presets import ssd, rcnn
# from gluoncv.model_zoo import get_model
# import gluoncv.data.transforms.image as timage
# import gluoncv.data.transforms.bbox as tbbox
import cv2
# import sys
# sys.path.append(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..')))
import time
# from gluoncv.utils.metrics.voc_detection import VOC07MApMetric
# from gluoncv.utils.bbox import bbox_iou 
# from gluoncv.data.transforms.presets.ssd import SSDDefaultValTransform

# ROS related
import rospy
# from Kinect import Kinect
from std_msgs.msg import Int32MultiArray # String
# from sensor_msgs.msg import Image
# from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge
import rospkg

class Detector(object):
	def __init__(self, param, model='ssd300', ctx='gpu', filter_threshold=0.5, nms_thresh=0.5):
		self.filter_threshold = filter_threshold
        
		# ROS related
		# rospy.init_node('obj_detection', anonymous=True)
		# path = rospack.get_path("ssggcnn_ur5_grasping")
		# param_path = path + "/params/" + param
		# bounding_boxes_pub = rospy.Publisher('bb_points_array', Int32MultiArray, queue_size=10)
		# rospy.Subscriber("/camera/color/image_raw", Image, cam.set_image)
		# rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback, queue_size=10)
        
		# Transform from ROS image to OpenCV image type
		self.bridge = CvBridge()

		# Choose the default processing unit
		if ctx == 'cpu':
		    self.ctx = [mx.cpu()]
		elif ctx == 'gpu':
		    self.ctx = [mx.gpu(0)]
		else:
		    raise ValueError('Invalid context.')
        
        # self.width, self.height = dataset_commons.get_model_prop(model)
        # self.model_name = model

		# Load GluonCV parameters
		# self.model_names = rospy.get_param("/model_names")
		# print('model_names: ', self.model_names)

  #       net = get_model(model_name, pretrained=False, ctx=self.ctx)
  #       # net.set_nms(nms_thresh=0.5, nms_topk=2)
  #       net.hybridize(static_alloc=True, static_shape=True)
  #       net.initialize(force_reinit=True, ctx=self.ctx)
  #       print(self.classes)
  #       net.reset_class(classes=self.classes)
  #       net.load_parameters(param_path, ctx=self.ctx)
		
  #       self.net = net

  #       self.classes = rospy.get_param("/classes")
		# print('classes: ', self.classes)

    # def filter_predictions(self, bounding_boxes, scores, class_IDs):
    #     threshold = self.threshold
    #     idx = scores.squeeze().asnumpy() > threshold
    #     fscores = scores.squeeze().asnumpy()[idx]
    #     fids = class_IDs.squeeze().asnumpy()[idx]
    #     fbboxes = bounding_boxes.squeeze().asnumpy()[idx]
    #     return fbboxes, fscores, fids

    # def image_callback(self, color_msg):
    #     color_img = self.bridge.imgmsg_to_cv2(color_msg)
    #     height_res, width_res, _ = color_img.shape
    #     color_img = color_img[0 : self.crop_size, 
    #                 (width_res - self.crop_size)//2 : (width_res - self.crop_size)//2 + self.crop_size]
    #     self.color_img = color_img

    # def network_inference(self):
    # 	color_img = self.color_img

    # 	image_tensor, image = gcv.data.transforms.presets.ssd.load_test(color_img, self.width)
    #     labels, scores, bboxes = self.net(image_tensor.as_in_context(self.ctx))
        
    #     self.labels = labels
    #     self.scores = scores
    #     self.bboxes = bboxes

   #  def detect_main(self, image, plot=False):
   #  	color_img = self.color_img
   #  	while not rospy.is_shutdown():
   #  		with TimeIt('ssd_process_time'):
			# 	[caixas,timag]= det.detect(im)
			# size = len(caixas)
			# if size != 0:
			# 	i = 0
			# 	points_to_send_list = []
			# 	# print("size: ", size)
			# 	while i < size:
			# 		caixas[i].class_name = ""
						
			# 		point1= Point()
			# 		point2= Point()
			# 		point1.x=int(caixas[i].x1)
			# 		point1.y=int(caixas[i].y1)
			# 		point2.x=int(caixas[i].x2)
			# 		point2.y=int(caixas[i].y2)

			# 		# Verify if the bouding box is too big for the image and ignore
			# 		if abs(point2.x - point1.x) < 200:
			# 			points_to_send_list.append(point1.x)
			# 			points_to_send_list.append(point1.y)
			# 			points_to_send_list.append(point2.x)
			# 			points_to_send_list.append(point2.y)
						
			# 			img=caixas.draw(im)
			# 			cv2.circle(img,(int(caixas[i].x1),int(caixas[i].y1)), 2, (0,0,255), -1)
			# 			cv2.circle(img,(int(caixas[i].x2),int(caixas[i].y2)), 2, (0,0,255), -1)
											
			# 			ssd_img_pub.publish(CvBridge().cv2_to_imgmsg(img, 'bgr8'))
			# 	i+=1
			# 	points_to_send.data = points_to_send_list # assign the array with the value you want to send
			# 	print(points_to_send.data)
			# 	arraypub.publish(points_to_send)
			# 	points_to_send.data = []
			# else:
			# 	print("No obj found")
			
			# rate.sleep()
        

def main():
	# TODO: You just need to pass the param name inside the log folder (checkpoints folder configured in config.json)
	params = 'ssd_512_resnet50_v1_voc_best_epoch_0017_map_0.9534.params'

	obj_detect = Detector(params, 
						  model='ssd300_vgg16_voc', 
						  ctx='gpu', 
						  filter_threshold=0.5, 
						  nms_thresh=0.5)

    # obj_detect.network_inference()

if __name__ == "__main__":
	main()