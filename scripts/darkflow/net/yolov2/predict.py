import numpy as np
import math
import cv2
import os
import json
#from scipy.special import expit
#from utils.box import BoundBox, box_iou, prob_compare
#from utils.box import prob_compare2, box_intersection
from ...utils.box import BoundBox
from ...cython_utils.cy_yolo2_findboxes import box_constructor
path = os.getcwd()
import math
# file = open(path + '/' + out.txt,'a')
#import rospy
#from std_msgs.msg import String

def expit(x):
	return 1. / (1. + np.exp(-x))

def _softmax(x):
    e_x = np.exp(x - np.max(x))
    out = e_x / e_x.sum()
    return out

def findboxes(self, net_out):
	# meta
	meta = self.meta
	boxes = list()
	boxes=box_constructor(meta,net_out)
	return boxes


def get_calibration_values(path):
    # will be updated later to fit more calibration values
    f = open(path + os.sep + "calibration.txt", "r")
    return float(f.read())

def distance_to_camera(knownWidth, focalLength, perWidth):
    # compute the distance from the marker to the camera
	return (knownWidth * focalLength) / perWidth

def calibrate(pixelWidth):
	print('vvvvvvvvvvvvvvv')
	# info about buoy
	# known_height_mm = 189.44
	# known_width_mm = 279.89
	known_height_m = 0.18944
	known_width_m = 0.27989
	# known_height_in = 7.45826772
	# known_width_in = 11.0192913

	# for now, we're using meters and buoys
	KNOWN_DISTANCE = 0.5
	KNOWN_HEIGHT = known_height_m
	KNOWN_WIDTH = known_width_m

	focalLength = (pixelWidth * KNOWN_DISTANCE) / KNOWN_WIDTH
	f = open(path + os.sep + "calibration.txt", "w+")
	# f.write("buoy ")
	f.write(str(focalLength))

def length(v):
	return math.sqrt(v[0]**2+v[1]**2)

def dot_product(v,w):
	return v[0]*w[0]+v[1]*w[1]
def determinant(v,w):
	return v[0]*w[1]-v[1]*w[0]
def inner_angle(v,w):
	cosx=dot_product(v,w)/(length(v)*length(w))
	rad=math.acos(cosx) # in radians
	return rad*180/math.pi # returns degrees
def angle_clockwise(A, B):
	inner=inner_angle(A,B)
	det = determinant(A,B)
	if det<0: #this is a property of the det. If the det < 0 then B is clockwise of A
		return inner
	elif det>0: # if the det > 0 then A is immediately clockwise of B
		return 360-inner
	else: return 0	

def postprocess(self, net_out, im, save = True):
	#-----------------Publish setup--------------
	#refresh_rate = 50
	#pub = rospy.Publisher('POWER LEVEL OVER 9000 CV DETECTION', String, queue_size=10)
	#rospy.init_node('talker', anonymous = True)
	#rate = rospy.Rate(refresh_rate)
	#-------------DONE--------------------     
	
	"""
	Takes net output, draw net_out, save to disk
	"""
	global path
	boxes = self.findboxes(net_out)

	# meta
	meta = self.meta
	threshold = meta['thresh']
	colors = meta['colors']
	labels = meta['labels']
	if type(im) is not np.ndarray:
		imgcv = cv2.imread(im)
	else: imgcv = im
	h, w, _ = imgcv.shape
	mid_x,mid_y = w//2,h//2
	resultsForJSON = []
	for b in boxes:
		boxResults = self.process_box(b, h, w, threshold)
		if boxResults is None:
			continue
		left, right, top, bot, mess, max_indx, confidence = boxResults
		thick = int((h + w) // 300)
		if self.FLAGS.json:
			resultsForJSON.append({"label": mess, "confidence": float('%.2f' % confidence), "topleft": {"x": left, "y": top}, "bottomright": {"x": right, "y": bot}})
			continue
		# file.write('{}\t{}\t{}\t{}\t{}\n'.format(str(left),str(top),str(right),str(bot),mess))
		cv2.rectangle(imgcv,
			(left, top), (right, bot),
			colors[max_indx], thick)
		bx,by = left-mid_x,top-mid_y
		rx,ry = 0,abs(by)
		v_vec = [bx,by]
		w_vec = [rx,ry]
		angle = angle_clockwise(w_vec,v_vec)
		print(bx,by,rx,ry,angle,mid_x,mid_y)
		perWidth = right-left
		known_height_m = 0.18944
		known_width_m = 0.27989
		if not os.path.exists(path + os.sep + 'calibration.txt'):
			calibrate(perWidth)

		focal_length = get_calibration_values(path)
		dist = distance_to_camera(known_width_m,focal_length,perWidth)
		mess1 = mess + ' Dist: {} Angle: {}'.format(dist,angle)
		print('\n',mess)
		object_data = {(left,top,right,bot):(mess,dist,angle)}
		#pub.publish(str(object_data))
		cv2.putText(imgcv, mess1, (left, top - 12),
			0, 1e-3 * h, colors[max_indx],thick//3)

	if not save: return imgcv

	outfolder = os.path.join(self.FLAGS.imgdir, 'out')
	img_name = os.path.join(outfolder, os.path.basename(im))
	if self.FLAGS.json:
		textJSON = json.dumps(resultsForJSON)
		textFile = os.path.splitext(img_name)[0] + ".json"
		with open(textFile, 'w') as f:
			f.write(textJSON)
		return

	cv2.imwrite(img_name, imgcv)
