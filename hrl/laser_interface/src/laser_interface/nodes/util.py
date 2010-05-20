from opencv import cv
from opencv import highgui
import numpy as numpy
np = numpy
import numpy.linalg as la
import numpy as np
import logging as lg
import sys
import math
import time

class log(object):
	CRITICAL = 50
	ERROR	= 40
	WARNING  = 30
	INFO	 = 20
	DEBUG	= 10
	NOTSET   = 0
	level_strings = {50: 'CRITICAL', 40: 'ERROR', 30: 'WARNING', 20: 'INFO', 10: 'DEBUG', 0: 'NOTSET'}

	#Users can set a global level so that all logs created 
	#can have the same default level
	global_lv = lg.WARNING

	#List of all logs instantiated so far
	logs		   = dict()

	def __init__(self, name, lv=None, file=sys.stdout):
		""" Create a log with a specified level and/or output file """
		#So that we can have multiple logs of the same name
		if self.logs.has_key(name):
			self.logs[name].append(self)
		else:
			self.logs[name] = [self]

		self.name = name
		if lv != None:
			self.lv = lv
		else:
			self.lv = self.global_lv
		self.file = file

	def log(self, level, *msgs):
		"""
			Print out a message in default file if level given is greater
			than this logger's set level
		"""
		if self.lv <= level:
			try:
				level_name = self.level_strings[level]
			except exceptions.KeyError:
				level_name = str(level)

			print >>self.file, "%s (%s):" % (self.name, level_name),
			for m in msgs:
				print >>self.file, m,
			print >>self.file

	@classmethod
	def set_logs_lv(log_class, level):
		""" 
			Set the log level for all log objects created so far as 
		"""
		for l_name in log_class.logs:
			for log in log_class.logs[l_name]:
				log.lv = level


##################################################################
#
# IMAGE CONVERSION FUNCTIONS
#
##################################################################
#
# the key is an opencv type returned by cv.cvGetElemType(im)
# the value is (numpy type, number of channels)
#
#Type of the matrix elements.
#Usually it is specified in form CV_<bit_depth>(S|U|F)C<number_of_channels>,
#for example:
#	CV_8UC1 means an 8-bit unsigned single-channel matrix
#	CV_32SC2 means a 32-bit signed matrix with two channels 
#


cv2np_type_dict = {cv.CV_16S	  : (np.int16, 1),	
				   cv.CV_16SC	 : (np.int16, 1),   
				   cv.CV_16SC1	: (np.int16, 1),   
				   cv.CV_16SC2	: (np.int16, 2),   
				   cv.CV_16SC3	: (np.int16, 3),   
				   cv.CV_16SC4	: (np.int16, 4),   
				   cv.CV_16U	  : (np.uint16, 1),   
				   cv.CV_16UC	 : (np.uint16, 1),   
				   cv.CV_16UC1	: (np.uint16, 1),   
				   cv.CV_16UC2	: (np.uint16, 2),   
				   cv.CV_16UC3	: (np.uint16, 3),   
				   cv.CV_16UC4	: (np.uint16, 4),   
				   cv.CV_32F	  : (np.float32, 1),   
				   cv.CV_32FC	 : (np.float32, 1),   
				   cv.CV_32FC1	: (np.float32, 1),   
				   cv.CV_32FC2	: (np.float32, 2),   
				   cv.CV_32FC3	: (np.float32, 3),   
				   cv.CV_32FC4	: (np.float32, 4),   
				   cv.CV_32S	  : (np.int32, 1),   
				   cv.CV_32SC	 : (np.int32, 1),   
				   cv.CV_32SC1	: (np.int32, 1),   
				   cv.CV_32SC2	: (np.int32, 2),   
				   cv.CV_32SC3	: (np.int32, 3),   
				   cv.CV_32SC4	: (np.int32, 4),   
				   cv.CV_64F	  : (np.float64, 1),   
				   cv.CV_64FC	 : (np.float64, 1),   
				   cv.CV_64FC1	: (np.float64, 1),   
				   cv.CV_64FC2	: (np.float64, 2),   
				   cv.CV_64FC3	: (np.float64, 3),   
				   cv.CV_64FC4	: (np.float64, 4),   
				   cv.CV_8S	   : (np.int8, 1),   
				   cv.CV_8SC	  : (np.int8, 1),   
				   cv.CV_8SC1	 : (np.int8, 1),   
				   cv.CV_8SC2	 : (np.int8, 2),   
				   cv.CV_8SC3	 : (np.int8, 3),   
				   cv.CV_8SC4	 : (np.int8, 4),   
				   cv.CV_8U	   : (np.uint8, 1),   
				   cv.CV_8UC	  : (np.uint8, 1),   
				   cv.CV_8UC1	 : (np.uint8, 1),   
				   cv.CV_8UC2	 : (np.uint8, 2),   
				   cv.CV_8UC3	 : (np.uint8, 3),   
				   cv.CV_8UC4	 : (np.uint8, 4)}


cv2np_type_dict_invertible = {cv.CV_16SC1	: (np.int16, 1),   
							  cv.CV_16SC2	: (np.int16, 2),   
							  cv.CV_16SC3	: (np.int16, 3),   
							  cv.CV_16SC4	: (np.int16, 4),   
							  cv.CV_16UC1	: (np.uint16, 1),   
							  cv.CV_16UC2	: (np.uint16, 2),   
							  cv.CV_16UC3	: (np.uint16, 3),   
							  cv.CV_16UC4	: (np.uint16, 4),   
							  cv.CV_32FC1	: (np.float32, 1),   
							  cv.CV_32FC2	: (np.float32, 2),   
							  cv.CV_32FC3	: (np.float32, 3),   
							  cv.CV_32FC4	: (np.float32, 4),   
							  cv.CV_32SC1	: (np.int32, 1),   
							  cv.CV_32SC2	: (np.int32, 2),   
							  cv.CV_32SC3	: (np.int32, 3),   
							  cv.CV_32SC4	: (np.int32, 4),   
							  cv.CV_64FC1	: (np.float64, 1),   
							  cv.CV_64FC2	: (np.float64, 2),   
							  cv.CV_64FC3	: (np.float64, 3),   
							  cv.CV_64FC4	: (np.float64, 4),   
							  cv.CV_8SC1	 : (np.int8, 1),   
							  cv.CV_8SC2	 : (np.int8, 2),   
							  cv.CV_8SC3	 : (np.int8, 3),   
							  cv.CV_8SC4	 : (np.int8, 4),   
							  cv.CV_8UC1	 : (np.uint8, 1),   
							  cv.CV_8UC2	 : (np.uint8, 2),   
							  cv.CV_8UC3	 : (np.uint8, 3),   
							  cv.CV_8UC4	 : (np.uint8, 4)}


#def cv2np(im):
#	numpy_type, nchannels = cv2np_type_dict[cv.cvGetElemType(im)]
#	array_size = [im.height, im.width, nchannels]
#	np_im = np.frombuffer(im.imageData, dtype=numpy_type)
#	return np.reshape(np_im, array_size)

def cv2np(im, format='RGB'):
	if format == 'BGR':
		cv.cvCvtColor( im, im, cv.CV_BGR2RGB )
	numpy_type, nchannels = cv2np_type_dict[cv.cvGetElemType(im)]
	array_size = [im.height, im.width, nchannels]
	np_im = np.frombuffer(im.imageData, dtype=numpy_type, 
            count=im.height*im.width*nchannels*(im.depth/8))
	return np.reshape(np_im, array_size)

def cv2pil( im, format='RGB' ):
	return np2pil( cv2np( im, format))



if True:
	np2cv_type_dict = dict([(str(np.dtype(v[0])), v[1]), k] for
						   k,v in cv2np_type_dict_invertible.items())


	def np2cv(im):
		image = np2pil( im )
		image.save('test.bmp', 'BMP')
		cvim = highgui.cvLoadImage('test.bmp')
		return cvim

	def np1d2cvImage(nparr, height, width):
		""" takes a 1-d numpy array and converts to an opencv image
				of the appropriate height and width
		"""
		np2d = np.reshape(nparr, [height, width])
		cvim = np2cv(np2d)
		return cvim


#	def np2cv_old(im):
#		print 'WARNING: np2cv is not reliable or well tested (it is a bit flakey...)'
#		#raise AssertionError('np2cv does not work :-(')
#		if len(im.shape) == 3:
#			shp = im.shape
#			channels = shp[2]
#			height = shp[0]
#			width = shp[1]
#			#height, width, channels = im.shape
#		elif len(im.shape) == 2:
#			height, width = im.shape
#			channels = 1
#		else:
#			raise AssertionError("unrecognized shape for the input image. should be 3 or 2, but was %d." % len(im.shape))
#		key = (str(im.dtype), channels)
#		cv_type = np2cv_type_dict[key]
#		print 'attempt to create opencv image with (key, width, height, channels) =', (key, width, height, channels)
#		print 'making a cvMat with type =', cv_type
#		cvmat = cv.cvCreateMat(height, width, cv_type)
#		if len(im.shape) == 3:
#			for y in xrange(height):
#				for x in xrange(width):
#					pix = [float(v) for v in im[y,x]]
#					scalar = cv.cvScalar(*pix)
#					cvmat[y,x] = scalar
#		else:
#			for y in xrange(height):
#				for x in xrange(width):
#					pix = float(im[y,x])
#					cvmat[y,x] = cv.cvScalar(pix, pix, pix)
#		print 'resulted in an image openCV image with the following properties:'
#		numpy_type, nchannels = cv2np_type_dict[cv.cvGetElemType(cvmat)]
#		print '(numpy_type, nchannels, cvmat.width, cvmat.height) =', (numpy_type, nchannels, cvmat.width, cvmat.height)
#		return cvmat


if False:
	#
	#
	# IPL_DEPTH_8U - unsigned 8-bit integers
	# IPL_DEPTH_8S - signed 8-bit integers
	# IPL_DEPTH_16S - signed 16-bit integers
	# IPL_DEPTH_32S - signed 32-bit integers
	# IPL_DEPTH_32F - single precision floating-point numbers
	# IPL_DEPTH_64F - double precision floating-point numbers
	#
	#

	np2cv_type_dict = {None						:  cv.IPL_DEPTH_1U,
					   str(np.dtype(np.bool))	  :  cv.IPL_DEPTH_8U,
					   str(np.dtype(np.bool8))	 :  cv.IPL_DEPTH_8U,
					   str(np.dtype(np.uint8))	 :  cv.IPL_DEPTH_8U, 
					   str(np.dtype(np.int8))	  :  cv.IPL_DEPTH_8S,
					   str(np.dtype(np.uint16))	:  cv.IPL_DEPTH_16U,
					   str(np.dtype(np.int16))	 :  cv.IPL_DEPTH_16S,
					   str(np.dtype(np.int32))	 :  cv.IPL_DEPTH_32S,
					   str(np.dtype(np.float32))   :  cv.IPL_DEPTH_32F,
					   str(np.dtype(np.float64))   :  cv.IPL_DEPTH_64F}



	def np2cv(im):
		print 'WARNING: np2cv is not reliable or well tested (it is a bit flakey...)'
		#raise AssertionError('np2cv does not work :-(')
		if len(im.shape) == 3:
			shp = im.shape
			channels = shp[2]
			height = shp[0]
			width = shp[1]
			#height, width, channels = im.shape
		elif len(im.shape) == 2:
			height, width = im.shape
			channels = 1
		else:
			raise AssertionError("unrecognized shape for the input image. should be 3 or 2, but was %d." % len(im.shape))
		key = str(im.dtype)
		cv_type = np2cv_type_dict[key]
		print 'attempt to create opencv image with (key, width, height, channels) =', (key, width, height, channels)
		cv_im = cv.cvCreateImage(cv.cvSize(width, height), cv_type, channels)
		#cv_im.imageData = im.tostring()
		if True:
			if len(im.shape) == 3:
				for y in xrange(height):
					for x in xrange(width):
						pix = [float(v) for v in im[y,x]]
						scalar = cv.cvScalar(*pix)
						#print scalar
						cv_im[y,x] = scalar
			else:
				for y in xrange(height):
					for x in xrange(width):
						pix = float(im[y,x])
						cv_im[y,x] = cv.cvScalar(pix, pix, pix)
						#print 'im[y,x], cv_im[y,x] =', im[y,x], cv_im[y,x]
		print 'resulted in an image openCV image with the following properties:'
		numpy_type, nchannels = cv2np_type_dict[cv.cvGetElemType(cv_im)]
		print '(numpy_type, nchannels, cvmat.width, cvmat.height) =', (numpy_type, nchannels, cv_im.width, cv_im.height)
		return cv_im



def get_sub_image( image, coord, sizex=10, sizey=10, save=True ):

        high_x = coord.x + int(round(sizex/2.0))
        low_x  = coord.x - int(sizex/2.0)
        high_y = coord.y + int(round(sizey/2.0))
        low_y  = coord.y - int(sizey/2.0)

        if high_x > image.width - 1 :
            high_x = image.width - 1
            low_x  = image.width - 1 - sizex
        elif low_x < 0 :
            low_x  = 0
            high_x = sizex - 1
        
        if high_y > image.height - 1 :
            high_y = image.height - 1
            low_y  = image.height - 1 - sizey
        elif low_y < 0 :
            low_y  = 0
            high_y = sizey - 1

        b = image[low_y:high_y,low_x:high_x]

        if save:
            curtime = time.localtime()
	    curtime_raw = time.time()
	    i = float( 100*(curtime_raw - int(curtime_raw)))
            date_name = time.strftime('%Y%m%d%I%M%S_' + str(i), curtime)
            highgui.cvSaveImage( date_name+'dot.png' , b )

        return {'sub_img':b,'sub_img_top_left': cv.cvPoint(low_x,low_y) }


def matchTemplate(self, template, image):
        '''	matchTemplate(self, template, image):		\
                returns - correlation value of best match (b/w 0 & 1)	\
                top-left coord of template for the best match (cvPoint) \
        '''

        matchResultHeight = image.height-template.height+1
        matchResultWidth = image.width-template.width+1

        #print 'matchResultHeight: %d matchResultWidth %d'%(matchResultHeight, matchResultWidth)
        matchResult = cv.cvCreateMat(matchResultHeight, matchResultWidth, cv.CV_32FC1)
        cv.cvMatchTemplate(image, template, matchResult, cv.CV_TM_CCORR_NORMED)

        min_loc = cv.cvPoint(0,0)
        max_loc = cv.cvPoint(0,0)

        min_val, max_val = cv.cvMinMaxLoc(matchResult, min_loc, max_loc)

        return {'image': matchResult , 'max_val':max_val, 'max_loc':max_loc}


##################################################################
#
# DISPLAY HELPER FUNCTIONS
#
##################################################################


def display_images(image_list, max_x = 1200, max_y = 1000, save_images=False):
	"""
	Display a list of OpenCV images tiled across the screen
	with maximum width of max_x and maximum height of max_y

	save_images - will save the images(with timestamp)
	"""

	curtime=time.localtime()
	date_name = time.strftime('%Y_%m_%d_%I%M%S', curtime)

	loc_x, loc_y = 0, 0
	wins = []
	for i, im in enumerate(image_list):
		if save_images:
			if im.nChannels == 1 and im.depth == cv.IPL_DEPTH_32F:
				clr = cv.cvCreateImage(cv.cvSize(im.width, im.height), cv.IPL_DEPTH_8U, 1)
				cv.cvConvertScale(im, clr, 255.0)
				im = clr
			highgui.cvSaveImage('image%d_'%i+date_name+'.png', im)

		window_name = 'image %d' % i
		wins.append((window_name, im)) 
		highgui.cvNamedWindow(window_name, highgui.CV_WINDOW_AUTOSIZE)
		highgui.cvMoveWindow(window_name, loc_x, loc_y)
		loc_x = loc_x + im.width
		if loc_x > max_x:
			loc_x = 0
			loc_y = loc_y + im.height
			if loc_y > max_y:
				loc_y = 0
	while True:
		for name, im in wins:
			highgui.cvShowImage(name, im)
		keypress = highgui.cvWaitKey(10)
		if keypress == '\x1b':
			break


def draw_ellipse(image, center, axes, angle,
				 start_angle=0.0, end_angle=360.0,
				 color=(255,0,0), thickness=1):
	center = cv.cvPoint(rnd(center[0]), rnd(center[1]))
	axes = cv.cvSize(rnd(axes[0]), rnd(axes[1]))
	color = cv.CV_RGB(color[0], color[1], color[2])
	cv.cvEllipse(image, center, axes, angle, start_angle, end_angle, color, thickness)  


def draw_line(image, pnt1, pnt2, color=(255,0,0)):
	cv.cvLine(image,
			  cv.cvPoint(rnd(pnt1[0]), rnd(pnt1[1])),
			  cv.cvPoint(rnd(pnt2[0]), rnd(pnt2[1])),
			  cv.CV_RGB(*color))


class Ellipse(object):
	def __init__(self):
		self.ready = False

	def __str__(self):
		return 'center, axis_lengths, angle = ' +  str((self.center, self.axis_lengths, self.angle))

	def set_from_gaussian(self, gaussian, weight):
		"""Assumes that the first two dimensions of the gaussians are the spatial dimensions to be used."""
		g = gaussian
		w = weight
		cov = g.cov[:2,:2]
		mean = g.mean[:2].T
		mean = np.array([mean[0,0], mean[0,1]])
		evec, eva, evecT = la.svd(cov)
		e1, e2 = np.sqrt(eva)
		v1 = np.array(evecT[0]).flatten()
		v2 = np.array(evecT[1]).flatten()
		if e1 > e2:
			pe = e1
			pv = v1
			se = e2
			sv = v2
		else:
			pe = e2
			pv = v2
			se = e1
			sv = v1
		self.center = mean
		## self.axes removed due to ambiguous name
		## replaced with self.axis_lengths
		#self.axes = (pe, se)
		self.axis_lengths = (pe, se)
		self.angle = 360.0*(np.arctan2(-pv[1],pv[0])/(2.0*np.pi))
		self.principal_axis = pv
		self.ready = True
		print self

	def set_from_cvbox2d(self, cvbox2d):
		box = cvbox2d
		self.center = (box.center.x, box.center.y)
		## self.axes removed due to ambiguous name
		## replaced with self.axis_lengths
		# self.axes = (box.size.width, box.size.height)
		self.axis_lengths = (box.size.width, box.size.height)
		self.angle = box.angle
		self.principal_axis = None
		self.ready = True
		print self

	def draw_on_image(self, image,
					  start_angle=0.0, end_angle=360.0,
					  color=(0,0,255), thickness=1,
					  principal_axis_color=(255,255,255)):
		if self.ready:
			draw_ellipse(image, self.center, self.axis_lengths, self.angle,
						 start_angle, end_angle,
						 color, thickness)
			if self.principal_axis is not None:
				draw_line(image,
						  self.center,
						  self.center + (self.principal_axis * self.axis_lengths[0]),
						  principal_axis_color)

def mask_image( im, mask ):
	if mask.depth == 8:
		bim = cv.cvCreateImage( cv.cvSize(mask.width, mask.height), cv.IPL_DEPTH_32F, mask.nChannels )
		cv.cvConvertScale( mask, bim, 1.0/255.0)

	if im.depth == 8:
		newim = cv.cvCreateImage( cv.cvSize(im.width, im.height), cv.IPL_DEPTH_32F, im.nChannels )
		cv.cvConvertScale( im, newim, 1.0/255.0)

		
	print 'newim.depth = ',newim.depth
	print 'newim.nChannels = ',newim.nChannels
	print 'bim.depth = ',bim.depth
	print 'bim.nChannels = ',bim.nChannels
	if newim.nChannels == 3 and newim.depth == 32 and bim.nChannels == 3 and bim.depth == 32:
		outputIm = cv.cvCloneImage( bim )
		cv.cvMul( bim, newim, outputIm, 1 )
		return outputIm
	else:
		print 'oops problem with formats'
		return mask
		


##################################################################
#					Angles related functions 
##################################################################
def vec_of_ang(a):
	return numpy.matrix([numpy.cos(a), numpy.sin(a)]).T

def ang_of_vec(a):
	a = a / numpy.linalg.norm(a) 
	return math.atan2(a[1,0], a[0,0])

def avg_ang(wa, a, wb, b):
	""" 
		 Calculates the average between two angles 
		 wa  weight of first angle
		 a   first angle
		 wb  weight of second angle
		 b   second angle
	"""
	return ang_of_vec(wa * vec_of_ang(a) + wb * vec_of_ang(b))

def blend_ang(alpha, a, b):
	return avg_ang(alpha, a, 1-alpha,b)

def standard_rad(t):
	if t > 0:
		return ((t + numpy.pi) % (numpy.pi * 2))  - numpy.pi
	else:
		return ((t - numpy.pi) % (numpy.pi * -2)) + numpy.pi

def best_turn_dir(reference, new_angle):
	""" positive is left, negative is right! """
	return standard_rad(reference - new_angle)

def cart_of_pol(p):
   """ Finds cartesian coordinates of polar points [r, t]' """
   r = p[0,:]
   t = p[1,:]
   x = numpy.multiply(numpy.cos(t), r)
   y = numpy.multiply(numpy.sin(t), r)
   return numpy.vstack((x,y))

def pol_of_cart(p):
	""" Find polar coordinates of cartesian points [x, y]' """
	norm = numpy.linalg.norm(p)
	ang = math.atan2(p[1,0], p[0,0])
	return numpy.matrix([norm, ang]).T

##################################################################
#
# NUMPY HELPER FUNCTIONS 
#
##################################################################
def list_mat_to_mat(list_mat, axis=0):
	return np.concatenate(tuple(list_mat), axis=axis)

def list_of_mat(mat):
	for i in range(mat.shape[1]):
		yield mat[:,i]

def nearest(mat, target):
	'''
	Return a sorted list of the nearest (euclidean dist) element
	of a matrix to a target value and their indeices.
	'''
        diff_vec = mat - target
	pwr = np.ones_like(mat[0])*2
        dist = np.power(np.sum(np.power( diff_vec ,pwr),axis=1),0.5)
        indices = dist.argsort(axis=0)

	return mat[indices.A1], indices

##################################################################
#
# Lists 
#
##################################################################
def el_from_list(l, indices):
	for i in indices:
		yield l[i]

##################################################################
#
# MISCELLANEOUS MATH HELPER FUNCTIONS
#
##################################################################
def blend(alpha, a, b):
	return (alpha * a) + ((1-alpha) * b)

def approx_equal(a, b, epsilon=.001):
	return (b < (a+epsilon)) and ((a-epsilon) < b)

def norm(mat):
	"""
		Calculate L2 norm for column vectors in a matrix 
	"""
	return np.power(np.sum(np.power(mat,2), axis=0), 0.5)

def rnd(v):
	return int(round(v))

def point_to_homo(p):
	""" Convert points into homogeneous coordinates """
	o = numpy.matrix(numpy.ones((1, p.shape[1])))
	return numpy.vstack((p,o))

	""" Convert points back from homogeneous coordinates """
def homo_to_point(p):
	return p[0:p.shape[0]-1,:]

def get_nearest_feature( image, this_point, n=2000 ):
	"""
	Get the n-nearest features to a specified image coordinate.
	Features are determined using cvGoodFeaturesToTrack.
	"""

	_red = cv.cvScalar (0, 0, 255, 0);
	_green = cv.cvScalar (0, 255, 0, 0);
	_blue = cv.cvScalar (255,0,0,0);
	_white = cv.cvRealScalar (255)
	_black = cv.cvRealScalar (0)

	quality = 0.01
	min_distance = 4
	N_best = n
	win_size = 11

	grey = cv.cvCreateImage (cv.cvGetSize (image), 8, 1)
	eig = cv.cvCreateImage (cv.cvGetSize (image), 32, 1)
	temp = cv.cvCreateImage (cv.cvGetSize (image), 32, 1)

	# create a grey version of the image
	cv.cvCvtColor ( image, grey, cv.CV_BGR2GRAY)

	points = cv.cvGoodFeaturesToTrack ( 
		grey, eig, temp,
		N_best,
		quality, min_distance, None, 3, 0, 0.04)

	# refine the corner locations
	better_points = cv.cvFindCornerSubPix (
		grey,
		points,
		cv.cvSize (win_size, win_size), cv.cvSize (-1, -1),
		cv.cvTermCriteria (cv.CV_TERMCRIT_ITER | cv.CV_TERMCRIT_EPS,
						   20, 0.03))

	eigs = []
	for i in range(len(points)):
		eigs.append(cv.cvGetMat(eig)[int(points[i].y)][int(points[i].x)])

	mypoints = np.matrix(np.zeros((len(points)*2),dtype=float)).reshape(len(points),2)
	dists = []
	for i,point in enumerate(points):
		mypoints[i,0]=point.x
		mypoints[i,1]=point.y
		dists.append( np.linalg.norm(mypoints[i,:]-this_point) )

	dists = np.array(dists)
	sorteddists = dists.argsort()

	cv.cvDrawCircle ( image, points[ sorteddists[0] ], 5, _green, 2, 8, 0 )

	return better_points[ sorteddists[0] ]

def fitLine_highslope(x, y):
	""" finds a,b such that x=ay+b is the best line for the current
		laser scan data. (good if expected slope in close to infinity)
		x, y -- nX1 matrices
		returns a,b,mean residual
	"""

	A = np.column_stack((y, np.ones((y.shape[0],1))))
#	print 'x'
#	print x
	x,resids,rank,s = np.linalg.linalg.lstsq(A, x)
	a = x[0,0]
	b = x[1,0]

	return a,b,resids/y.shape[0]



##################################################################
#
# DEPRECATED FUNCTIONS FOR LEGACY CODE
#
##################################################################


def cvmat2numpymat(cvmat):
	nmat = numpy.zeros((cvmat.width,cvmat.height))
	for i in range(cvmat.width):
		for j in range(cvmat.height):
			nmat[i][j] = cvmat[i][j]
	return nmat

def numpymat2cvmat(nmat):
    cvmat = cv.cvCreateMat(nmat.shape[0],nmat.shape[1],cv.CV_32FC1)
    for i in range(nmat.shape[0]):
        for j in range(nmat.shape[1]):
            #print cvmat[i][j]
            #print nmat[i,j]	  
            cvmat[i,j] = nmat[i,j]	  
    return cvmat

def find_homography(points1,points2):
	cv_homography = cv.cvCreateMat(3,3,cv.CV_32FC1)
	cv_points1 = numpymat2cvmat(points1)
	cv_points2 = numpymat2cvmat(points2)
	cv.cvFindHomography(cv_points1,cv_points2,cv_homography)
	return cvmat2numpymat(cv_homography)

