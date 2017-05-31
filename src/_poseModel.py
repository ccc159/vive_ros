import rospy
import numpy
from math import sqrt
from tf.transformations import inverse_matrix, euler_from_matrix,\
	translation_from_matrix, quaternion_from_matrix, quaternion_matrix,\
	translation_matrix
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion




class PoseModel:
	
	def __init__(self, matrix=None):
		self._matrix = matrix
		self.id = None
		self.timestamp = None
	#eof
	
	
	
	def getMatrix(self):
		return self._matrix
	#eof
	
	def setMatrix(self, matrix):
		self._matrix = matrix
		return self
	#eof
	
	
	
	def getPose(self):
		return matrixToPose(self._matrix)
	#eof
	
	def setPose(self, obj):
		self._matrix = poseToMatrix(obj)
		return self
	#eof
	
	
	
	def getPoseStamped(self, header=None):
		if header is None:
			header = Header(0, rospy.rostime.get_rostime(), "world")
		
		return PoseStamped(header, self.getPose())
	#eof
	
	def setPoseStamped(self, obj):
		self.setPose(obj.pose)
		self.timestamp = getStampedTime(obj)
		
		return self
	#eof
	
	
	
	def getInverseMatrix(self):
		return inverse_matrix(self._matrix)
	#eof
	
	def setInverseMatrix(self, matrix):
		self._matrix = inverse_matrix(matrix)
		return self
	#eof
	
	
	
	def getMarker(self):
		m = Marker()
		m.pose = self.pose
		return m
	#eof
	
	def setMarker(self, obj):
		self.setPose(obj.pose)
		return self
	#eof
	
	
	
	def transformByMatrix(self, matrix):
		return self.setMatrix(self.getTransformedMatrix(matrix))
	#eof
	
	
	
	def getTransformedMatrix(self, matrix):
		return numpy.dot(self.getMatrix(), matrix)
	#eof
	
	
	
	def getRotationAroundZ(self):
		return euler_from_matrix(self._matrix, 'szyx')[0]
	#eof
	
	
	
	def isNone(self):
		return self._matrix is None
	#eof
	
	
	
	def getPositionDistance(self):
		t = translation_from_matrix(self._matrix)
		return sqrt(pow(t[0],2) + pow(t[1],2) + pow(t[2],2))
	#eof
	
	
	
	def getUnitTranslationVector(self):
		d = self.getPositionDistance()
		
		if(d == 0):
			return (0,0,0)
		else:
			t = translation_from_matrix(self._matrix)
			return (t[0]/d, t[1]/d, t[2]/d)
	#eof
	
	
	
	def getTranslation(self):
		return translation_from_matrix(self._matrix)
	#eof
	
	
	
	def setTranslation(self, t):
		t = translation_matrix(t)
		self.setTranslationMatrix(t)
	#eof
	
	
	
	def getTranslationMatrix(self):
		return translation_matrix(self.getTranslation())
	#eof
	
	
	
	def setTranslationMatrix(self, t):
		r = self.getRotationMatrix()
		self._matrix = numpy.dot(t,r)
	#eof
	
	
	
	def getQuaternion(self):
		return quaternion_from_matrix(self._matrix)
	#eof
	
	
	
	def setQuaternion(self, q):
		q = quaternion_matrix(q)
		self.setRotationMatrix(q)
	#eof
	
	
	
	def getRotationMatrix(self):
		return quaternion_matrix(self.getQuaternion())
	#eof
	
	
	
	def setRotationMatrix(self, r):
		r = quaternion_matrix(quaternion_from_matrix(r))
		t = self.getTranslationMatrix()
		self._matrix = numpy.dot(t,r)
	#eof
	
	
	
	def setTranslationQuaternion(self, t, q):
		q = quaternion_matrix(q)
		t = translation_matrix(t)
		
		self._matrix = numpy.dot(t,q)
	#eof
	
	
	def getPoint(self):
		t = self.getTranslation()
		p = Point()
		p.x = t[0]
		p.y = t[1]
		p.z = t[2]
		
		return p
	#eof
#eoc



#utils.........

def transformationMatrix(translation, orientation):
	translation = translation_matrix(translation)
	orientation = quaternion_matrix(orientation)

	return numpy.dot(translation, orientation)


# eof


def poseToMatrix(pose):
	q = pose.orientation
	t = pose.position

	return transformationMatrix([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])


# eof


def markerTransformationMatrix(marker):
	q = marker.pose.orientation
	t = marker.pose.position

	return transformationMatrix([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])


# eof



def transformPoseByMatrix(pose, matrix):
	return numpy.dot(matrix, pose)


# eof



def matrixToPose(matrix):
	t = tuple(translation_from_matrix(matrix))
	q = tuple(quaternion_from_matrix(matrix))

	return Pose(Point(t[0], t[1], t[2]), Quaternion(q[0], q[1], q[2], q[3]))


# eof



def getStampedTime(obj):
	return obj.header.stamp.secs + (obj.header.stamp.nsecs / 1000000000.0)