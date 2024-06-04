import numpy as np
from PyQt5.QtCore import Qt,QPoint
from PyQt5.QtGui import  QColor, QPen, QBrush, QPainter, QPolygon

class plotColorPalette():
	pen_x = QPen(QColor(255,0,0),2)
	pen_y = QPen(QColor(0,255,0),2)
	pen_z = QPen(QColor(0,0,255),2)

	RAS_TN_DB = QPen(QColor(0, 96, 186),3)
	RAS_TN_GR = QPen(QColor(44, 171, 5),3)
	RAS_TN_YE = QPen(QColor(235, 227, 0),3)
	RAS_TN_PU = QPen(QColor(206, 0, 224),3)
	RAS_TN_LB = QPen(QColor(28, 164, 255),3)
	RAS_TN_OR = QPen(QColor(255, 149, 0),3)

	default_vessel_hull = QPen(QColor(0, 0, 0),3)
	vessel_hull_disabled = QPen(QColor(20, 20, 20),3)

	thrusters = QPen(QColor(0, 0, 0),2)


class plotTree2d():
	""" Class to assist in drawing 2d objects in a tree structure. 
		The root of the tree should not have a parent.
	"""
	def __init__(self, line:np.ndarray=None,parent:'plotTree2d'=None, brush:QBrush=None, pen:QPen=None, translation:np.ndarray=np.array([0.0,0.0]), rotation:float=0.0,inheritLayout:'plotTree2d'=None,name:str=None, drawscale_:float=1.0):
		self.children = []
		self.name = name
		self.parent = parent
		self.drawscale = drawscale_

		# Set default layout
		self.line = None
		self.brush = None
		self.pen = QPen(QColor(0,0,0))

		# Inherit layout from referenced object if given
		if inheritLayout is not None:
			self.line = inheritLayout.line
			self.brush = inheritLayout.brush
			self.pen = inheritLayout.pen
		
		# Set specified layout
		if line is not None:
			self.line = line
		if brush is not None:
			self.brush = brush
		if pen is not None:
			self.pen = pen

		# Set translation and rotation
		self.translation = translation
		self.rotation = rotation

		if parent is not None:
			parent.addChild(self)

	def addChild(self, child:'plotTree2d'):
		# Check if the object added is not the root of itself
		if self.getRoot() is child:
			raise ValueError("Cannot add a root to child of itself to avoid recursive plotting")
		else:
			# chech if child is not already a child
			if child in self.children:
				raise ValueError("Cannot add a child that is already a child")
			else:
				# Check if child has a parent
				if child.parent is not None:
					# Remove child from old parent
					if child in child.parent.children:
						child.parent.children.remove(child)
				self.children.append(child)
				child.parent = self

	def getRoot(self):
		if self.parent is None:
			return self
		else:
			return self.parent.getRoot()

	def draw(self, painter:QPainter):
		# Draw self
		if self.line is not None:

			# Set the pen and brush
			painter.setPen(self.pen)
			if self.brush is not None:
				painter.setBrush(self.brush)
			else:
				painter.setBrush(QBrush(Qt.NoBrush))

			# Rotate the hull outline, translate and scale to pixel coordinates
			outline = (np.matmul(rotation_matrix_2d(self.getGlobalRotation()),self.line)+self.getGlobalTranslation()[:, np.newaxis])*self.drawscale

			# print the type of outline [0][0]
			#print("outline type: ", type(outline[0][0]), "outline: ", outline[0][0])
			#rounded_int = np.round(outline[0][0])
			# make a list of QPoint objects and translate to center
			outline_qpoint = []
			for i in range(len(outline[0])):
				point = QPoint(int(np.round(outline[0][i])), int(np.round(outline[1][i])))
				outline_qpoint.append(point)
			
			# Draw the outline
			painter.drawPolygon(QPolygon(outline_qpoint))

		# Draw children (if any)
		for child in self.children:
			child.draw(painter)

	def getGlobalTranslation(self):
		if self.parent is not None:
			# My coordinate system is expressed in my parent's local coordinate system
			return self.parent.getGlobalTranslation() + np.matmul(rotation_matrix_2d(self.parent.getGlobalRotation()),self.translation)
		else:
			# I am root, thus my coordinate system is expressed in the global coordinate system
			return self.translation
	
	def getGlobalRotation(self):
		if self.parent is not None:
			# My coordinate system is expressed in my parent's local coordinate system
			return self.parent.getGlobalRotation() + self.rotation
		else:
			# I am root, thus my coordinate system is expressed in the global coordinate system
			return self.rotation

def rotation_matrix_2d(theta):
	""" Returns a 2D rotation matrix. """
	return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])