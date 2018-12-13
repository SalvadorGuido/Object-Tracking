from numpy import * 
from numpy.linalg import inv 
import matplotlib.pyplot as plt 

class Kalman: # Definición de clase, class name_of_the_class
	def _init_(self):
		####### X[k-1] ###########
		self.x = None 
		self.y = None 
		self.Vx = None 
		self.Vy = None 
		self.Ax = None
		self.Ay = None 
		######### Initial Values #############
		self.Dt = None 
		self.YawRate = None
		self.cosYawAngle = None 
		self.sinYawAngle = None
		self.FA = None 
		self.PNC = 2.25 
		self.InitValOfP = 1000
		self.VelStabilityFactor = None 
		self.SignedEgoSpeed = None
		self.Overground = None 
		self.MountingtoCenterY = None 
		self.MountingtoCenterX = None 
		self.SignedEgoAcel = None
		############# Kalman´s Parameters ##############
		self.X = None
		self.A = None 
		self.P = None 
		self.Q = None 
		self.H = None 
		self.R = None 
		self.I = None 
		############# New X(k-1) ############
		self.Oldx = None
		self.Oldy = None
		self.OldVx = None
		self.OldVy = None 
		###################### Relative Velocities ###########################
		self.VelRelVx = None
		self.VelRelVy = None
		self.RelVelVx = None
		self.RelVelVy = None  
		###################### Aceleration - Framawork  ######################
		self.AB1 = None
		self.AB2 = None
		################ Position prediction ####################
		self.XNew = None
		############## Velocities prediction #####################
		################ Aceleration prediction ####################
		#self. = array([[0],[0],[0],[0],[0],[0]]) 

	def set_initialKalVal(self, posx, posy, velx, vely, dt, yawRate, sinYangle, cosYangle, egoSpeed, mountingCenterY, mountingCenterX, egoAccel):
		self.x = posx 
		self.y = posy 
		self.Vx = velx 
		self.Vy = vely 
		self.Ax = 0
		self.Ay = 0
		self.Dt = dt 
		self.YawRate = yawRate 
		self.cosYawAngle = cosYangle 
		self.sinYawAngle = sinYangle
		self.SignedEgoSpeed = egoSpeed
		self.MountingtoCenterY = mountingCenterX
		self.MountingtoCenterX = mountingCenterY
		self.SignedEgoAcel = egoAccel
		self.FA = 1-(0.3*self.Dt) 
		self.X = array([[self.x],[self.Vx],[self.Ax],[self.y],[self.Vy],[self.Ay]]) 



	def Matrix_A_P_Q_H_R_I(self):
		# Transition Matrix 
		self.A = array( [ [1, self.Dt, 0, self.YawRate*self.Dt, 0, 0], [0, 1, self.Dt, 0, self.YawRate*self.Dt, 0], 
    	[0, 0, self.cosYawAngle*self.FA, 0, 0, self.sinYawAngle*self.FA], [-self.YawRate*self.Dt, 0, 0, 1, self.Dt, 0], 
    	[0, -self.YawRate*self.Dt, 0, 0, 1, self.Dt], [0, 0, -self.sinYawAngle*self.FA, 0, 0, self.cosYawAngle*self.FA] ] )
    	# Covariance Matrix of The Process
		#self.P = diag( ( self.InitValOfP ,self.InitValOfP ,self.InitValOfP ,self.InitValOfP ,self.InitValOfP ,self.InitValOfP ) )
		# Process Noise Matrix
		#self.Q = diag( ( self.PNC*self.Dt*self.Dt , self.PNC*self.Dt*self.Dt, self.PNC*self.Dt*self.Dt, self.PNC*self.Dt*self.Dt,
		#self.PNC*self.Dt*self.Dt, self.PNC*self.Dt*self.Dt ) )
		# Transformation Matrix 
		#self.H = diag( ( 1 ,1 ,1 ,1 ,1 ,1 ) )
		# Radar Sensitivity
		#self.R = diag( ( 25 ,25 ,6 ,6 ,1 ,1 ) )
		# Identity Matrix
		#I = identity(6)

	def OldStateVector(self):
		self.Oldx = self.X[0]
		self.Oldy = self.X[3]
		self.OldVx = self.X[1]*self.VelStabilityFactor
		self.OldVy = self.X[4]*self.VelStabilityFactor

	def RelativeVelocities(self):
		self.VelRelVx =  self.OldVx - self.SignedEgoSpeed + self.YawRate*(self.Oldy + self.MountingtoCenterY)
		self.VelRelVy = self.OldVy -  self.YawRate*(self.Oldx + self.MountingtoCenterX)

	def AceleratioFramework(self):
		self.AB1 = self.X[2]*self.cosYawAngle + self.X[5].sinYawAngle
		self.AB2 = -self.X[2]*self.sinYawAngle + self.X[5]*self.cosYawAngle

	def KalmanFilter_Predict(self):
		self.X[0] =  self.VelRelVx*self.Dt
		self.X[3] =  self.VelRelVy*self.Dt
		self.X[1] = (self.Ax - self.SignedEgoAcel + self.YawRate*self.OldVy)*self.Dt
		self.X[4] = (self.Ay - self.YawRate*self.OldVx)*self.Dt
		self.X[2] *= self.FA
		self.X[5] *= self.FA
		self.P = dot(self.A, dot(self.P, self.A.T))
	


















