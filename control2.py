import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, Vector3, PoseWithCovariance, Pose
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Header, String
from nav_msgs.msg import Odometry
import tf
from math import pi, radians, copysign, sqrt, pow
from numpy import *
import cv2

class Interface: 
	def __init__(self):
		rospy.init_node('interface', anonymous=False)
		rospy.loginfo("To stop TurtleBot CTRL + C")
		rospy.on_shutdown(self.shutdown)
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		rate = rospy.Rate(10);
		
		# Initialize tf listener, and give some time to fill its buffer
		self.tf_listener = tf.TransformListener()
		rospy.sleep(2)

		self.scan_data = LaserScan()
		self.imu_data = Imu()
		self.odom_data = Odometry()
		rospy.Subscriber('odom',Odometry,self.odom_callback)
		rospy.Subscriber('scan',LaserScan,self.scan_callback)
		rospy.Subscriber('mobile_base/sensors/imu_data',Imu,self.imu_callback);
		MAPH = 600
		MAPW = 600
		PLENGTH = 0.1 # length per pixel
		SECURE_DIS = 0.25 # secure distance
		MAP_INFO = 5 # speed of map creating, namely how much information you want a laser point to change the map
		self.gmap = ones((MAPH,MAPW),dtype=uint8)*128
		self.vmap = ones((MAPH,MAPW,3),dtype=uint8)*128
		self.vmap[:,:,0] *= 0
		self.vmap[:,:,1] *= 0
		yaw = 0
		last_yaw = 0
		yaw_flag = False
		last_time = 0
		time = 0
		last_x = 0
		last_y = 0
		px = 0
		py = 0

		while not rospy.is_shutdown():	
			# input: self.scan_data is the laser data, include:
				# float32 angle_min        # start angle of the scan [rad]
				# float32 angle_max        # end angle of the scan [rad]
				# float32 angle_increment  # angular distance between measurements [rad]
				# float32 time_increment   # time between measurements [seconds]
				# float32 scan_time        # time between scans [seconds]
				# float32 range_min        # minimum range value [m]
				# float32 range_max        # maximum range value [m]
				# float32[] ranges         # range data [m]
				# float32[] intensities    # intensity data [device-specific units]
			# input: self.imu_data is the imu data, include:
				# Quaternion orientation
				# Vector3 angular_velocity
				# Vector3 linear_acceleration
			# input: self.odom_data is the pose information, include:
				# header.stamp             # time stamp
				# pose.pose.position       # position in robot coordination
				# twist.twist.linear.x     # linear speed

			#____________________________input end_________________________________

			opx = self.odom_data.pose.pose.position.x
			opy = self.odom_data.pose.pose.position.y # global position			
			if(self.odom_data.header.stamp == 0): continue
			last_time = time
			time = rospy.get_time()		
			if(last_time == 0): continue
			sp = self.odom_data.twist.twist.linear.x
			dr = sp*(time - last_time)
			last_x = px
			last_y = py
			px = last_x + dr*cos(yaw)
			py = last_y + dr*sin(yaw)
			mx = 0
			my = 0
			x = self.imu_data.orientation.x
			y = self.imu_data.orientation.y
			z = self.imu_data.orientation.z
			w = self.imu_data.orientation.w	
			# print(self.scan_data)
			yaw = arctan((2*(w*z+x*y))/(1-2*(y*y+z*z)))	
			if(last_yaw*yaw<-1): yaw_flag = not yaw_flag			
			last_yaw = yaw	
			if(yaw_flag): yaw += pi		
			scan_angle = self.scan_data.angle_min
			for r in self.scan_data.ranges:
				if(isnan(r)): 
					scan_angle += self.scan_data.angle_increment
					continue				
				angle = scan_angle + yaw
				gx = px + r*cos(angle)
				gy = py + r*sin(angle) # global position of laser point
				mpx = int(px/PLENGTH + MAPW/2)
				mpy = int(py/PLENGTH + MAPH/2)
				if(mpx>=0 and mpx<MAPW and mpy>=0 and mpy<MAPH): 
					self.vmap[mpy,mpx,1]=128	# where is the robot now
				mx = int(gx/PLENGTH + MAPW/2)
				my = int(gy/PLENGTH + MAPH/2)
				if(mx<0 or mx>=MAPW or my<0 or my>=MAPH): 
					scan_angle += self.scan_data.angle_increment
					continue 
				# occupancy grid mapping
				delta_x = mx-mpx # calculate the grids where the laser line go through
				delta_y = my-mpy
				if(abs(delta_x) > abs(delta_y)):
					k = float(delta_y)/delta_x
					if(delta_x>0):
						for x in range(mpx+1,mx):
							ty = int(mpy + k*(x-mpx))
							self.gmap[ty,x] = max(self.gmap[ty,x] - MAP_INFO,0)
					else:
						for x in range(mx+1,mpx):
							ty = int(my + k*(x-mx))
							self.gmap[ty,x] = max(self.gmap[ty,x] - MAP_INFO,0)
				else:
					k = float(delta_x)/delta_y
					if(delta_y>0):
						for y in range(mpy+1,my):
							tx = int(mpx + k*(y-mpy))
							self.gmap[y,tx] = max(self.gmap[y,tx] - MAP_INFO,0)
					else:
						for y in range(my+1,mpy):
							tx = int(mx + k*(y-my))
							self.gmap[y,tx] = max(self.gmap[y,tx] - MAP_INFO,0)
				
				self.gmap[my,mx] = min(self.gmap[my,mx] + MAP_INFO*10,255)		
				scan_angle += self.scan_data.angle_increment
			self.vmap[:,:,2] = self.gmap[:,:]
			cv2.imwrite("map.png",cv2.flip(self.vmap,0))			
			
			#___________________________mapping end________________________________
			speed = 0.2
			steer = 0
			# speed and steer is the variable to output
	
			MAX_D = self.gmap.shape[0] + self.gmap.shape[1] 
			#obs = argwhere(self.gmap>200) # where are obstacles
			min_d = MAX_D # find which obstacle point is the nearest
			mx = int(px/PLENGTH + MAPW/2)
			my = int(py/PLENGTH + MAPH/2)			
			x0 = max(mx - 25,0)
			xn = min(mx + 25,MAPW)
			y0 = max(my - 25,0)
			yn = min(my + 25,MAPH)
			obs = argwhere(self.gmap[y0:yn,x0:xn]>200) + [y0,x0]						
			for p in obs:
				# the inner product of (p[1]-mx,p[0]-my) and yaw should be positve
				dis = sqrt((my-p[0])**2 + (mx-p[1])**2)
				if(((p[1]-mx)*cos(yaw)+(p[0]-my)*sin(yaw))<0.5): continue 				
				if(min_d > dis):
					min_d = dis
			min_d = min_d*PLENGTH
			
			if(len(obs)): speed = (min_d-SECURE_DIS)*1 # speed will slow down when robot near obstacles
			else: speed = 0	# speed will be zero when we haven't got the laser data
			
			#print('min_d = {}, speed = {}'.format(min_d,speed))
			if(speed>0.4): speed = 0.4
			if(speed<0.01): speed = 0	
			alter_steer = [-0.6,-0.4,-0.2,0,0.2,0.4,0.6]
			DT = 0.5 # prediction precision
			max_eva = 0
			for s in alter_steer: # choose the best steer in alter set
				future_x = px
				future_y = py
				future_yaw = yaw
				eva = 0
				for t in range(6): # predict the sport of robot in next 3 seconds by evaluate function
					dis = speed*DT
					future_x = future_x + dis*cos(future_yaw)
					future_y = future_y + dis*sin(future_yaw)
					future_yaw = future_yaw + s*DT
					mx = int(future_x/PLENGTH + MAPW/2)
					my = int(future_y/PLENGTH + MAPH/2)
					eva += self.evaluate(my,mx,future_yaw)
				if(eva > max_eva):
					max_eva = eva
					steer = s
				print('steer = {}, its evaluation score = {}'.format(s,eva))
			if(speed==0): steer = 0.4 # if robot is not move ,rotate it to find a way
			print((speed,steer))
			#speed = 0
			#steer = 0
			
			#____________________________output begin_________________________________
			move_cmd = Twist()			
			move_cmd.linear.x = speed
			move_cmd.angular.z = steer
			self.cmd_vel.publish(move_cmd)
			rate.sleep()
			
	def scan_callback(self,data):
		self.scan_data = data
		
	def imu_callback(self,data):
		self.imu_data = data
		
	def odom_callback(self,data):
		self.odom_data = data
  
	def shutdown(self):
		rospy.loginfo("Stopping the robot...")
		self.cmd_vel.publish(Twist())
		rospy.sleep(1)
		
	def isVaild(self,y,x):
		if y<0 or x<0 or y>=self.gmap.shape[0] or x>=self.gmap.shape[1]: return False
		return True
    
	def evaluate(self,y,x,v): # to evaluate whether a pose is good or bad
		if not self.isVaild(y,x): return -19971314
		if self.gmap[y][x] == 128: return -19971314
		W_POSITION = 4 # weight of position
		W_DIRECTION = 1 # weight of direction
		MAX_D = self.gmap.shape[0] + self.gmap.shape[1]
		#obs = argwhere(self.gmap>128) # where are obstacles	
		x0 = max(x - 25,0)
		xn = min(x + 25,self.gmap.shape[1])
		y0 = max(y - 25,0)
		yn = min(y + 25,self.gmap.shape[0])
		obs = argwhere(self.gmap[y0:yn,x0:xn]>128) + [y0,x0]		
		min_d = MAX_D # find which obstacle point is the nearest
		for p in obs:
			# the inner product of (p[1]-x,p[0]-y) and v should be positve
			dis = sqrt((y-p[0])**2 + (x-p[1])**2)
			if(((p[1]-x)*cos(v)+(p[0]-y)*sin(v))/dis<0.5): continue			
			if(min_d > dis):
				min_d = dis
		line_d = 0 # how far is the nearest obstacle if go straight
		for line_d in range(MAX_D):
			mx = int(x + line_d*cos(v))
			my = int(y + line_d*sin(v))
			if(mx>=0 and mx<self.gmap.shape[1] and my>=0 and my<self.gmap.shape[0]): break
			if(self.gmap[my,mx]>100): break
		eva = line_d*W_DIRECTION + min_d*W_POSITION
		return eva
		    		
if __name__=='__main__':
	try:
		Interface()
	except:
		rospy.loginfo("interface node terminated!")
