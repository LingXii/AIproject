import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, Vector3
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Header, String
import tf
from math import pi, radians, copysign, sqrt, pow

class Interface: 
	def __init__(self):
		rospy.init_node('interface', anonymous=False)
		rospy.loginfo("To stop TurtleBot CTRL + C")
		rospy.on_shutdown(self.shutdown)
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		r = rospy.Rate(10);
		
		# Initialize tf listener, and give some time to fill its buffer
		self.tf_listener = tf.TransformListener()
		rospy.sleep(2)

		self.odom_frame = '/odom'
		self.base_frame = '/base_footprint'
		self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))
		self.scan_data = LaserScan()
		self.imu_data = Imu()
		rospy.Subscriber('scan',LaserScan,self.scan_callback)
		rospy.Subscriber('mobile_base/sensors/imu_data',Imu,self.imu_callback);

		while not rospy.is_shutdown():		
			try:
				(trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0)) 
			except (tf.Exception, tf.ConnectivityException, tf.LookupException): 
				rospy.loginfo("TF Exception")
			# input: trans is position(x,y,z)
			# input: rot is angle(x,y,z,c) where c is the covariance
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

			#____________________________input_________________________________
			speed = 0 
			steer = 0 
			# speed and steer is the variable to output
			
			#____________________________output_________________________________
			move_cmd = Twist()
			move_cmd.linear.x = speed
			move_cmd.angular.z = steer
			self.cmd_vel.publish(move_cmd)
			r.sleep()
			
	def scan_callback(self,data):
		self.scan_data = data
		
	def imu_callback(self,data):
		self.imu_data = data
  
	def shutdown(self):
		rospy.loginfo("Stopping the robot...")
		self.cmd_vel.publish(Twist())
		rospy.sleep(1)
    
if __name__=='__main__':
	try:
		Interface()
	except:
		rospy.loginfo("interface node terminated!")
