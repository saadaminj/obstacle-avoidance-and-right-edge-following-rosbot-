
import rclpy
import math
import logging
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformRegistration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

mynode_ = None
pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}

twstmsg_ = None
_mymsg = None

weights = list()
near = 0
middle = 0
far = 0

distance_set = [0,0.2,0.5,0.8]

fast_firing_strength = list()
medium_firing_strength = list()
slow_firing_strength = list()

rf_weights = list()
rb_weights = list()

fr_weights = list()
f_weights = list()
fl_weights = list()		

right_firing_strength = list()
left_firing_strength = list()

medium_centroid = 0.2
fast_centroid = 0.3
slow_centroid = 0.1

left_centroid = 1
right_centroid = -1

speed = 0
steer = 0

# this function implements the equation for rising shape membership
def rising_edge(a,b,x):
	return (x - a) / (b - a)

# this function implements the equation for falling shape membership	
def falling_edge(a,b,x):
	return (b - x) / (b - a)

# this function calculates the degree of membership in near, medium and far categories
def distance_fuzzy_value(lidar_value):
	if lidar_value <= distance_set[1]:
		near = 1
		middle = 0
		far = 0

	elif lidar_value > distance_set[1] and lidar_value <= distance_set[2]:
		near = falling_edge(distance_set[1],distance_set[2],lidar_value)
		middle = rising_edge(distance_set[1],distance_set[2],lidar_value)
		far = 0

	elif lidar_value > distance_set[2] and lidar_value <= distance_set[3]:
		near = 0
		middle = falling_edge(distance_set[2],distance_set[3],lidar_value)
		far = rising_edge(distance_set[2],distance_set[3],lidar_value)
		
	elif lidar_value > distance_set[3]:
		near = 0
		middle = 0
		far = 1

	return near, middle, far

# main function attached to timer callback
def timer_callback():
    global pub_, twstmsg_
    if ( twstmsg_ != None ):
        pub_.publish(twstmsg_)


def clbk_laser(msg):
	global regions_, twstmsg_, _mymsg

	_mymsg = msg

	regions_ = {
		#LIDAR readings are anti-clockwise
		'right':  find_nearest(msg.ranges[85:95]),
		'fright': find_nearest (msg.ranges[130:140]),
		'front':  find_nearest (msg.ranges[175:185]),
		'fleft':  find_nearest (msg.ranges[220:230]),
		'left':   find_nearest (msg.ranges[265:275]),	
	}
	try:
		twstmsg_= movement()    
	except Exception as e:
		print(e)

		
	# this function calculates speed and steer by degree of membership in obstacle avoidance and right edge following for front sensor
def oa_ref_combined(msg, regions, lidar_reading):

	ref = [0,0]
	oa = [0,0]
	
	combined_set = [0.17, 0.25, 0.5]
	
	# find nearest value from the front of the robot ranges
	x = find_nearest(lidar_reading.ranges[165:205])

	# if nearest distance to anything on the front is greater than 0.5 so do right edge following
	if x > combined_set[2]:
		ref = rightEdgeFollowing(msg,regions,lidar_reading)
		msg.linear.x = ref.linear.x
		msg.angular.z = ref.angular.z	

	# if nearest distance to anything on the front is lesser than 0.17 so do obstacle avoidance
	if x <= combined_set[0]:
		oa = obstacleAvoidance(msg,regions,lidar_reading)
		msg.linear.x = oa.linear.x
		msg.angular.z = oa.angular.z

	# if nearest distance to anything on the front is greater than 0.17 and less than 0.5 then implement fuzzy logic so that the outputs merge both features i.e OA and REF
	if x > combined_set[0] and x <= combined_set[2]:

		# get result for both oa and ref
		oa = obstacleAvoidance(msg,regions,lidar_reading)
		ref = rightEdgeFollowing(msg,regions,lidar_reading)

		# calculate intersection point of both ref and oa graph and multiply it with the respective linear velocity and angular velocity
		msg.linear.x = ((oa.linear.x * falling_edge(combined_set[1],combined_set[2],x)) + (ref.linear.x * rising_edge(combined_set[1],combined_set[2],x)))
		msg.angular.z = (oa.angular.z * falling_edge(combined_set[1],combined_set[2],x)) + (ref.angular.z* rising_edge(combined_set[1],combined_set[2],x))
	
	return msg
	

	
# this function calculate crisp values from range input
def rightEdgeFollowing(msg, regions, _mymsg):

	# reset all lists
	left_firing_strength.clear()
	right_firing_strength.clear()
	medium_firing_strength.clear()
	slow_firing_strength.clear()
	fast_firing_strength.clear()

	# calculates the degree of membership in each category for right back sensor
	back_right_near, back_right_middle, back_right_far = distance_fuzzy_value(find_nearest(_mymsg.ranges[15:75]))
 
	# calculates the degree of membership in each category for right front sensor
	front_right_near, front_right_middle, front_right_far = distance_fuzzy_value(find_nearest(_mymsg.ranges[105:165]))
	
	if back_right_near > 0 and front_right_near > 0 :
		left_firing_strength.append(min(front_right_near, back_right_near))
		slow_firing_strength.append(min(front_right_near, back_right_near))
		
	if back_right_near > 0 and front_right_middle > 0 :
		right_firing_strength.append(min(front_right_middle, back_right_near))
		medium_firing_strength.append(min(front_right_middle, back_right_near))
  
	if back_right_near > 0 and front_right_far > 0:
		right_firing_strength.append(min(front_right_far, back_right_near))
		slow_firing_strength.append(min(front_right_far, back_right_near))
  
	if back_right_middle > 0 and front_right_near > 0:
		left_firing_strength.append(min(front_right_near, back_right_middle))
		slow_firing_strength.append(min(front_right_near, back_right_middle))
		
	if back_right_middle > 0 and front_right_middle > 0:
		left_firing_strength.append(min(front_right_middle, back_right_middle))
		medium_firing_strength.append(min(front_right_middle, back_right_middle))
		right_firing_strength.append(min(front_right_middle, back_right_middle))

	if back_right_middle > 0 and front_right_far > 0 :
		right_firing_strength.append(min(front_right_far, back_right_middle))
		medium_firing_strength.append(min(front_right_far, back_right_middle))
  
	if back_right_far > 0 and front_right_near > 0:
		left_firing_strength.append(min(front_right_near, back_right_far))
		slow_firing_strength.append(min(front_right_near, back_right_far))
  
	if  back_right_far > 0 and front_right_middle > 0 :
		left_firing_strength.append(min(front_right_middle, back_right_far))
		medium_firing_strength.append(min(front_right_middle, back_right_far))
	
	if back_right_far > 0 and front_right_far > 0:
		right_firing_strength.append(min(front_right_far, back_right_far))
		fast_firing_strength.append(min(front_right_far, back_right_far))
		
	msg.linear.x = -((slow_centroid * sum(slow_firing_strength) + medium_centroid * sum(medium_firing_strength) + fast_centroid * sum(fast_firing_strength)) / (sum(slow_firing_strength) + sum(medium_firing_strength) + sum(fast_firing_strength)))
	msg.angular.z = (right_centroid * sum(right_firing_strength) + left_centroid * sum(left_firing_strength) ) / (sum(right_firing_strength)+ sum(left_firing_strength))
	
	return msg
	
# this function calculate crisp values from range input		
def obstacleAvoidance(msg, regions, _mymsg):
	
	# reset all lists
	fast_firing_strength.clear()
	left_firing_strength.clear()
	right_firing_strength.clear()
	slow_firing_strength.clear()
	medium_firing_strength.clear()

	# calculates the degree of membership in each category for front left sensor
	front_left_near, front_left_middle, front_left_far  = distance_fuzzy_value(find_nearest(_mymsg.ranges[210:250]))
 
	# calculates the degree of membership in each category for front right sensor
	front_right_near, front_right_middle, front_right_far = distance_fuzzy_value(find_nearest(_mymsg.ranges[100:135]))
	
	# calculates the degree of membership in each category for front sensor
	front_weights_near, front_weights_middle, front_weights_far = distance_fuzzy_value(find_nearest(_mymsg.ranges[157:202]))
	
	
	if front_weights_near > 0 and front_left_near > 0 and front_right_near > 0 :
		left_firing_strength.append(min(front_right_near, front_weights_near, front_left_near))
		slow_firing_strength.append(min(front_right_near, front_weights_near, front_left_near))
		
	if front_weights_near > 0 and front_left_middle > 0 and front_right_near > 0 :
		left_firing_strength.append(min(front_right_near, front_weights_near, front_left_middle))
		slow_firing_strength.append(min(front_right_near, front_weights_near, front_left_middle))
		
	if front_weights_near > 0 and front_left_far > 0 and front_right_near > 0 :
		left_firing_strength.append(min(front_right_near, front_weights_near, front_left_far))
		slow_firing_strength.append(min(front_right_near, front_weights_near, front_left_far))
		
	if front_weights_near > 0 and front_left_near > 0 and front_right_middle > 0:
		right_firing_strength.append(min(front_right_middle, front_weights_near, front_left_near))
		slow_firing_strength.append(min(front_right_middle, front_weights_near, front_left_near))
		
	if front_weights_near > 0 and front_left_middle > 0 and front_right_middle > 0:
		left_firing_strength.append(min(front_right_middle, front_weights_near, front_left_middle))
		slow_firing_strength.append(min(front_right_middle, front_weights_near, front_left_middle))
		
	if front_weights_near > 0 and front_left_far > 0 and front_right_middle > 0:
		left_firing_strength.append(min(front_right_middle, front_weights_near, front_left_far))
		slow_firing_strength.append(min(front_right_middle, front_weights_near, front_left_far))
		
	if front_weights_near > 0 and front_left_near > 0 and front_right_far > 0:
		right_firing_strength.append(min(front_right_far, front_weights_near, front_left_near))
		slow_firing_strength.append(min(front_right_far, front_weights_near, front_left_near))
		
	if front_weights_near > 0 and front_left_middle > 0 and front_right_far > 0:
		right_firing_strength.append(min(front_right_far, front_weights_near, front_left_middle))
		slow_firing_strength.append(min(front_right_far, front_weights_near, front_left_middle))
		
	if front_weights_near > 0 and front_left_far > 0 and front_right_far > 0:
		left_firing_strength.append(min(front_right_far, front_weights_near, front_left_far))
		slow_firing_strength.append(min(front_right_far, front_weights_near, front_left_far))
		
	if front_weights_middle > 0 and front_left_near > 0 and front_right_near > 0:
		left_firing_strength.append(min(front_right_near, front_weights_middle, front_left_near))
		right_firing_strength.append(min(front_right_near, front_weights_middle, front_left_near))
		slow_firing_strength.append(min(front_right_near, front_weights_middle, front_left_near))
		
	if front_weights_middle > 0 and front_left_middle > 0 and front_right_near > 0:
		left_firing_strength.append(min(front_right_near, front_weights_middle, front_left_middle))
		slow_firing_strength.append(min(front_right_near, front_weights_middle, front_left_middle))
		
	if front_weights_middle > 0 and front_left_far > 0 and front_right_near > 0 :
		left_firing_strength.append(min(front_right_near, front_weights_middle, front_left_far))
		slow_firing_strength.append(min(front_right_near, front_weights_middle, front_left_far))
		
	if front_weights_middle > 0 and front_left_near > 0 and front_right_middle > 0:
		right_firing_strength.append(min(front_right_middle, front_weights_middle, front_left_near))
		slow_firing_strength.append(min(front_right_middle, front_weights_middle, front_left_near))
		
	if front_weights_middle > 0 and front_left_middle > 0 and front_right_middle > 0:
		left_firing_strength.append(min(front_right_middle, front_weights_middle, front_left_middle))
		medium_firing_strength.append(min(front_right_middle, front_weights_middle, front_left_middle))
		
	if front_weights_middle > 0 and front_left_far > 0 and front_right_middle > 0 :
		left_firing_strength.append(min(front_right_middle, front_weights_middle, front_left_far))
		medium_firing_strength.append(min(front_right_middle, front_weights_middle, front_left_far))
		
	if front_weights_far > 0 and front_left_near > 0 and front_right_near > 0 :
		left_firing_strength.append(min(front_right_near, front_weights_far, front_left_near))
		right_firing_strength.append(min(front_right_near, front_weights_far, front_left_near))
		slow_firing_strength.append(min(front_right_near, front_weights_far, front_left_near))
		
	if front_weights_middle > 0 and front_left_near > 0 and front_right_far > 0:
		right_firing_strength.append(min(front_right_far, front_weights_middle, front_left_near))
		slow_firing_strength.append(min(front_right_far, front_weights_middle, front_left_near))
		
	if front_weights_middle > 0 and front_left_middle > 0 and front_right_far > 0:
		right_firing_strength.append(min(front_right_far, front_weights_middle, front_left_middle))
		medium_firing_strength.append(min(front_right_far, front_weights_middle, front_left_middle))
		
	if front_weights_middle > 0 and front_left_far > 0 and front_right_far > 0 :
		left_firing_strength.append(min(front_right_far, front_weights_middle, front_left_far))
		medium_firing_strength.append(min(front_right_far, front_weights_middle, front_left_far))
		
	if front_weights_far > 0 and front_left_middle > 0 and front_right_near > 0 :
		left_firing_strength.append(min(front_right_near, front_weights_far, front_left_middle))
		right_firing_strength.append(min(front_right_near, front_weights_far, front_left_middle))
		slow_firing_strength.append(min(front_right_near, front_weights_far, front_left_middle))
		
	if front_weights_far > 0 and front_left_far > 0 and front_right_near > 0 :
		left_firing_strength.append(min(front_right_near, front_weights_far, front_left_far))
		slow_firing_strength.append(min(front_right_near, front_weights_far, front_left_far))
		
	if front_weights_far > 0 and front_left_near > 0 and front_right_middle > 0:
		right_firing_strength.append(min(front_right_middle, front_weights_far, front_left_near))
		slow_firing_strength.append(min(front_right_middle, front_weights_far, front_left_near))
		
	if front_weights_far > 0 and front_left_middle > 0 and front_right_middle > 0:
		left_firing_strength.append(min(front_right_middle, front_weights_far, front_left_middle))
		right_firing_strength.append(min(front_right_middle, front_weights_far, front_left_middle))
		slow_firing_strength.append(min(front_right_middle, front_weights_far, front_left_middle))
		
	if front_weights_far > 0 and front_left_far > 0 and front_right_middle > 0:
		left_firing_strength.append(min(front_right_middle, front_weights_far, front_left_far))
		medium_firing_strength.append(min(front_right_middle, front_weights_far, front_left_far))
		
	if front_weights_far > 0 and front_left_near > 0 and front_right_far > 0 :
		right_firing_strength.append(min(front_right_far, front_weights_far, front_left_near))
		slow_firing_strength.append(min(front_right_far, front_weights_far, front_left_near))
		
	if front_weights_far > 0 and front_left_middle > 0 and front_right_far > 0:
		right_firing_strength.append(min(front_right_far, front_weights_far, front_left_middle))
		medium_firing_strength.append(min(front_right_far, front_weights_far, front_left_middle))
		
	if front_weights_far > 0 and front_left_far > 0 and front_right_far > 0:
		left_firing_strength.append(min(front_right_far, front_weights_far, front_left_far))
		right_firing_strength.append(min(front_right_far, front_weights_far, front_left_far))
		fast_firing_strength.append(min(front_right_far, front_weights_far, front_left_far))
	
	msg.linear.x = -((slow_centroid * sum(slow_firing_strength) + medium_centroid * sum(medium_firing_strength) + fast_centroid * sum(fast_firing_strength)) / (sum(slow_firing_strength) + sum(medium_firing_strength) + sum(fast_firing_strength)))
	msg.angular.z = (left_centroid * sum(left_firing_strength) + right_centroid * sum(right_firing_strength)) / (sum(left_firing_strength) + sum(right_firing_strength))
 
	return msg


# Find nearest point
def find_nearest(list):
    f_list = filter(lambda item: item > 0.0, list)  # exclude zeros
    return min(min(f_list, default=10), 10)

#Basic movement method
def movement():
    global regions_, mynode_, _mymsg
    regions = regions_
    print("Min distance in 360 region: ", _mymsg.ranges[359])
    print("Min distance in 270 region: ", _mymsg.ranges[270])
    print("Min distance in 180 region: ", _mymsg.ranges[180])
    print("Min distance in 90 region: ", _mymsg.ranges[90])
    print("Min distance in 0 region: ", _mymsg.ranges[0])

    #create an object of twist class, used to express the linear and angular velocity of the turtlebot 
    msg = Twist()
    return oa_ref_combined(msg, regions, _mymsg)
	
#used to stop the rosbot
def stop():
    global pub_
    msg = Twist()
    msg.angular.z = 0.0
    msg.linear.x = 0.0
    pub_.publish(msg)


def main():
    global pub_, mynode_

    rclpy.init()
    mynode_ = rclpy.create_node('reading_laser')

    # define qos profile (the subscriber default 'reliability' is not compatible with robot publisher 'best effort')
    qos = QoSProfile(
        depth=10,
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    )

    # publisher for twist velocity messages (default qos depth 10)
    pub_ = mynode_.create_publisher(Twist, '/cmd_vel', 10)

    # subscribe to laser topic (with our qos)
    sub = mynode_.create_subscription(LaserScan, '/scan', clbk_laser, qos)

    # Configure timer
    timer_period = 0.2  # seconds 
    timer = mynode_.create_timer(timer_period, timer_callback)

    # Run and handle keyboard interrupt (ctrl-c)
    try:
        rclpy.spin(mynode_)
    except KeyboardInterrupt:
        stop()  # stop the robot
    except:
        stop()  # stop the robot
    finally:
        # Clean up
        mynode_.destroy_timer(timer)
        mynode_.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
