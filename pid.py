
# pid controller final 

import rclpy
import math
import random

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


kp = 0.2
ki = 0.0
kd = 10

e = 0.0
ei = 0.0
ed = 0.0
epre = 0.0

desired_distance = 0.4
current_distance = 0
output = 0

# main function attached to timer callback
def timer_callback():
    global pub_, twstmsg_
    if ( twstmsg_ != None ):
        pub_.publish(twstmsg_)


def clbk_laser(msg):
    global regions_, twstmsg_
    
    regions_ = {
        #LIDAR readings are anti-clockwise
        'right':  find_nearest(msg.ranges[45:155]),
        'fright': find_nearest (msg.ranges[130:140]),
        'front':  find_nearest (msg.ranges[0:20]),
        'fleft':  find_nearest (msg.ranges[220:230]),
        'left':   find_nearest (msg.ranges[265:275]),
        
    }    
    twstmsg_= movement()

    
# Find nearest point
def find_nearest(list):
    f_list = filter(lambda item: item > 0.0, list)  # exclude zeros
    return min(min(f_list, default=10), 10)




#Basic movement method
def movement():
    global pub_
    global regions_, mynode_
    global e,ei,epre,ed,kp,kd,ki,desired_distance,current_distance
    regions = regions_
    # print("Min distance in right region: ", regions['right'])
    
    #create an object of twist class, used to express the linear and angular velocity of the turtlebot 
    msg = Twist()

    current_distance = regions['right']
      
    e = desired_distance - current_distance
    ei = ei + e
    ed = e - epre
    epre = e

    
    output = (kp * e) + (ki * ei) + (kd * ed)
    msg.angular.z = output
    print(e)
    # print("angular motion: ", msg.angular.z)
    msg.linear.x = -0.08

    return msg

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
    except Exception:
        pub_.publish(Exception)
        stop()  # stop the robot
    finally:
        # Clean up
        mynode_.destroy_timer(timer)
        mynode_.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
