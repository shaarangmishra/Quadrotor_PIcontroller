import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, Twist
import tf
import numpy as np
import threading
import math

class controller():
    def __init__(self):
        self.pose_subscriber = rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, self.update_pose)
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10 )# define a publisher that publishes Twist messages on the /cmd_vel topic

        self.k_position = 1.5
        self.k_angle = 0.1
        self.k_vel = 1
        self.k_avel = 0.5
        

        self.rate = rospy.Rate(10)
   

    def update_pose(self, data):
        #update current pose of the robot
        #self.pose_stamped = data
        self.current_x = data.pose.position.x
        self.current_y = data.pose.position.y
        self.current_z = data.pose.position.z
        self.roll, self.pitch, self.yaw = self.quaternion_to_euler(data.pose)


    def get_goal(self):
        #getting goal position and angle from user
        self.goal_x = input("Set your x goal:")
        self.goal_y = input("Set your y goal:")
        self.goal_z = input("Set your z goal:")
        angle_deg = input("Set your angle goal:")
        self.goal_angle = angle_deg*np.pi/180
        print("goal selected")

    def get_goal_cont(self):
        while not rospy.is_shutdown():
            self.get_goal()

    def quaternion_to_euler(self, robot_pose):
        #transforms robot position from quaternion to roll, pitch, yaw
        quaternion = (
            robot_pose.orientation.x,
            robot_pose.orientation.y,
            robot_pose.orientation.z,
            robot_pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        return roll, pitch, yaw

    def control_law(self):
        #p_controller,
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        alpha = math.atan2(dx, dy)
        len_v = math.sqrt((dx **2) + (dy **2))
        delta_ang = alpha + self.yaw
        delta_x = len_v * math.sin(delta_ang)
        delta_y = len_v * math.cos(delta_ang)
        #define a control law here, use the "self.goal" varibles and "self.pose_stamped" variables here
        ux = self.k_position * delta_x
        uy = self.k_position * delta_y
        uz = self.k_position * (self.goal_z - self.current_z)
        uw = self.k_angle * (self.goal_angle - self.yaw)
        #return the control values
        return ux,uy,uz,uw

    def move2goal(self):
        #uses control values and publishes velocities in x,y,z and angular_z directions.

        while not rospy.is_shutdown():
            ux,uy,uz,uw = self.control_law()
            speed = Twist()
            speed.linear.x = ux * self.k_vel
            speed.linear.y =  uy * self.k_vel
            speed.linear.z =  uz * self.k_vel
            speed.angular.x = 0 
            speed.angular.y = 0
            speed.angular.z = uw * self.k_avel

            self.vel_publisher.publish(speed)
            self.rate.sleep()

        rospy.spin()
if __name__ == '__main__':
    try:
        rospy.init_node('p_controller')
        p_controller = controller()
        rospy.sleep(1) # waiting for the initial pose update
        p_controller.get_goal()
        t1 = threading.Thread(name='thread1', target=p_controller.get_goal_cont)
        t2 = threading.Thread(name='thread2', target=p_controller.move2goal)
        t1.start()
        t2.start()
    except rospy.ROSInterruptException: pass
