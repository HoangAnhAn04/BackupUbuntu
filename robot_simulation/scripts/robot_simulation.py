import rospy
import numpy as np
import tf
from geometry_msgs.msg import Pose2D, Twist
from visualization_msgs.msg import Marker
from robot_msgs.srv import ChangeColor, ChangeColorRequest, ChangeColorResponse

class  RobotSimulation:
  def __init__(self):
    self.robot_pose = Pose2D()
    self.robot_rviz = Marker()

    self.robot_pose_publisher = rospy.Publisher("robot_pose", Pose2D, queue_size=1)
    self.robot_rviz_publisher = rospy.Publisher("robot_rviz", Marker, queue_size=1)

    self.cmd_vel_subscriber = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_cb)
    self.change_color_srv = rospy.Service("change_color", ChangeColor, self.change_color_cb) 

    self.robot_rviz.color.r = 1.0
    self.robot_rviz.color.a = 1.0

    self.time_frame = rospy.Time.now().to_sec()

  def cmd_vel_cb(self, data):
    new_time_frame = rospy.Time.now().to_sec()
    dt = new_time_frame - self.time_frame
    self.time_frame = new_time_frame

    dtheta = data.angular.z * dt
    dx = data.linear.x * dt * np.cos(self.robot_pose.theta + dtheta/2)
    dy = data.linear.x * dt * np.sin(self.robot_pose.theta + dtheta/2)

    self.robot_pose.x += dx
    self.robot_pose.y += dy
    self.robot_pose.theta += dtheta

    self.robot_pose_publisher.publish(self.robot_pose)

    self.robot_rviz.header.stamp = rospy.Time.now()
    self.robot_rviz.header.frame_id = "map"
    self.robot_rviz.type = Marker.CUBE
    self.robot_rviz.action = Marker.ADD
    
    self.robot_rviz.pose.position.x = self.robot_pose.x
    self.robot_rviz.pose.position.y = self.robot_pose.y
    self.robot_rviz.pose.position.z = 0.0

    quaternion = tf.transformations.quaternion_from_euler(0, 0, self.robot_pose.theta)
    self.robot_rviz.pose.orientation.x = quaternion[0]
    self.robot_rviz.pose.orientation.y = quaternion[1]
    self.robot_rviz.pose.orientation.z = quaternion[2]
    self.robot_rviz.pose.orientation.w = quaternion[3]

    self.robot_rviz.scale.x = 1.0
    self.robot_rviz.scale.y = 0.4
    self.robot_rviz.scale.z = 0.3
    
    # self.robot_rviz.color.r = 1.0
    # self.robot_rviz.color.a = 1.0

    self.robot_rviz_publisher.publish(self.robot_rviz)

  def  change_color_cb(self, req):
    self.robot_rviz.color = req.color
    res = ChangeColorResponse()
    res.success = True
    res.message = "OK"
    return res

def main():
  rospy.init_node("robot_simulation")
  robot_simulation= RobotSimulation()
  rospy.spin()
main()