#!/usr/bin/env python
import time
import json
import math
import rospy
import socket
import threading
import actionlib
import PyKDL
import tf

import socket
import fcntl
import struct
import uuid

# from black_maine_app.msg import DistributionAction
import geometry_msgs.msg
import black_maine_app.msg
import diagnostic_msgs.msg

import paho.mqtt.client as mqtt

class MqttListener:

  def __init__(self):

    self.robot_id = rospy.get_param('mqtt_listener/robot_id', "truck_01")

    self.ip_address = ""
    self.mac_address = ""
    self.battery = 0.0
    self.charge_state = ""
    self.robot_state = "idel"
    self.amcl_position = {"x": 0, "y": 0, "z": 0}
    self.amcl_orientation = {"r": 0, "p": 0, "y": 0}
    # self.diagnostics = diagnostic_msgs.msg.DiagnosticArray
    self.task_finished = False
    self.feedback_msg = []
    self.client = mqtt.Client(client_id=self.robot_id)
    self.client.on_connect = self.on_connect
    self.client.on_message = self.on_message
    self.client.connect("192.168.30.2", 1883, 60)

    self._sub_feedback = rospy.Subscriber("black_maine_distribution_server/feedback", black_maine_app.msg.DistributionActionFeedback, self.FeedbackCallback)
    self._sub = rospy.Subscriber("amcl_pose", geometry_msgs.msg.PoseWithCovarianceStamped, self.AmclPoseCallback)
    self._sub = rospy.Subscriber("diagnostics", diagnostic_msgs.msg.DiagnosticArray, self.DiagnosticCallback)
    self._sub_result = rospy.Subscriber("black_maine_distribution_server/result", black_maine_app.msg.DistributionActionResult, self.ResultCallback)

    self._turtlebot_client = actionlib.SimpleActionClient("black_maine_distribution_server", black_maine_app.msg.DistributionAction)

  def DiagnosticCallback(self, msg):
    # print(type(msg.status))
    # print(len(msg.status))
    if len(msg.status) != 0:
      if msg.status[0].name == "mobile_base_nodelet_manager: Battery":
        self.battery = msg.status[0].values[1].value
        self.charge_state = msg.status[0].values[5].value

  def AmclPoseCallback(self, msg):
    # print("amcl callback")
    self.amcl_position["x"] =  msg.pose.pose.position.x
    self.amcl_position["y"] =  msg.pose.pose.position.y
    self.amcl_position["z"] =  msg.pose.pose.position.z
    x = msg.pose.pose.orientation.x
    y = msg.pose.pose.orientation.y
    z = msg.pose.pose.orientation.z
    w = msg.pose.pose.orientation.w
    (self.amcl_orientation["r"], self.amcl_orientation["p"], self.amcl_orientation["y"]) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    # self.amcl_orientation["r"] = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
    # self.amcl_orientation["p"] = math.asin(2*(w*y-z*z))
    # self.amcl_orientation["y"] = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))

  def UDPLinstenLoop(self):
    # rospy.loginfo("start udp linster..")
    self.client.loop_start()

  def RobotStatePublisher(self):
    # rate = rospy.Rate(10)
    version = rospy.get_param("mqtt_listener/version")
    while not rospy.is_shutdown():
      # robot_id = rospy.get_param("robot_id")
      if self.robot_state == "busy":
        detail = "Task in progress"
      elif self.robot_state == "idel":
        detail = "No task"
      # print("robot state publish")
    # Task in progress
      # print(type(self.amcl_pose))
      feedback_msg = { "message_type": "state", "id": self.robot_id, "version": version, "state": self.robot_state, "details": detail,
                       "battery": self.battery, "charge_state": self.charge_state, "type": 'truck', "ip": self.ip_address, "mac_address": self.mac_address,
                       "pose": self.amcl_position, "orientation": self.amcl_orientation, "localization_score":100.0}
      feedback_json = json.dumps(feedback_msg)
      # print(feedback_json)
      self.client.publish("iqr/usc/robot/state", feedback_json)
      
      # rate.sleep()
      time.sleep(2.0)


  # The callback for when the client receives a CONNACK response from the server.
  def on_connect(self, client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    # self.client.subscribe([("iqr/usc/callbutton/state", 0), ("iqr/usc/agv_manager/state", 0)])
    self.client.subscribe("iqr/usc/agv_manager/message")
    self.client.message_callback_add("iqr/usc/agv_manager/message", self.callback)
    # message_callback_add(sub, callback)

  def callback(self, client, userdata, msg):
    # pass
    rospy.loginfo("callbutton callback ")
    # print(msg.payload)

    # msg_dist = "{"message_type":"state","id":"btn_01","state":"ready","ip":"192.168.0.184","mac_address":"98:F4:AB:23:0F:BC"}"
    msg_dist = eval(msg.payload)
    robot_id = msg_dist["robot_id"]
    task_id = msg_dist["task_id"]
    goal_list = msg_dist["goal_list"]

    goal_list_goal = []
    confirm_list = []
    # print(type(goal_list))
    for x in xrange(0, len(goal_list)):
      goal_dist = goal_list[x]
      goal_list_goal.append(goal_dist["goal"])
      if goal_dist["need_confirm"]:
        confirm_list.append("True")
      else:
        confirm_list.append("False")
      # print(goal_dist["goal"])
    if robot_id == self.robot_id:
      self.robot_state = "busy"
      self.SendGoal(robot_id, task_id, goal_list_goal, confirm_list)
    # print(goal_list)


  # The callback for when a PUBLISH message is received from the server.
  def on_message(self, client, userdata, msg):
      # rospy.loginfo("mqtt receive msg ")
      # print(msg.topic+" "+str(msg.payload))
      pass

  def SendGoal(self, robot_id, task_id, goal_list_goal, confirm_list):
    action_name = "/black_maine_distribution_server"
    goal_msg = black_maine_app.msg.DistributionGoal()
    goal_msg.task_id = task_id
    goal_msg.places = goal_list_goal
    goal_msg.confirm_list = confirm_list
    # print(type(goal_msg.places[0]))
    self._turtlebot_client.send_goal(goal_msg)
    rospy.loginfo("send goal")

  def FeedbackCallback(self, msg):
    goal = msg.feedback.goal
    goal_state = msg.feedback.goal_state
    task_id = msg.feedback.task_id
    feedback_msg = { "robot_id": self.robot_id, "task_id": task_id, "sub_goal": goal, "sub_state": goal_state, "state": 'active'}
    feedback_json = json.dumps(feedback_msg)
    # print(feedback_json)
    self.client.publish("iqr/usc/task/state", feedback_json)
    pass

  def ResultCallback(self, msg):

    task_id = msg.result.task_id
    status = msg.status.status
    if status != 5:
      self.robot_state = "idel"

    if status == 2:
      state = "canceled"
    elif status == 3:
      state = "succeed"
    elif status == 4:
      state = "aborted"
    feedback_msg = { "robot_id": self.robot_id, "task_id": task_id, "sub_goal": 'none', "sub_state": 'none', "state": state}
    feedback_json = json.dumps(feedback_msg)
    # print(feedback_json)
    self.client.publish("iqr/usc/task/state", feedback_json)
    pass

  def get_ip_address(self, ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.ip_address = socket.inet_ntoa(fcntl.ioctl(
      s.fileno(),
      0x8915,  # SIOCGIFADDR
      struct.pack('256s', ifname[:15])
    )[20:24])

  def get_mac_address(self): 
      mac=uuid.UUID(int = uuid.getnode()).hex[-12:] 
      self.mac_address = ":".join([mac[e:e+2] for e in range(0,11,2)])
      # return ":".join([mac[e:e+2] for e in range(0,11,2)])
      # print mac_address
      # return mac_address

def main():
  rospy.init_node('brain_control_interface')
  ML = MqttListener()
  # BCI.UDPLinstenLoop()
  t0 = threading.Thread(target=ML.UDPLinstenLoop,args=())
  t0.start()
  t1 = threading.Thread(target=ML.RobotStatePublisher,args=())
  t1.start()
  ML.get_mac_address()
  ML.get_ip_address('wlan0')
  rospy.spin()

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
