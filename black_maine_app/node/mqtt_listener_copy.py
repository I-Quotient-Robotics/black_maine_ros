#!/usr/bin/env python
import time
import math
import rospy
import socket
import threading
import actionlib

import json
# from turtlebot_app.msg import DistributionAction
import turtlebot_app.msg
import paho.mqtt.client as mqtt

class MqttListener:

  def __init__(self):

    self.task_finished = False
    self.feedback_msg = []
    self.client = mqtt.Client()
    self.client.on_connect = self.on_connect
    self.client.on_message = self.on_message
    self.client.connect("192.168.0.110", 1883, 60)

    # self._sub = rospy.Subscriber("aruco_marker_publisher/markers_list", UInt32MultiArray, self.arucoDetectCallback)
    self._sub_feedback = rospy.Subscriber("turtlebot_distribution_server/feedback", turtlebot_app.msg.DistributionActionFeedback, self.FeedbackCallback)

    self._sub_result = rospy.Subscriber("turtlebot_distribution_server/result", turtlebot_app.msg.DistributionActionResult, self.ResultCallback)

    # self._pub = rospy.Publisher('brain_control_msg', brain_control_msg, queue_size=1)
    # self._grasp_client = rospy.ServiceProxy('vision_grasp', vision_grasp)
    self._turtlebot_client = actionlib.SimpleActionClient("turtlebot_distribution_server", turtlebot_app.msg.DistributionAction)

  def UDPLinstenLoop(self):
    # rospy.loginfo("start udp linster..")
    self.client.loop_start()

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
    print(msg.payload)
    # msg_dist = "{"message_type":"state","id":"btn_01","state":"ready","ip":"192.168.0.184","mac_address":"98:F4:AB:23:0F:BC"}"
    msg_dist = eval(msg.payload)
    robot_id = msg_dist["robot_id"]
    task_id = msg_dist["task_id"]
    goal_list = msg_dist["goal_list"]
    # goal_str = "[{'goal': 'qq'}, {'goal': 'aa'}]"
    # goal_list = eval(goal_str)
    goal_list_goal = []
    # print(type(goal_list))
    for x in xrange(0, len(goal_list)-1):
      goal_dist = goal_list[x]
      goal_list_goal.append(goal_dist["goal"])
      print(goal_dist["goal"])
    if robot_id == "truck_01":
      self.SendGoal(robot_id, task_id, goal_list_goal)
    # print(goal_list)

  # The callback for when a PUBLISH message is received from the server.
  def on_message(self, client, userdata, msg):
      # rospy.loginfo("mqtt receive msg ")
      # print(msg.topic+" "+str(msg.payload))
      pass

  def SendGoal(self, robot_id, task_id, goal_list_goal):
    action_name = "/turtlebot_distribution_server"
    goal_msg = turtlebot_app.msg.DistributionGoal()
    goal_msg.task_id = task_id
    goal_msg.places = goal_list_goal
    print(type(goal_msg.places[0]))
    self._turtlebot_client.send_goal(goal_msg)
    rospy.loginfo("sned goal")

  # def FeedbackCallback(self, msg):
  #   feedback_msg = "{ 'robot_id': " + "'truck_01'," + "'task_id':" + "'task_id'," + "'goal_list': [ {'goal': 'standby', 'need_confirm': 'true'}, {'goal': 't2', 'need_confirm': 'true'} ] }" 
  #   self.client.publish("iqr/usc/agv_manager/state", feedback_msg)
  #   print("truck1 callback")
  #   pass

  def FeedbackCallback(self, msg):
    goal = msg.feedback.goal
    goal_state = msg.feedback.goal_state
    task_id = msg.feedback.task_id
    # step_index = msg.feedback.step_index
    # step_description = msg.feedback.step_description
    feedback_msg = { "robot_id": 'truck_01', "task_id": task_id, "sub_goal": goal, "sub_state": goal_state, "state": 'active'}
    feedback_json = json.dumps(feedback_msg)
    # feedback_msg = "{ 'robot_id': 'truck_01', 'task_id':" + task_id + ", 'sub_goal':" + goal + ", 'sub_state':" + goal_state + ", 'state': 'active'}"
    print(feedback_json)
    self.client.publish("iqr/usc/task/state", feedback_json)
    print("truck feedback callback")
    pass

  def ResultCallback(self, msg):

    task_id = msg.result.task_id
    status = msg.status.status
    if status == 2:
      state = "canceled"
    elif status == 3:
      state = "succeed"
    elif status == 4:
      state = "aborted"
    # feedback_msg = "{ 'robot_id': 'truck_01', 'task_id':" + task_id + ",'sub_goal: 'none', 'sub_state': 'none', 'task_status': '" + state +  "'}"
    feedback_msg = { "robot_id": 'truck_01', "task_id": task_id, "sub_goal": 'none', "sub_state": 'none', "state": state}
    feedback_json = json.dumps(feedback_msg)
    print(feedback_json)
    self.client.publish("iqr/usc/task/state", feedback_json)
    print("truck result callback")
    pass

  # def ResultCallback(self, msg):
  #   # self.task_finished = True
  #   task_id = msg.result.task_id
  #   status = msg.status.status
  #   feedback_msg_dist = eval(self.feedback_msg)
  #   feedback_msg_dist["task_status"] = str(status)
  #   # self.feedback_msg = "{ 'robot_id': 'truck_01', 'task_id':" + task_id + ", 'state':" + str(2) + ", 'detail': finished, 'goal_status':" + goal_state + "}"
  #   self.client.publish("iqr/usc/task/state", str(feedback_msg_dist))
  #   print("truck result callback")
  #   # self].task_finished = False
  #   pass

def main():
  rospy.init_node('brain_control_interface')
  ML = MqttListener()
  # BCI.UDPLinstenLoop()
  t0 = threading.Thread(target=ML.UDPLinstenLoop,args=())
  t0.start()
  rospy.spin()

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass