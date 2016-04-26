#!/usr/bin/env python
import rospy
from hmi_msgs.msg import QueryAction, QueryGoal
from actionlib import SimpleActionClient

class Api(object):

	def __init__(self, name):
		'''
		Wrap the actionlib interface with the API
		'''
		self._client = SimpleActionClient(name, QueryAction)
		rospy.loginfo('waiting for "%s" server', name)
		self._client.wait_for_server()

	def query(self, description, spec, choices):
		'''
		Perform a HMI query, returns a dict of {choicename: value}
		'''
		pass

	def query_raw(self, description, spec):
		'''
		Perform a HMI query without choices, returns a string
		'''
		rospy.loginfo('Question: %s, spec: %s', description, spec)

		goal = QueryGoal(description, spec, None)
		self._client.send_goal_and_wait(goal, execute_timeout=rospy.Duration(10), preempt_timeout=rospy.Duration(1))
		print self._client.get_result()

	def set_description(self, description):
		pass

	def set_grammar(self, spec):
		pass

	def wait_for_grammar_set(self, spec):
		pass