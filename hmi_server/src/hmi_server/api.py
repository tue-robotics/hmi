#!/usr/bin/env python
import rospy
from hmi_msgs.msg import QueryAction, QueryGoal, Choice
from actionlib import SimpleActionClient, SimpleGoalState, GoalStatus


def queryToROS(description, spec, choices):
	'''Convert a query call to a ROS message'''

	# convert the dict to an array of Choices
	choices = [Choice(id=choice,values=values) for (choice, values) in choices.items()]
	return QueryGoal(description, spec, choices)


def resultFromROS(answer):
	'''Convert a ROS result to Python result'''

	# convert the array of Choices back to a dict
	result = {}
	for choice in answer.results:
		if choice.id in result:
			rospy.logwarn('duplicate key "%s" in answer', choice.id)
		else:
			result[choice.id] = choice.value
	return result

class Api(object):

	def __init__(self, name):
		'''
		Wrap the actionlib interface with the API
		'''
		self._client = SimpleActionClient(name, QueryAction)
		rospy.loginfo('waiting for "%s" server', name)
		self._client.wait_for_server()

	def _send_goal_and_wait_and_get_result(self, goal):
		state = self._client.send_goal_and_wait(goal, execute_timeout=rospy.Duration(10), preempt_timeout=rospy.Duration(1))

		if state != GoalStatus.SUCCEEDED:
			raise RuntimeError("Goal did not succeed, it was: %s" % GoalStatus.to_string(state))

		return self._client.get_result()

	def query(self, description, spec, choices):
		'''
		Perform a HMI query, returns a dict of {choicename: value}
		'''
		rospy.loginfo('Question: %s, spec: %s', description, spec)
		goal = queryToROS(description, spec, choices)

		answer = self._send_goal_and_wait_and_get_result(goal)

		result = resultFromROS(answer)
		rospy.loginfo('Answer: %s', result)
		return result

	def query_raw(self, description, spec):
		'''
		Perform a HMI query without choices, returns a string
		'''
		rospy.loginfo('Question: %s, spec: %s', description, spec)

		goal = queryToROS(description, spec, None)
		answer = self._send_goal_and_wait_and_get_result(goal)
		return answer

	def set_description(self, description):
		pass

	def set_grammar(self, spec):
		pass

	def wait_for_grammar_set(self, spec):
		pass