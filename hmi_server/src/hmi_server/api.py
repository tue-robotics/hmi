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

    def _send_query(self, description, spec, choices):
        goal = queryToROS(description, spec, choices)
        state = self._client.send_goal(goal)

    def _wait_for_result_and_get(self):
        execute_timeout = rospy.Duration(10)
        preempt_timeout = rospy.Duration(1)

        if not self._client.wait_for_result(execute_timeout):
            # preempt action
            rospy.logdebug("Canceling goal")
            self._client.cancel_goal()
            if self._client.wait_for_result(preempt_timeout):
                rospy.loginfo("Preempt finished within specified preempt_timeout [%.2f]", preempt_timeout.to_sec());
            else:
                rospy.loginfo("Preempt didn't finish specified preempt_timeout [%.2f]", preempt_timeout.to_sec());
        
        state = self._client.get_state()
        if state != GoalStatus.SUCCEEDED:
            raise RuntimeError("Goal did not succeed, it was: %s" % GoalStatus.to_string(state))

        return self._client.get_result()

    def query(self, description, spec, choices):
        '''
        Perform a HMI query, returns a dict of {choicename: value}
        '''
        rospy.loginfo('Question: %s, spec: %s', description, spec)

        self._send_query(description, spec, choices)
        answer = self._wait_for_result_and_get()

        rospy.logdebug('Answer: %s', answer)
        result = resultFromROS(answer)
        rospy.loginfo('Result: %s', result)

        return result

    def query_raw(self, description, spec):
        '''
        Perform a HMI query without choices, returns a string
        '''
        rospy.loginfo('Question: %s, spec: %s', description, spec)

        self._send_query(description, spec, {})
        answer = self._wait_for_result_and_get()

        rospy.logdebug('Answer: %s', answer)
        result = answer.raw_result
        rospy.loginfo('Result: %s', result)

        return result

    def set_description(self, description):
        pass

    def set_grammar(self, spec):
        pass

    def wait_for_grammar_set(self, spec):
        pass