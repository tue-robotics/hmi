#!/usr/bin/env python

import traceback
from abc import ABCMeta, abstractmethod

import rospy
from actionlib import SimpleActionServer
from hmi_msgs.msg import QueryAction, QueryResult, QueryGoal, QueryActionFeedback
from .common import trim_string


class AbstractHMIServer(object):
    """
    Abstract base class for a hmi servers

    """
    __metaclass__ = ABCMeta

    def __init__(self, name):
        self._action_name = name
        self._server = SimpleActionServer(name, QueryAction,
                                         execute_cb=self._execute_cb, auto_start=False)
        self._server.start()
        rospy.loginfo('HMI server started on "%s"', name)

    def _execute_cb(self, goal):

        # TODO: refactor this somewhere
        choices = {}
        for choice in goal.choices:
            if choice.id in choices:
                rospy.logwarn('duplicate key "%s" in answer', choice.id)
            else:
                choices[choice.id] = choice.values

        rospy.loginfo('I got a question: %s', goal.description)
        rospy.loginfo('This is the spec: %s, %s', trim_string(goal.spec), repr(choices))

        try:
            result = self._determine_answer(description=goal.description,
                                           spec=goal.spec,
                                           choices=choices,
                                           is_preempt_requested=self._server.is_preempt_requested)
        except Exception as e:
            # rospy.logwarn('_determine_answer raised an exception: %s', e)
            # import pdb; pdb.set_trace()
            tb = traceback.format_exc()
            rospy.logerr('_determine_answer raised an exception: %s' % tb)

            self._server.set_aborted()
        else:
            # we've got a result or a cancel
            if result:
                self._set_succeeded(result=result.to_ros(self._action_name))
                rospy.loginfo('result: %s', result)
            else:
                rospy.loginfo('cancelled')
                self._server.set_aborted(text="Cancelled by user")

    def _set_succeeded(self, result):
        self._server.set_succeeded(result)

    def _publish_feedback(self):
        self._server.publish_feedback(QueryActionFeedback())

    @abstractmethod
    def _determine_answer(self, description, spec, choices, is_preempt_requested):
        '''
        Overwrite this method to provide custom implementations

        Return the answer
        Return None if nothing is heared
        Raise an Exception if an error occured
        '''
        pass


if __name__ == "__main__":
    import doctest
    doctest.testmod()
