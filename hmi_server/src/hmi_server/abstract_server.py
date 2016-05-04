#!/usr/bin/env python

import rospy
from hmi_msgs.msg import QueryAction, QueryResult, Result, QueryGoal, Choice
from actionlib import SimpleActionServer
from abc import ABCMeta, abstractmethod


def queryToROS(description, spec, choices):
    '''Convert a query call to a ROS message'''

    # convert the dict to an array of Choices
    choices = [Choice(id=choice,values=values) for (choice, values) in choices.items()]
    return QueryGoal(description, spec, choices)


# TODO: this belongs in HMIResult
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


class HMIResult(object):

    def __init__(self, raw_result='', results=None):
        '''
        raw_result is a optional string
        results is an optional dict
        '''
        self.results = results if results else {}
        self.raw_result = raw_result

    def to_ros(self):
        results = [Result(id=choice,value=value) for (choice, value) in self.results.items()]
        return QueryResult(results=results, raw_result=self.raw_result)

    def from_ros(self):
        raise NotImplementedError()

class AbstractHMIServer(object):
    """
    Abstract base class for a hmi client

    >>> class HMIServer(AbstractHMIServer):
    ...     def __init__(self):
    ...         pass
    ...     def _determine_answer(self, description, spec, choices):
    ...         return QueryResult()
    ...     def _set_succeeded(self, result):
    ...         print result

    >>> server = HMIServer()

    >>> from hmi_msgs.msg import QueryGoal
    >>> goal = QueryGoal(description='q', spec='spec', choices=[])

    >>> server._execute_cb(goal)
    raw_result: ''
    results: []

    >>> class HMIServer(AbstractHMIServer):
    ...     def __init__(self):
    ...         pass


    >>> server = HMIServer()
    Traceback (most recent call last):
    ...
    TypeError: Can't instantiate abstract class HMIServer with abstract methods _determine_answer

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
        rospy.loginfo('This is the spec: %s, %s', goal.spec, repr(choices))

        try:
            result = self._determine_answer(description=goal.description,
                                           spec=goal.spec,
                                           choices=choices,
                                           is_preempt_requested=self._server.is_preempt_requested)
        except Exception as e:
            rospy.logwarn('_determine_answer raised an exception: %s', e)
            self._server.set_aborted()
        else:
            # we've got a result or a cancel
            if result:
                self._set_succeeded(result=result.to_ros())
                rospy.loginfo('result: %s', result)
            else:
                rospy.loginfo('cancelled')
                self._server.set_aborted(text="Cancelled by user")

    def _set_succeeded(self, result):
        self._server.set_succeeded(result)

    def _set_succeeded(self, result):
        self._server.set_succeeded(result)

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