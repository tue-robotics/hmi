#!/usr/bin/env python

import rospy
from hmi_msgs.msg import HumanInteractionAction, HumanInteractionResult, Result
from actionlib import SimpleActionServer
from abc import ABCMeta, abstractmethod

class AbstractHMIServer(object):
    """
    Abstract base class for a hmi client

    >>> class HMIServer(AbstractHMIServer):
    ...     def __init__(self):
    ...         pass
    ...     def _determine_answer(self, question, spec, choices):
    ...         return HumanInteractionResult()
    ...     def _set_succeeded(self, result):
    ...         print result

    >>> server = HMIServer()

    >>> from hmi_msgs.msg import HumanInteractionGoal
    >>> goal = HumanInteractionGoal(question='q', spec='spec', choices=[])

    >>> server._execute_cb(goal)
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
        self.server = SimpleActionServer(name, HumanInteractionAction,
                                         execute_cb=self._execute_cb, auto_start=False)
        self.server.start()
        rospy.loginfo('%s started', name)

    def _execute_cb(self, goal):

        rospy.loginfo('I got a question: %s', goal.question)
        rospy.loginfo('This is the spec: %s, %s', goal.spec, repr(goal.choices))
        result = self._determine_answer(question=goal.question,
                                       spec=goal.spec,
                                       choices=goal.choices)

        if result:
            self._set_succeeded(result=result)
        else:
            self.server.set_aborted(text="Cancelled by user")

    def _set_succeeded(self, result):
        self.server.set_succeeded(result)

    def _set_succeeded(self, result):
        self.server.set_succeeded(result)

    @abstractmethod
    def _determine_answer(self, question, spec, choices):
        pass
        

if __name__ == "__main__":
    import doctest
    doctest.testmod()