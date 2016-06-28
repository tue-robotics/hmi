#!/usr/bin/env python
import rospy
from hmi_msgs.msg import QueryAction, QueryGoal, Choice
from actionlib import SimpleActionClient, SimpleGoalState, GoalStatus
from hmi_server.abstract_server import queryToROS, resultFromROS
from dragonfly_speech_recognition.srv import GetSpeechResponse


class TimeoutException(Exception):
    pass


class Api(object):

    def __init__(self, name):
        '''
        Wrap the actionlib interface with the API
        '''
        self._client = SimpleActionClient(name, QueryAction)
        rospy.loginfo('waiting for "%s" server', name)
        self._client.wait_for_server()

    def _print_example(self, req_spec, req_choices):
        # Copy request
        example = "(%s)" % req_spec

        # Pick random group if available
        while re.search('\([^\)]+\)', example):
            options = re.findall('\([^\(\)]+\)', example)
            for option in options:
                example = example.replace(option, random.choice(option[1:-1].split("|")), 1)

        # Fetch all the residual choices
        choices = re.findall("<([^<>]+)>", example)

        # Parse the choices in the ending result :)
        for c in choices:
            for req_c in req_choices:
                if req_c == c:
                    value = random.choice(req_choices[req_c])
                    example = example.replace("<%s>"%c, value)

        rospy.loginfo("Example: \x1b[1;43m'{}'\x1b[0m".format(example))

    def _send_query(self, description, spec, choices):
        goal = queryToROS(description, spec, choices)
        state = self._client.send_goal(goal)

    def _wait_for_result_and_get(self, timeout=None):
        execute_timeout = rospy.Duration(timeout) if timeout else rospy.Duration(10)
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
            raise TimeoutException("Goal did not succeed, it was: %s" % GoalStatus.to_string(state))

        return self._client.get_result()

    def query(self, description, spec, choices):
        '''
        Perform a HMI query, returns a dict of {choicename: value}
        '''
        rospy.loginfo('Question: %s, spec: %s', description, spec)
        self._print_example(spec, choices)

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
        self._print_example(spec, choices)

        self._send_query(description, spec, {})
        answer = self._wait_for_result_and_get()

        rospy.logdebug('Answer: %s', answer)
        result = answer.raw_result
        rospy.loginfo('Result: %s', result)

        return result

    def old_query(self, spec, choices, timeout=10):
        '''
        Convert old queryies to a HMI query
        '''
        rospy.loginfo('spec: %s', spec)
        self._print_example(spec, choices)

        self._send_query('', spec, choices)
        try:
            answer = self._wait_for_result_and_get(timeout=timeout)
        except TimeoutException:
            return None

        rospy.logdebug('Answer: %s', answer)
        choices = resultFromROS(answer)

        result = GetSpeechResponse(result=answer.raw_result)
        result.choices = choices

        rospy.loginfo('Result: %s', result)

        return result

    def set_description(self, description):
        pass

    def set_grammar(self, spec):
        pass

    def wait_for_grammar_set(self, spec):
        pass
