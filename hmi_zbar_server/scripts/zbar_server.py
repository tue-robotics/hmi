#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from hmi_server.abstract_server import AbstractHMIServer, HMIResult


class HMIServer(AbstractHMIServer):

    def _barcode_callback(self, msg):
        self._barcode = msg.data

    def _determine_answer(self, description, spec, choices, is_preempt_requested):
        self._barcode = None

        barcode_sub = rospy.Subscriber("qr_data_topic", String, self._barcode_callback, queue_size=1)
        rospy.loginfo("ZBAR HMI Server: subscribed to %s", barcode_sub.name)

        while not rospy.is_shutdown() and not is_preempt_requested():
            rospy.sleep(.1)
            if self._barcode:
                rospy.loginfo("ZBAR HMI Server: Received barcode: '%s'", self._barcode)
                break

        result = HMIResult(raw_result=self._barcode)
        self._barcode = None

        return result


if __name__ == '__main__':
    rospy.init_node('hmi_zbar_server')
    HMIServer(rospy.get_name())
    rospy.spin()
