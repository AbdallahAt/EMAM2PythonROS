import rospy

from components.messages import *
from components.common.port import Port

class RosSubscriber:
    def __init__(self, topic, type):
        rospy.init_node("", anonymous=True)
        rospy.Subscriber(topic, type, self.setOutput)
        self.out1 = Port(0)

    def setOutput(self, msg):
        self.out1.value = msg

    def execute():
        pass