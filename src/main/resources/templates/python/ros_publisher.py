import rospy
from components.messages import *
from components.common.port import Port


class RosPublisher():
    def __init__(self, topic, type):
        rospy.init_node("", anonymous=True)
        self.pub = rospy.Publisher(topic, type)
        self.in1 = Port(0)

    def execute(self):
        self.pub.publish(self.in1.value)
