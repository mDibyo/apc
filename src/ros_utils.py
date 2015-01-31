__author__ = 'dibyo'


import rospy


class ROSNode(object):
    def __init__(self, name, anonymous=False):
        self.name = name
        self.anonymous = anonymous

        rospy.init_node(self.name, anonymous=self.anonymous)


class TopicSubscriberNode(ROSNode):
    def __init__(self, name, topic, msg_type, callback=None, anonymous=False):
        super(TopicSubscriberNode, self).__init__(name, anonymous)

        self.topic = topic
        self.msg_type = msg_type
        self.callback = callback

        if self.callback is None:
            self.callback = lambda data: rospy.loginfo(data)

    def subscribe(self):
        rospy.Subscriber(self.topic, self.msg_type, self.callback)
        rospy.spin()
