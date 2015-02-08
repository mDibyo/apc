__author__ = 'dibyo'


import rospy


class ROSNode(object):
    def __init__(self, name, **kwargs):
        """

        :param name: the name of the node
        :param anonymous: whether the node is anonymous
        :return:
        """
        self.name = name
        self.anonymous = kwargs.get('anonymous', False)

        rospy.init_node(self.name, anonymous=self.anonymous)

    @staticmethod
    def spin():
        rospy.spin()


class TopicSubscriberNode(ROSNode):
    def __init__(self, name, topic, msg_type, **kwargs):
        """

        :param name: the name of the node
        :param topic: the topic to subscribe to
        :param msg_type: type of messages in the topic
        :param callback: function/method to be called on topic
        :param callback_args: addition args to be passed to callback
        :param anonymous: whether the node is anonymous
        """
        super(TopicSubscriberNode, self).__init__(name, kwargs)

        self.topic = topic
        self.msg_type = msg_type

        # Handle callbacks
        self.callback = self.callback_args = None
        callback = kwargs.get('callback', None)
        callback_args = kwargs.get('callback_args', None) \
            if callback is not None else None
        if callback is not None:
            self.add_callback(callback, callback_args)

    def add_callback(self, callback=None, callback_args=None):
        """

        :param callback: the function called when message is received
        :param callback_args: additional arguments to be passed to
            callback
        """
        self.callback = callback
        self.callback_args = callback_args
        if self.callback is None:
            def callback(data):
                rospy.loginfo(data)
            self.callback = callback

        self.subscriber = rospy.Subscriber(self.topic, self.msg_type,
                                           self.callback, self.callback_args)


class TopicPublisherNode(ROSNode):
    def __init__(self, name, topic, msg_type, **kwargs):
        """

        :param name: the name of the node
        :param topic: the topic to publish to
        :param msg_type: the type of messages in the topic 
        :param anonymous: whether the node is anonymous
        :param wait_on_connections: whether the node waits till there
            is at least one subscriber to topic
        """
        super(TopicPublisherNode, self).__init__(name, kwargs)

        self.topic = topic
        self.msg_type = msg_type
        self.callback = kwargs.get('callback', None)
