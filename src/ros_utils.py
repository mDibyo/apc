#!/usr/bin/env python

import rospy


__author__ = 'dibyo'


class ROSNode(object):
    def __init__(self, name, **kwargs):
        """

        :param name: the name of the node
        :param anonymous: whether the node is anonymous
        :param init: whether the node should be initialized
        """
        self.name = name
        self.anonymous = kwargs.get('anonymous', False)
        self.disable_signals = kwargs.get('disable_signals', False)

        if kwargs.get('init', True):
            rospy.init_node(self.name, anonymous=self.anonymous,
                            disable_signals=self.disable_signals)

    @staticmethod
    def spin():
        """
        Prevent the node from exiting
        """
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
        super(TopicSubscriberNode, self).__init__(name, **kwargs)

        self.topic = topic
        self.msg_type = msg_type
        self.last_msg = None
        self.msg_filter = lambda msg: True

        # Handle callback
        self.callback = self.callback_args = self.subscriber = None
        callback = kwargs.get('callback', None)
        if callback is not None:
            callback_args = kwargs.get('callback_args', None)
            self.add_callback(callback, callback_args)

    def add_callback(self, callback=None, callback_args=None, loginfo=False):
        """

        :param callback: the function to be called when there is a
            new message in the topic
        :param callback_args: addition args that must be passed to
            the callback
        :param loginfo: whether messages received should be logged
        """
        self.callback_args = callback_args
        self.callback = callback

        if callback is None:
            def callback(data):
                if loginfo:
                    rospy.loginfo(data)
            self.callback = callback
            self.callback_args = None

        self.subscriber = rospy.Subscriber(self.topic, self.msg_type,
                                           self._wrap_callback(callback),
                                           self.callback_args)

    def set_msg_filter(self, msg_filter):
        self.msg_filter = msg_filter

    def _wrap_callback(self, callback):
        def callback_wrapper(data, *args):
            if self.msg_filter(data):
                self.last_msg = data
            return callback(data, *args)

        return callback_wrapper


class TopicPublisherNode(ROSNode):
    def __init__(self, name, topic, msg_type, **kwargs):
        """

        :param name: the name of the node
        :param topic: the topic to publish to
        :param msg_type: the type of messages in the topic
        :param anonymous: whether the node is anonymous
        :param wait_for_subscriber: whether the node should wait till
            there is a subscriber on the topic
        """
        super(TopicPublisherNode, self).__init__(name, kwargs)

        self.topic = topic
        self.msg_type = msg_type
        self.callback = kwargs.get('callback', None)

        self.publisher = rospy.Subscriber(self.topic, self.msg_type, self.callback)

    def publish(self, msg):
        self.publisher.publish(msg)


def topic(topic_string):
    return None if not topic_string else topic_string

