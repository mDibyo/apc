__author__ = 'dibyo'


from ros_utils import TopicSubscriberNode

if __name__ == "__main__":
    node = TopicSubscriberNode("apc_object_listener",
                               "apc/update",
                               )