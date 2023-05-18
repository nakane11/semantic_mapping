#!/usr/bin/env python3

import rospy
import message_filters
from jsk_topic_tools import ConnectionBasedTransport
from jsk_recognition_msgs.msg import ClassificationResult, ClusterPointIndices
from sensor_msgs.msg import PointCloud2

from object_dict import *
from semantic_map import *

class SemanticMappingNode(ConnectionBasedTransport):

    def __init__(self):
        super(self.__class__, self).__init__()
        origin = self.get_robotpose()
        if not origin: exit(1)
        self.map = SemanticGridMapUtils(resolution=0.05, origin=origin, width=1000, height=1000, depth=200)
        # 50*50*10 m

    def get_robotpose(self):
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time(0), timeout=rospy.Duration(10.0))
        except Exception as e:
            rospy.logerr(e)
            return False
        return [trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z]
    
    def subscribe(self):
        sub_cls = message_filters.Subscriber('~input/class', ClassificationResult, queue_size=1)
        sub_indices = message_filters.Subscriber('~input/cluster_indices', ClusterPointIndices, queue_size=1, buff_size=2**24)
        sub_cloud = message_filters.Subscriber('~input/cloud', PointCloud2, queue_size=1, buff_size=2**24)
        self.subs =[sub_cls, sub_indices, sub_cloud]
        if rospy.get_param('~approximate_sync', True):
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                fs=self.subs, queue_size=queue_size, slop=slop)
        else:
            sync = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=queue_size)
        sync.registerCallback(self.add_from_msg)

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

    def add_from_msg(self, cls_msg, indices_msg, cloud_msg):
        for label in cls_msg.label_names):
            obj = Object(obj_type=label)
            points = []
            for index in indices_msg.indices:
                # wx, wy, wz = transform()
                points.append([wx, wy, wz])
                obj.points = points
                for p in points:
                    mx, my, mz = world_to_map(p.x, p.y, p.z)
                    self.map.add_object(mx, my, mz, obj)

if __name__ == '__main__':
    rospy.init_node('semantic_mapping_node')
    SemanticMappingNode()
    rospy.spin()
