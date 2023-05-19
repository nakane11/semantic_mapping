#!/usr/bin/env python3

import rospy
import message_filters
import PyKDL
import tf2_geometry_msgs
import tf2_ros
from jsk_topic_tools import ConnectionBasedTransport
from jsk_recognition_msgs.msg import ClassificationResult, BoundingBoxArray

from object_dict import *
from semantic_map import *

class SemanticMappingNode(ConnectionBasedTransport):

    def __init__(self):
        super(self.__class__, self).__init__()
        self.base_frame_id = rospy.get_param("~base_frame_id", "map")
        self._duration_timeout = rospy.get_param("~timeout", 6.0)
        self._tf_buffer = tf2_ros.Buffer(rospy.Duration(10))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self.resolution = rospy.get_param("~resolution", 0.025)

        origin = self.get_robotpose()
        print(origin)
        if not origin: exit(1)
        self.map = SemanticGridMapUtils(resolution=0.025, origin=origin, width=400, height=200, depth=200)
        # 50*50*10 m
        self._pub = self.advertise("~output", BoundingBoxArray, queue_size=1)

    def get_robotpose(self):
        try:
            trans = self._tf_buffer.lookup_transform('map', 'base_footprint', rospy.Time(0), timeout=rospy.Duration(10.0))
        except Exception as e:
            rospy.logerr(e)
            return False
        return [trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z]
    
    def subscribe(self):
        sub_cls = message_filters.Subscriber('/fg_node/output/class', ClassificationResult, queue_size=1)
        sub_boxes = message_filters.Subscriber('/multi_euclidean_cluster_point_indices_decomposer/boxes', BoundingBoxArray, queue_size=1)
        self.subs =[sub_cls, sub_boxes]
        if rospy.get_param('~approximate_sync', True):
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                fs=self.subs, queue_size=100, slop=slop)
        else:
            sync = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=100)
        sync.registerCallback(self.add_from_msg)

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

    def add_from_msg(self, cls_msg, bbox_msg):
        try:
            pykdl_transform_base_to_camera = tf2_geometry_msgs.transform_to_kdl(
                self._tf_buffer.lookup_transform(
                    self.base_frame_id,
                    cls_msg.header.frame_id,
                    cls_msg.header.stamp,
                    timeout=rospy.Duration(self._duration_timeout)))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn('{}'.format(e))
            return
        for label, box in zip(cls_msg.label_names, bbox_msg.boxes):
            roi = [[box.pose.position.x - box.dimensions.x/2, box.pose.position.x + box.dimensions.x/2],
                   [box.pose.position.y - box.dimensions.y/2, box.pose.position.y + box.dimensions.y/2],
                   [box.pose.position.z - box.dimensions.z/2, box.pose.position.z + box.dimensions.z/2]]
            for x_idx in range(int(box.dimensions.x/self.resolution)):
                x = box.pose.position.x - box.dimensions.x/2 + self.resolution * x_idx
                for y_idx in range(int(box.dimensions.y/self.resolution)):
                    y = box.pose.position.y - box.dimensions.y/2 + self.resolution * y_idx
                    for z_idx in range(int(box.dimensions.z/self.resolution)):
                        z = box.pose.position.z - box.dimensions.z/2 + self.resolution * z_idx
                        x, y, z = pykdl_transform_base_to_camera * PyKDL.Vector(x, y, z)
                        mx, my, mz = self.map.world_to_map(x, y, z)
                         self.map.add_object(label, x=mx, y=my, z=mz)
        print(np.where(self.map.get_value_map(label)))

 if __name__ == '__main__':
    rospy.init_node('semantic_mapping_node')
    SemanticMappingNode()
    rospy.spin()
