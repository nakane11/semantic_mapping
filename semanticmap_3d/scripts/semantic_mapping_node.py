#!/usr/bin/env python3

import rospy
import message_filters
import PyKDL
import tf2_geometry_msgs
import tf2_ros
from jsk_topic_tools import ConnectionBasedTransport
from jsk_recognition_msgs.msg import ClassificationResult, BoundingBoxArray
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray

from jsk_recognition_utils.color import labelcolormap
from object_dict import *
from semantic_map import *
from visualize import *
from clustering import x_means

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
        self.map = SemanticGridMapUtils(resolution=self.resolution, origin=origin, width=400, height=400, depth=200)
        self._pub = self.advertise("~output/cloud", PointCloud2, queue_size=1)
        self._pub_center = self.advertise("~output/center", MarkerArray, queue_size=1)

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
            x,y,z = pykdl_transform_base_to_camera * PyKDL.Vector(box.pose.position.x,
                                                                  box.pose.position.y,
                                                                  box.pose.position.z)
            center = self.map.world_to_map(x, y, z)
            dimension = int(min(box.dimensions.x,
                                box.dimensions.y,
                                box.dimensions.z)/self.map.resolution/2)
            if dimension == 0: continue
            self.map.add_object(label, center, dimension)

        header = cls_msg.header
        header.frame_id = "map"
        self.publish_pc(header)

    def publish_pc(self, header):
        data = self.map.get_map()
        points = np.zeros((0,3))
        colors =np.zeros((0,3))
        markers = []
        id = 0

        for i,v in enumerate(data.values()):
            h_arr = 127 + labelcolormap()[i]/2
            idx = np.where(v)
            wx, wy, wz = map(lambda x: x.reshape([-1,1]),
                             self.map.map_to_world(idx[0],idx[1],idx[2]))
            point = np.hstack((wx, wy, wz)).astype(np.float32)
            if point.shape[0] <= 0:
                continue
            centers, clusters = x_means(point)
            for p, q in zip(centers, clusters):
                print(len(q))
                m = make_sphere(header, p, h_arr, id)
                id += 1
                markers.append(m)

            print(list(data.keys())[i], len(centers))
            points = np.vstack((points, point))
            color_norm = v / np.max(v) * 255
            c = color_norm[idx].reshape([-1,1])
            c_arr = np.dot(c, h_arr.reshape((1,-1)))
            colors = np.vstack((colors, c_arr))

        pub_msg = create_cloud_xyzrgb(header, points, colors)
        self._pub.publish(pub_msg)

        pub_marker_array = MarkerArray(markers=markers)
        self._pub_center.publish(pub_marker_array)

if __name__ == '__main__':
    rospy.init_node('semantic_mapping_node')
    SemanticMappingNode()
    rospy.spin()
