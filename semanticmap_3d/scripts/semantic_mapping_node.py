#!/usr/bin/env python3

import rospkg
import rospy
import message_filters
import PyKDL
import tf2_geometry_msgs
import tf2_ros
from jsk_topic_tools import ConnectionBasedTransport
from eos import make_fancy_output_dir
from pathlib import Path
import open3d as o3d

from jsk_recognition_msgs.msg import ClassificationResult, BoundingBoxArray
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from std_srvs.srv import Empty, EmptyResponse

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
        rospy.Service('~save_map', Empty, self.save_map)
        rospy.on_shutdown(self.save_map)

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
        multiple_points = np.zeros((0,3))
        colors =np.zeros((0,3))
        markers = []
        id = 0

        for i,v in enumerate(data.values()):
            points, idx = self.extract_pt(v)
            if points.shape[0] <= 0:
                continue
            multiple_points = np.vstack((multiple_points, points))

            h_arr = 127 + labelcolormap()[i]/2
            color_norm = v / np.max(v) * 255
            c = color_norm[idx].reshape([-1,1])
            c_arr = np.dot(c, h_arr.reshape((1,-1)))
            colors = np.vstack((colors, c_arr))

            centers, clusters = x_means(points)
            for p, q in zip(centers, clusters):
                print(len(q))
                m = make_sphere(header, p, h_arr, id)
                id += 1
                markers.append(m)
            print(list(data.keys())[i], len(centers))

        pub_msg = create_cloud_xyzrgb(header, multiple_points, colors)
        self._pub.publish(pub_msg)

        pub_marker_array = MarkerArray(markers=markers)
        self._pub_center.publish(pub_marker_array)

    def extract_pt(self, arr):
        idx = np.where(arr)
        wx, wy, wz = map(lambda x: x.reshape([-1,1]),
                         self.map.map_to_world(idx[0],idx[1],idx[2]))
        points = np.hstack((wx, wy, wz)).astype(np.float32)
        return points, idx

    def save_map(self, req=None):
        rospack = rospkg.RosPack()
        outdir = Path(make_fancy_output_dir(rospack.get_path('semanticmap_3d'), no_save=True)).resolve()
        data = self.map.get_map()
        for k,v in zip(list(data.keys()), data.values()):
            outpath = str(outdir / (k + '.pcd'))
            points, idx = self.extract_pt(v)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            o3d.io.write_point_cloud(outpath, pcd)
        return EmptyResponse()

if __name__ == '__main__':
    rospy.init_node('semantic_mapping_node')
    SemanticMappingNode()
    rospy.spin()
