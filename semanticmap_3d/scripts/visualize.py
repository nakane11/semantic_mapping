from sensor_msgs.msg import PointCloud2,PointField
import sensor_msgs.point_cloud2 as pcd2
import numpy as np
from visualization_msgs.msg import Marker

def create_cloud_xyzrgb(header, points, colors):

    fields = [  PointField("x",0,PointField.FLOAT32,1),
                PointField("y",4,PointField.FLOAT32,1),
                PointField("z",8,PointField.FLOAT32,1),
                PointField("r",12,PointField.FLOAT32,1),
                PointField("g",16,PointField.FLOAT32,1),
                PointField("b",20,PointField.FLOAT32,1)]    

    data = np.hstack((points, colors)).astype(np.float32)
    return pcd2.create_cloud(header, fields, data)

def create_cloud_xyz(header, points):
    fields = [  PointField("x",0,PointField.FLOAT32,1),
                PointField("y",4,PointField.FLOAT32,1),
                PointField("z",8,PointField.FLOAT32,1)]
    return pcd2.create_cloud(header, fields, points)

def str2int(text):
    s = 0
    for i in text:
        s += ord(i)
    return s

def make_sphere(header, pose, color, id):
    m = Marker(header=header)
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.color.r = color[0]/255.0
    m.color.g = color[1]/255.0
    m.color.b = color[2]/255.0
    m.color.a = 1.0
    m.pose.position.x = pose[0]
    m.pose.position.y = pose[1]
    m.pose.position.z = pose[2]
    m.pose.orientation.w = 1.0
    m.scale.x = 0.05
    m.scale.y = 0.05
    m.scale.z = 0.05
    m.id = id
    return m
    
    
