from sensor_msgs.msg import PointCloud2,PointField
import sensor_msgs.point_cloud2 as pcd2
import numpy as np

def create_cloud_xyzrgb(header, points, colors):

    fields = [  PointField("x",0,PointField.FLOAT32,1),
                PointField("y",4,PointField.FLOAT32,1),
                PointField("z",8,PointField.FLOAT32,1),
                PointField("r",12,PointField.FLOAT32,1),
                PointField("g",16,PointField.FLOAT32,1),
                PointField("b",20,PointField.FLOAT32,1)]    

    data = np.hstack((points, colors)).astype(np.float32)
    return pcd2.create_cloud(header, fields, data)
    
    
