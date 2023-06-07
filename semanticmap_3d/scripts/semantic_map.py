import numpy as np
from collections import defaultdict

class SemanticGridMapUtils(object):

    def __init__(self, resolution, origin, width, height, depth, map_data=None):
        self._resolution = resolution
        self._origin = np.array(origin)
        self._width = width
        self._height = height
        self._depth = depth
        self._label2voxel = defaultdict(lambda: np.zeros(
            (self._width, self._height, self._depth), dtype=np.int64))
        # self._roi = None
        # if map_data is not None:
        #     self.load_map(map_data)
        # else:
        #     self.init_map()

    @property
    def resolution(self):
        """Return resolution of this map.

        Returns
        -------
        self._resolution : float
            resoulution of this map.
            size of a cell.
        """
        return self._resolution

    @property
    def origin(self):
        """Return origin vector.

        Returns
        -------
        self._origin : numpy.ndarray
            [x, y, z]. Unit is meter.
        """
        return self._origin

    @property
    def height(self):
        """Return height

        Returns
        -------
        self._height : int
            height of map.
        """
        return self._height

    @property
    def width(self):
        """Return width

        Returns
        -------
        self._width : int
            width of map.
        """
        return self._width

    @property
    def depth(self):
        """Return depth

        Returns
        -------
        self._depth : int
            depth of map.
        """
        return self._depth

    # def data(self, label=None):
    #     """Return data of 3D grid map.

    #     Each cel represents the .

    #     Returns
    #     -------
    #     self._map_data : numpy.ndarray
    #         Occupancy probabilities are in the range [0,100].
    #         Unknown is -1.
    #     """
    #     if label is not None:
    #         return self._label2voxel
    #     else:
    #         return self._label2voxel.get(label)

    # @property
    # def roi(self):
    #     return self._roi

    @staticmethod
    def from_rostopic(topic_name, depth):
        import nav_msgs.msg

        msg = rospy.wait_for_message(topic_name,
                                     nav_msgs.msg.OccupancyGrid)

        resolution = msg.info.resolution
        height = msg.info.height
        width = msg.info.width
        origin = (msg.info.origin.position.x,
                  msg.info.origin.position.y,
                  msg.info.origin.position.z)
        return SemanticGridMapUtils(
            resolution, origin, width, height, depth)

    def world_to_map(self, wx, wy, wz):
        mx = (wx - self._origin[0]) / self._resolution
        my = (wy - self._origin[1]) / self._resolution
        mz = (wz - self._origin[2]) / self._resolution
        if not abs(mx) < self._width/2 or not abs(my) < self._height or not 0 <= mz < self._depth:
            return (-1, -1, -1)
        return (int(mx+self._width/2), int(my+self._height/2), int(mz))

    def map_to_world(self, mx, my, mz):
        wx = self._origin[0] + (mx-self._width/2) * self._resolution
        wy = self._origin[1] + (my-self._height/2) * self._resolution
        wz = self._origin[2] + mz * self._resolution
        return (wx, wy, wz)

    # def batch_world_to_map(self, world_coords):
    #     """Batch of world_to_map.

    #     Parameters
    #     ----------
    #     world_coords : numpy.ndarray
    #         shape of (batch_size, 2)

    #     Returns
    #     -------
    #     map_coords : numpy.ndarray
    #         shape of (batch_size, 2)
    #     """
    #     map_coords = (world_coords - self._origin[:2]) / self._resolution
    #     return map_coords

    # def batch_map_to_world(self, map_coords):
    #     """Batch of map_to_world.

    #     Parameters
    #     ----------
    #     map_coords : numpy.ndarray
    #         shape of (batch_size, 2)

    #     Returns
    #     -------
    #     world_coords : numpy.ndarray
    #         shape of (batch_size, 2)
    #     """
    #     world_coords = self._origin[:2] + map_coords * self._resolution
    #     return world_coords

    # def extract_region(self):
    #     x, y = np.where(self.data > 0)
    #     min_y = y.min()
    #     min_x = x.min()
    #     max_y = y.max() + 1
    #     max_x = x.max() + 1
    #     self._roi = (min_y, min_x, max_y, max_x)

    # def init_map(self):
    #     self._map_data = np.empty((self._depth, self._height, self._width), dtype=object)

    # def load_map(self, raw_data):
    #     return

    def set_value(self, label, x, y, z, val=1, reset=False):
        if reset:
            self._label2voxel[label][x][y][z] = val
        else:
            self._label2voxel[label][x][y][z] += val

    def get_value(self, label, x, y, z):
        return self._label2voxel[label][x][y][z]

    def add_object_by_roi(self, label, roi, cnt=1):
        self._label2voxel[label][roi[0][0]:roi[0][1],
                                 roi[1][0]:roi[1][1],
                                 roi[2][0]:roi[2][1]] += 1

    def add_object(self, label, center, dim, cnt=1):
        roi = [[center[0]-dim, center[0]+dim],
               [center[1]-dim, center[1]+dim],
               [center[2]-dim, center[2]+dim]]
        self._label2voxel[label][roi[0][0]:roi[0][1],
                                 roi[1][0]:roi[1][1],
                                 roi[2][0]:roi[2][1]] += 1

    def get_map(self, label=None, roi=None):
        if label is None:
            return self._label2voxel
        if roi is None:
            return self._label2voxel[label]

        return self._label2voxel[label][roi[0][0]:roi[0][1],
                                        roi[1][0]:roi[1][1],
                                        roi[2][0]:roi[2][1]]
    def add_array(self, label, arr):
        self._label2voxel[label] = arr
