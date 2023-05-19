import numpy as np
from object_dict import *

class SemanticGridMapUtils(object):

    def __init__(self, resolution, origin, width, height, depth, map_data=None):
        self._resolution = resolution
        self._origin = np.array(origin)
        self._width = width
        self._height = height
        self._depth = depth
        self._roi = None
        if map_data is not None:
            self.load_map(map_data)
        else:
            self.init_map()

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
    def data(self):
        """Return data of 3D grid map.

        Each cel represents the .

        Returns
        -------
        self._map_data : numpy.ndarray
            Occupancy probabilities are in the range [0,100].
            Unknown is -1.
        """
        return self._map_data

    @property
    def roi(self):
        return self._roi

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
        mx = int((wx - self._origin[0]) / self._resolution)
        my = int((wy - self._origin[1]) / self._resolution)
        mz = int((wz - self._origin[2]) / self._resolution)
        if not abs(mx) < self._width/2 or not abs(my) < self._height or not 0 <= mz < self._depth:
            return (-1, -1, -1)
        return (int(mx+self._width/2), int(my+self._height/2), mz)

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

    def init_map(self):
        self._map_data = np.empty((self._depth, self._height, self._width), dtype=object)

    def load_map(self, raw_data):
        return

    def set_value(self, label, x, y, z, value):
        if self._map_data[z][y][x] == None:
            self._map_data[z][y][x] = ObjectDict()
        self._map_data[z][y][x].count(label, value)

    def get_value(self, label, x, y, z):
        if self._map_data[z][y][x] == None:
            return 0
        else:
            return self._map_data[z][y][x].count(label)

    def add_object(self, label, roi=None, x=None, y=None, z=None):
        if roi is not None:
            for x in range(roi[0][0], roi[0][1]):
                for y in range(roi[1][0], roi[1][1]):
                    for z in range(roi[2][0], roi[2][1]):
                        if self._map_data[z][y][x] == None:
                            self._map_data[z][y][x] = ObjectDict()
                        self._map_data[z][y][x].detect(label)
        else:
            if self._map_data[z][y][x] == None:
                self._map_data[z][y][x] = ObjectDict()
            self._map_data[z][y][x].detect(label)

    def get_data(self, x, y, z):
        if self._map_data[z][y][x] == None:
            return {}
        return self._map_data[z][y][x].objects

    def get_value_map(self, label, roi=None):
        result = np.zeros((self._depth, self._height, self._width))
        if not roi:
            for x in range(self._width):
                for y in range(self._height):
                    for z in range(self._depth):
                        if self._map_data[z][y][x] is not None:
                            result[z][y][x] = self._map_data[z][y][x].count(label)
        else:
            for x in range(roi[0][0], roi[0][1]):
                for y in range(roi[1][0], roi[1][1]):
                    for z in range(roi[2][0], roi[2][1]):
                        if self._map_data[z][y][x] is not None:
                            result[z][y][x] = self._map_data[z][y][x].count(label)
        return result
