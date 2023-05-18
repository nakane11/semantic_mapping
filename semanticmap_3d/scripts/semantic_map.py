import numpy as np

class SemanticGridMapUtils(object):

    def __init__(self, map_data=None, resolution, origin, width, height, depth):
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

    # @staticmethod
    # def from_rostopic(topic_name):
    #     import nav_msgs.msg

    #     msg = one_shot_subscribe(topic_name,
    #                              nav_msgs.msg.OccupancyGrid,
    #                              queue_size=1)

    #     resolution = msg.info.resolution
    #     height = msg.info.height
    #     width = msg.info.width
    #     origin = (msg.info.origin.position.x,
    #               msg.info.origin.position.y,
    #               msg.info.origin.position.z)
    #     map_data = msg.data
    #     return OccupancyGridMapUtils(
    #         map_data, resolution, origin, width, height)

    # def world_to_map(self, wx, wy):
    #     mx = int((wx - self._origin[0]) / self._resolution)
    #     my = int((wy - self._origin[1]) / self._resolution)
    #     if not 0 <= mx < self._width or not 0 <= my < self._height:
    #         return (-1, -1)
    #     return (mx, my)

    # def map_to_world(self, mx, my):
    #     wx = self._origin[0] + mx * self._resolution
    #     wy = self._origin[1] + my * self._resolution
    #     return (wx, wy)

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
        # self._map_data =
        return

    def add_object(self, x, y, z, obj, prob=1.0):
        if self._map_data[z][y][x] == None:
            self._map_data[z][y][x] = ObjectDict()
        self._map_data[z][y][x].add(obj, prob=1.0)

    def get_object_dict(self, x, y, z):
        return self._map_data[z][y][x]

    # def add_object_from_(self, ):
    #     obj = Object()
