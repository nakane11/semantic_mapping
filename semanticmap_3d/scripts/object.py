class ObjectDict(object):

    def __init__(self, obj_arr=[], prob_arr=[]):
        self._object_dict = {}
        if len(obj_arr) == len(prob_arr):
            for (obj, prob) in zip(obj_arr, prob_arr):
                self._object_dict[obj] = prob
                
    @property
    def object_dict(self):
        return self._object_dict

    @object_dict.setter
    def object_dict(self, d):
        self._object_dict = d

    def add(self, obj, prob=1.0):
        self._object_dict[obj] = prob

    def remove(self, obj):
        if obj in self._object_dict:
            del self._object_dict[obj]

    def remove_by_name(self, name):
        for obj in self._object_dict.keys():
            if obj.name == name:
                self.remove(obj)

    def get_by_name(self, name):
        return list(filter(lambda x: x.name==name, self._object_dict.keys()))
    
    def get_by_type(self, obj_type):
        return list(filter(lambda x: x.obj_type==obj_type, self._object_dict.keys()))


class Object(object):
    def __init__(self, name=None, obj_type=None, points=None):
        self._name = name
        self._obj_type = obj_type
        self._points = points

    @property
    def name(self):
        return self._name

    @property
    def obj_type(self):
        return self._obj_type
        
    @property
    def points(self):
        return self._points

    @points.setter
    def points(self, pts):
        self._points = pts    
