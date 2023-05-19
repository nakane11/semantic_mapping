class ObjectDict(object):

    def __init__(self, di=None):
        self._objects = {}
        if di is not None: self.objects = di

    @property
    def objects(self):
        return self._objects

    @objects.setter
    def objects(self, di):
        self._objects = di

    def detect(self, label):
        if label in self._objects:
            self._objects[label] += 1
        else:
            self._objects[label] = 1
        
    def remove(self, label):
        if label in self._objects:
            del self._objects[label]

    def count(self, label, cnt=None):
        # update
        if cnt is not None:
            self._objects[label] = cnt

        if label in self._objects:
            return self._objects[label]
        else:
            return 0
