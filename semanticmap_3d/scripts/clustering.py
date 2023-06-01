from pyclustering.cluster.xmeans import xmeans
from pyclustering.cluster.center_initializer import kmeans_plusplus_initializer

def x_means(X):
    xm_c = kmeans_plusplus_initializer(X, 1).initialize()
    xm_i = xmeans(data=X, initial_centers=xm_c, kmax=20, ccore=True)
    xm_i.process()
    print(xm_i._xmeans__centers)
    return xm_i.get_centers(), xm_i.get_clusters()
