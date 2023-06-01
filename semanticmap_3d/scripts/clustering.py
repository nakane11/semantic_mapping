# def kmeans(k, X, max_iter=300):
#     X_size,n_features = X.shape    
#     centroids  = X[np.random.choice(X_size,k)]
#     new_centroids = np.zeros((k, n_features))
#     cluster = np.zeros(X_size)

#     for epoch in range(max_iter):
#         for i in range(X_size):
#             distances = np.sum((centroids - X[i]) ** 2, axis=1)
#             cluster[i] = np.argsort(distances)[0]
#         for j in range(k):
#             new_centroids[j] = X[cluster==j].mean(axis=0)
#         if np.sum(new_centroids == centroids) == k:
#             break
#         centroids =  new_centroids
#     return cluster

from pyclustering.cluster.xmeans import xmeans
from pyclustering.cluster.center_initializer import kmeans_plusplus_initializer

def x_means(X):
    xm_c = kmeans_plusplus_initializer(X, 1).initialize()
    xm_i = xmeans(data=X, initial_centers=xm_c, kmax=20, ccore=True)
    xm_i.process()
    print(xm_i._xmeans__centers)
    return xm_i._xmeans__centers
