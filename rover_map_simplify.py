import scipy
import numpy as np

class KDTree:
    """
    Nearest neighbor search class with KDTree
    """

    def __init__(self, SharedVariable):
        ''' data = np.vstack((ox, oy)).T'''
        # store kd-tree
        self.SV = SharedVariable
        data = np.vstack((self.SV.MAP.global_obstacle_x, self.SV.MAP.global_obstacle_y)).T
        self.tree = scipy.spatial.cKDTree(data, 32)
        self.sample_points()

    def search(self, inp, k=1):
        """
        Search NN

        inp: input data, single frame or multi frame

        """

        if len(inp.shape) >= 2:  # multi input
            index = []
            dist = []

            for i in inp.T:
                idist, iindex = self.tree.query(i, k=k)
                index.append(iindex)
                dist.append(idist)

            return index, dist

        dist, index = self.tree.query(inp, k=k)
        return index, dist

    def search_in_distance(self, inp, r):
        """
        find points with in a distance r
        """

        index = self.tree.query_ball_point(inp, r)
        return index

    def sample_points(self):
        oxy = np.vstack((self.SV.MAP.global_obstacle_x, self.SV.MAP.global_obstacle_y)).T

        # generate voronoi point
        vor = scipy.spatial.Voronoi(oxy)
        # self.SV.KDTree_sample_x = [ix for [ix, iy] in vor.vertices]
        # self.SV.KDTree_sample_y = [iy for [ix, iy] in vor.vertices]
        self.SV.KDTree_sample_x = vor.vertices[:, 0]
        self.SV.KDTree_sample_y = vor.vertices[:, 1]

        # self.SV.KDTree_sample_x.append(self.SV.MAP.start_x)
        # self.SV.KDTree_sample_y.append(self.SV.MAP.start_y)
        # self.SV.KDTree_sample_x.append(self.SV.MAP.end_x)
        # self.SV.KDTree_sample_y.append(self.SV.MAP.end_y)

        np.append(self.SV.KDTree_sample_x, self.SV.MAP.start_x)
        np.append(self.SV.KDTree_sample_y, self.SV.MAP.start_y)
        np.append(self.SV.KDTree_sample_x, self.SV.MAP.end_x)
        np.append(self.SV.KDTree_sample_y, self.SV.MAP.end_y)