import numpy as np
import scipy.interpolate as si

# parameter
N = 3  # B Spline order

class Bspline:
    def __init__(self, SharedVariable):
        self.SV = SharedVariable



    def bspline_planning(self):
        '''self.SV.sample_number = sampling number'''
        t = range(len(self.SV.AS.route_x))
        x_tup = si.splrep(t, self.SV.AS.route_x, k=N)
        y_tup = si.splrep(t, self.SV.AS.route_y, k=N)

        x_list = list(x_tup)
        # xl = self.SV.route_x.tolist() # Uncomment if using array
        xl = self.SV.AS.route_x # Comment if using array
        x_list[1] = xl

        y_list = list(y_tup)
        # yl = self.SV.route_y.tolist() # Uncomment if using array
        yl = self.SV.AS.route_y # Comment if using array
        y_list[1] = yl

        ipl_t = np.linspace(0.0, len(self.SV.AS.route_x) - 1, self.SV.CF.sample_number)
        rx = si.splev(ipl_t, x_list)
        ry = si.splev(ipl_t, y_list)

        self.SV.CF.fitted_route_x = np.asarray(rx)
        self.SV.CF.fitted_route_y = np.asarray(ry)

class Bspline_sim:
    def __init__(self, SharedVariable):
        self.SV = SharedVariable



    def bspline_planning(self):
        '''self.SV.sample_number = sampling number'''
        t = range(len(self.SV.route_x))
        x_tup = si.splrep(t, self.SV.route_x, k=N)
        y_tup = si.splrep(t, self.SV.route_y, k=N)

        x_list = list(x_tup)
        # xl = self.SV.route_x.tolist() # Uncomment if using array
        xl = self.SV.route_x # Comment if using array
        x_list[1] = xl

        y_list = list(y_tup)
        # yl = self.SV.route_y.tolist() # Uncomment if using array
        yl = self.SV.route_y # Comment if using array
        y_list[1] = yl

        ipl_t = np.linspace(0.0, len(self.SV.route_x) - 1, self.SV.sample_number)
        rx = si.splev(ipl_t, x_list)
        ry = si.splev(ipl_t, y_list)

        self.SV.fitted_route_x = np.asarray(rx)
        self.SV.fitted_route_y = np.asarray(ry)
