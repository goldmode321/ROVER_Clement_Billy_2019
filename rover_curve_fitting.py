import numpy as np
import scipy.interpolate as si
import scipy

# parameter
N = 3  # B Spline order
class General:
    def __init__(self, SharedVariables):
        self.SV = SharedVariables

    def calculate_yaw(self):
        temp_fitted_route_x = np.roll(self.SV.CF.fitted_route_x, -1)
        dx = temp_fitted_route_x - self.SV.CF.fitted_route_x
        dx[-1] = dx[-2]
        temp_fitted_route_y = np.roll(self.SV.CF.fitted_route_y, -1)
        dy = temp_fitted_route_y - self.SV.CF.fitted_route_y
        dy[-1] = dy[-2]
        self.SV.CF.fitted_route_yaw_rad = np.arctan2(dy, dx)%(2*np.pi)
        self.SV.CF.fitted_route_yaw_deg = np.degrees(self.SV.CF.fitted_route_yaw_rad)%360



class Bspline(General):
    def __init__(self, SharedVariable):
        self.SV = SharedVariable
        super().__init__(self.SV)

    def fitting(self):
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
        self.calculate_yaw()



class CubicSpline(General):
    def __init__(self, SharedVariable):
        self.SV = SharedVariable
        super().__init__(self.SV)

    def fitting(self):
        cubic_spline_x = scipy.interpolate.CubicSpline(
            np.linspace(1, len(self.SV.AS.route_x), len(self.SV.AS.route_x)),
            self.SV.AS.route_x
        )
        cubic_spline_y = scipy.interpolate.CubicSpline(
            np.linspace(1, len(self.SV.AS.route_y), len(self.SV.AS.route_y)),
            self.SV.AS.route_y
        )
        self.SV.CF.fitted_route_x = cubic_spline_x(np.arange(1, len(self.SV.AS.route_x), 0.1))
        self.SV.CF.fitted_route_y = cubic_spline_y(np.arange(1, len(self.SV.AS.route_y), 0.1))
        self.calculate_yaw()
