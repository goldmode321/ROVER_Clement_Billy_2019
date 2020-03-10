import numpy as np
import pyqtgraph
from PyQt5 import QtGui, QtCore, QtWidgets
import  traceback
import os, sys
import multiprocessing
import threading
from queue import Queue
import scipy.spatial

import gui.simulator as sim_ui
import rover_shared_variable
import rover_car_model
import rover_pathplanning
import rover_map
import rover_curve_fitting
import rover_pathtracking

class SharedVariable:
    def __init__(self):
        self.lidar_data = np.array([])
        self.lidar_angle = np.array([])
        self.lidar_distance = np.array([])
        self.local_obstacle_x = np.array([])
        self.local_obstacle_y = np.array([])

        self.rover_x = 0
        self.rover_y = 0

        self.step_unit = 20 # cm
        self.rover_size = 30
        self.obstacle_size = 1
        self.lidar_scan_radius = 600

        self.shared_lidar_data = Queue()
        self.shared_map = Queue() # Map
        self.shared_config = Queue() # time delay, rover_x, rover_y, run_flag, lidar_scan_radius


class SimulatedLidar:
    def __init__(self, SharedVariable):
        self.SV = SharedVariable
        self.lidar_process = multiprocessing.Process(
            target=self.scanning,
            args=[self.SV.shared_lidar_data, self.SV.MAP, self.SV.shared_config],
            daemon=True
            )

    def scanning(self, shared_lidar_data, MAP, shared_config):
        global_obstacle_x = MAP.global_obstacle_x
        global_obstacle_y = MAP.global_obstacle_y
        run_flag = True
        while run_flag:
            try:
                local_obstacle_x = local_obstacle_y = lidar_angle = lidar_distance = np.array([])
                time_delay, rover_x, rover_y, run_flag, lidar_scan_radius = shared_config.get() # Get initial rover state
                distance_x, distance_y = global_obstacle_x - rover_x, global_obstacle_y - rover_y
                total_distance = np.hypot(distance_x, distance_y)
                too_large_index = np.where(total_distance > lidar_scan_radius)

                lidar_obstacle_x, lidar_obstacle_y = \
                    np.delete(global_obstacle_x, too_large_index), np.delete(global_obstacle_y, too_large_index)
                distance_x, distance_y = \
                    np.delete(distance_x, too_large_index), np.delete(distance_y, too_large_index)
                total_distance = np.delete(total_distance, too_large_index)
                lidar_obstacle_angle = np.arctan2(distance_x, distance_y)*180/np.pi
                for angle in range(360):
                    index_obstacle_of_angle = np.where(lidar_obstacle_angle > angle - 0.5 & lidar_obstacle_angle < angle + 0.5)
                    if len(index_obstacle_of_angle[0]) != 0:
                        index_obstacle_of_angle_closest = np.argmin(np.take(total_distance, index_obstacle_of_angle))
                        local_obstacle_x = np.append(local_obstacle_x, lidar_obstacle_x[index_obstacle_of_angle_closest])
                        local_obstacle_y = np.append(local_obstacle_y, lidar_obstacle_y[index_obstacle_of_angle_closest])
                        lidar_angle = np.append(lidar_angle, lidar_obstacle_angle[index_obstacle_of_angle_closest])
                        lidar_distance = np.append(lidar_distance, total_distance[index_obstacle_of_angle_closest])
                        if not shared_lidar_data.empty():
                            shared_lidar_data.get()
                            shared_lidar_data.put(lidar_angle, lidar_distance, local_obstacle_x, local_obstacle_y)
                        else:
                            shared_lidar_data.put(lidar_angle, lidar_distance, local_obstacle_x, local_obstacle_y)
            except:
                pass


class PathTracking:
    def __init__(self, SharedVariable):
        self.SV = SharedVariable
        




class Pens:
    def __init__(self):
        self.global_obstacle_pen = pyqtgraph.mkPen(color='k', width=10)
        self.local_obstacle_pen = pyqtgraph.mkPen(color=(0, 0 , 255, 90), width=10)
        self.astar_path_pen = pyqtgraph.mkPen(color=(100, 100, 100, 100), width=2)
        self.track_path_pen = pyqtgraph.mkPen(color=(255, 0, 0, 100), width=2)
        self.rover_pen = pyqtgraph.mkPen(color=(0, 255, 0, 100), width=10)
        self.transparent = pyqtgraph.mkPen(color=(255, 255, 255, 0))
        self.route_pen = pyqtgraph.mkPen(color=(255, 0, 0), width=2, style=QtCore.Qt.DotLine)
        self.fitted_route_pen = pyqtgraph.mkPen(color=(0, 0, 255), width=3)


class App:
    def __init__(self):
        os.environ["QT_AUTO_SCREEN_SCALE_FACTOR"] = "1"
        app = QtWidgets.QApplication(sys.argv)
        app.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
        self.MainWindow = QtWidgets.QMainWindow()

        self.gui = sim_ui.Ui_MainWindow()
        self.gui.setupUi(self.MainWindow)



        self.SV = rover_shared_variable.SharedVariables()
        self.map = rover_map.Map_sim(self.SV)
        self.SV.GUI.gui = self.gui
        self.Pen = Pens()
        map_name_list = [
            'maze1',
            'maze2',
        ]

        self.rover_size = 50
        self.px_mode = True
        self.run_flag = False
        self.test_tracking_flag = False


        for name in map_name_list: self.gui.combo_map.addItem(name)
        print(self.gui.combo_map.currentText())


        self.gui.spin_start_x.setValue(self.SV.AS.start_x)
        self.gui.spin_start_y.setValue(self.SV.AS.start_y)
        self.gui.spin_end_x.setValue(self.SV.AS.end_x)
        self.gui.spin_end_y.setValue(self.SV.AS.end_y)
        self.gui.spin_rover_radius.setValue(self.SV.AS.rover_size)
        self.gui.spin_safe_radius.setValue(self.SV.AS.obstacle_size)
        self.gui.spin_unitstep.setValue(self.SV.AS.step_unit)
        self.gui.spin_time_delay.setValue(self.SV.GUI.show_progress_delay*1000)

        # self.lidar = SimulatedLidar(self.SV)
        self.SV.GUI.show_progress = False
        self.CF = rover_curve_fitting.Bspline(self.SV)
        self.astar = rover_pathplanning.AstarPathPlanning_sim_v2(self.SV)
        self.path_tracking = rover_pathtracking.StanleyController(self.SV)

        self.map_plot_widget = pyqtgraph.PlotWidget(background='w')
        self.SV.GUI.route_plot = self.map_plot_widget.plot(pen=self.Pen.route_pen, symbol='s')
        self.SV.GUI.fitted_route_plot = self.map_plot_widget.plot(pen=self.Pen.fitted_route_pen)

        self.global_obstacle_plot = pyqtgraph.ScatterPlotItem(
            symbol='o',
            # size=self.SV.AS.obstacle_size,
            size = 20,
            brush=(0, 0, 0),
            pen=self.Pen.transparent,
        )
        self.local_obstacle_plot = pyqtgraph.ScatterPlotItem(
            symbol='o',
            # size=self.SV.AS.obstacle_size,
            size = 20,
            brush=(0, 89, 255),
            pen=self.Pen.transparent,
        )
        self.end_point_plot = pyqtgraph.ScatterPlotItem(
            symbol='x',
            size=self.SV.AS.rover_size*2,
            brush=(255, 0, 0)
        )
        self.rover_plot = pyqtgraph.ScatterPlotItem(
            symbol='o',
            # size=self.rover_size,
            size=20,
            brush=(0, 255 ,0)
        )
        self.SV.GUI.forward_plot = pyqtgraph.ScatterPlotItem(
            symbol='o',
            # size=self.SV.AS.obstacle_size,
            size = 5,
            brush=(0, 200, 0),
            pen=self.Pen.transparent,
        )
        self.SV.GUI.backward_plot = pyqtgraph.ScatterPlotItem(
            symbol='o',
            # size=self.SV.AS.obstacle_size,
            size = 5,
            brush=(200, 0, 0),
            pen=self.Pen.transparent,
        )




        self.map_plot_widget.addItem(self.global_obstacle_plot)
        self.map_plot_widget.addItem(self.local_obstacle_plot)
        self.map_plot_widget.addItem(self.end_point_plot)
        self.map_plot_widget.addItem(self.rover_plot)
        self.map_plot_widget.addItem(self.SV.GUI.forward_plot)
        self.map_plot_widget.addItem(self.SV.GUI.backward_plot)
        self.gui.horizontalLayout_3.addWidget(self.map_plot_widget)



        self.gui.spin_start_x.valueChanged.connect(self.change_start_end_yaw_point)
        self.gui.spin_start_y.valueChanged.connect(self.change_start_end_yaw_point)
        self.gui.spin_start_yaw.valueChanged.connect(self.change_start_end_yaw_point)
        self.gui.spin_end_x.valueChanged.connect(self.change_start_end_yaw_point)
        self.gui.spin_end_y.valueChanged.connect(self.change_start_end_yaw_point)
        self.gui.spin_rover_radius.valueChanged.connect(self.change_rover_size)
        self.gui.spin_safe_radius.valueChanged.connect(self.change_obstacle_size)
        self.gui.spin_time_delay.valueChanged.connect(self.change_show_progress_time_delay)
        self.gui.check_pxmode.clicked.connect(lambda: self.change_pxmode())
        self.gui.button_start.clicked.connect(self.button_start_clicked)
        self.gui.button_pause.clicked.connect(self.button_pause_clicked)
        self.gui.button_reset.clicked.connect(self.button_reset_clicked)
        self.gui.dspin_gcost.valueChanged.connect(self.change_g_h_cost)
        self.gui.dspin_hcost.valueChanged.connect(self.change_g_h_cost)
        self.gui.check_progress.toggled.connect(self.change_show_progress)
        self.gui.spin_unitstep.valueChanged.connect(self.change_step_unit)
        self.gui.combo_map.currentIndexChanged.connect(self.change_map)
        self.gui.radio_bspline.toggled.connect(self.change_curve_fitting)
        self.gui.buttonGroup.buttonClicked.connect(self.change_path_planning_mode)
        self.gui.groupBox_test_tracking.toggled.connect(self.change_test_tracking)


        self.mouse_view_box = self.map_plot_widget.plotItem.vb
        self.signal_proxy = pyqtgraph.SignalProxy(self.map_plot_widget.scene().sigMouseMoved, rateLimit=30, slot=self.mouse_move)


        self.plot_map()


        self.MainWindow.show()
        sys.exit(app.exec_())

    def change_map(self):
        self.map.map_name = self.gui.combo_map.currentText()
        self.map.generate_map()
        self.plot_map()
        self.gui.spin_start_x.blockSignals(True)
        self.gui.spin_start_y.blockSignals(True)
        self.gui.spin_end_x.blockSignals(True)
        self.gui.spin_end_y.blockSignals(True)
        self.gui.spin_start_x.setValue(self.SV.AS.start_x)
        self.gui.spin_start_y.setValue(self.SV.AS.start_y)
        self.gui.spin_end_x.setValue(self.SV.AS.end_x)
        self.gui.spin_end_y.setValue(self.SV.AS.end_y)
        self.gui.spin_start_x.blockSignals(False)
        self.gui.spin_start_y.blockSignals(False)
        self.gui.spin_end_x.blockSignals(False)
        self.gui.spin_end_y.blockSignals(False)

    def plot_map(self):
        self.global_obstacle_plot.setData(self.SV.GOBS.global_obstacle_x, self.SV.GOBS.global_obstacle_y)
        self.rover_plot.setData([self.SV.AS.start_x], [self.SV.AS.start_y])
        self.end_point_plot.setData([self.SV.AS.end_x], [self.SV.AS.end_y])

    def plot_route(self):
        self.SV.GUI.route_plot.setData(self.SV.AS.route_x, self.SV.AS.route_y)

    def plot_test_tracking(self):
        last_steering_angle = self.SV.PT.steering_angle
        self.SV.PT.current_x, self.SV.PT.current_y = self.SV.GUI.mouse_x, self.SV.GUI.mouse_y
        self.arrow_steering_angle.setPos(self.SV.PT.current_x, self.SV.PT.current_y)
        self.path_tracking.calculate_command()
        self.gui.lcd_steering_agnle.display(self.SV.PT.steering_angle)
        # self.arrow_steering_angle.setStyle(angle=self.SV.PT.steering_angle)
        self.arrow_steering_angle.rotate(self.SV.PT.steering_angle - last_steering_angle)
        self.gui.lcd_target_index.display(self.SV.PT.target_index)
        self.path_tracking_target_pos_plot.setData([self.SV.PT.target_position[0]], [self.SV.PT.target_position[1]])
        self.test_tracking_distance_plot.setData(self.SV.PT.distance)
        self.test_tracking_distance_x_plot.setData(self.SV.PT.distance_x)
        self.test_tracking_distance_y_plot.setData(self.SV.PT.distance_y)
        self.test_tracking_distance_strange_plot.setData(self.SV.PT.distance_strange)
        # print(self.arrow_steering_angle.opts['angle'])

    def change_test_tracking(self):
        if not self.test_tracking_flag:
            if len(self.SV.CF.fitted_route_x) > 1:
                self.arrow_steering_angle = pyqtgraph.ArrowItem(angle=self.SV.PT.steering_angle, tipAngle=30, headLen=20, tailLen=40, tailWidth=5)
                self.path_tracking_target_pos_plot = pyqtgraph.ScatterPlotItem(
                    symbol='x',
                    # size=self.rover_size,
                    size=30,
                    brush=(0, 255 ,225)
                )
                self.map_plot_widget.addItem(self.arrow_steering_angle)
                self.map_plot_widget.addItem(self.path_tracking_target_pos_plot)

                self.test_tracking_win = pyqtgraph.GraphicsWindow()
                self.test_tracking_widget = self.test_tracking_win.addPlot()
                self.test_tracking_widget.addLegend()
                # self.test_tracking_win.addItem(self.test_tracking_widget)
                self.test_tracking_distance_plot = self.test_tracking_widget.plot(
                    pen=pyqtgraph.mkPen(color=(255, 0, 0), width=5),
                    name='distance',
                )
                self.test_tracking_distance_x_plot = self.test_tracking_widget.plot(
                    pen=pyqtgraph.mkPen(color=(0, 255, 0), width=2),
                    name='distance X',
                )
                self.test_tracking_distance_y_plot = self.test_tracking_widget.plot(
                    pen=pyqtgraph.mkPen(color=(0, 220, 255), width=2),
                    name='distance Y'
                )
                self.test_tracking_distance_strange_plot = self.test_tracking_widget.plot(
                    pen=pyqtgraph.mkPen(color=(255, 255, 0), width=5),
                    name='distance Strange'
                )


                self.test_tracking_timer = QtCore.QTimer()
                self.test_tracking_timer.timeout.connect(self.plot_test_tracking)
                self.test_tracking_timer.start(50)
                self.test_tracking_flag = True
                print("Test tracking")
            else:
                self.gui.groupBox_test_tracking.setChecked(False)
                print("No route to plan")
        else:
            self.test_tracking_timer.stop()
            self.map_plot_widget.removeItem(self.arrow_steering_angle)
            self.map_plot_widget.removeItem(self.path_tracking_target_pos_plot)
            self.test_tracking_flag = False

    def change_start_end_yaw_point(self):
        if np.min(np.hypot(self.SV.GOBS.global_obstacle_x - self.gui.spin_start_x.value(),\
            self.SV.GOBS.global_obstacle_y - self.gui.spin_start_y.value())) <= self.SV.AS.obstacle_size + self.SV.AS.rover_size or\
                np.min(np.hypot(self.SV.GOBS.global_obstacle_x - self.gui.spin_end_x.value(),\
                    self.SV.GOBS.global_obstacle_y - self.gui.spin_start_y.value())) <= self.SV.AS.obstacle_size + self.SV.AS.rover_size:
            print('ROVER or End Point hit obstacle')
        else:
            self.SV.AS.start_x = self.SV.MAP.rover_x = self.gui.spin_start_x.value()
            self.SV.AS.start_y = self.SV.MAP.rover_y = self.gui.spin_start_y.value()
            self.SV.AS.attitude[0] = self.SV.PT.current_yaw = self.gui.spin_start_yaw.value()
            self.SV.AS.end_x = self.gui.spin_end_x.value()
            self.SV.AS.end_y = self.gui.spin_end_y.value()
            self.rover_plot.setData([self.SV.AS.start_x], [self.SV.AS.start_y])
            self.end_point_plot.setData([self.SV.AS.end_x], [self.SV.AS.end_y])

    def change_show_progress_time_delay(self):
        self.SV.GUI.show_progress_delay = self.gui.spin_time_delay.value()/1000

    def change_g_h_cost(self):
        self.SV.AS.G_cost_factor = self.gui.dspin_gcost.value()
        self.SV.AS.H_cost_factor = self.gui.dspin_hcost.value()

    def change_step_unit(self):
        self.SV.AS.step_unit = self.gui.spin_unitstep.value()

    def change_show_progress(self):
        self.SV.GUI.show_progress = False if self.SV.GUI.show_progress else True
        print("Show progress {}".format(self.SV.GUI.show_progress))
        if self.SV.GUI.show_progress:
            self.show_progress_timer = QtCore.QTimer()
            self.show_progress_timer.timeout.connect(self.plot_route)
            self.show_progress_timer.start(self.gui.spin_time_delay.value())
        else:
            self.show_progress_timer.stop()

    def change_rover_size(self):
        self.SV.AS.rover_size = self.gui.spin_rover_radius.value()

    def change_curve_fitting(self):
        if self.gui.radio_bspline.isChecked():
            self.CF = rover_curve_fitting.Bspline(self.SV)
            print("Change curve fitting method to Bspline")
        elif self.gui.radio_cubic_spline.isChecked():
            self.CF = rover_curve_fitting.Cubic_Spline(self.SV)
            print("Change curve fitting method to cubic spline")

    def change_obstacle_size(self):
        self.SV.AS.obstacle_size = self.gui.spin_safe_radius.value()
        self.global_obstacle_plot.setSize(self.SV.AS.obstacle_size)
        self.local_obstacle_plot.setSize(self.SV.AS.obstacle_size)

    def change_pxmode(self):
        if self.px_mode:
            self.global_obstacle_plot.setPxMode(False)
            self.local_obstacle_plot.setPxMode(False)
            self.rover_plot.setPxMode(False)
            self.end_point_plot.setPxMode(False)
            self.gui.check_pxmode.setChecked(False)
        else:
            self.global_obstacle_plot.setPxMode(True)
            self.local_obstacle_plot.setPxMode(True)
            self.rover_plot.setPxMode(True)
            self.end_point_plot.setPxMode(True)
            self.gui.check_pxmode.setChecked(True)
        self.px_mode = not self.px_mode
        print('check {}'.format(self.px_mode))

    def change_path_planning_mode(self, mode=0):
        if self.gui.radio_original_a.isChecked():
            self.astar = rover_pathplanning.AstarPathPlanning_sim(self.SV)
            print("Method change to Astar_Original")
        elif self.gui.radio_a_v2.isChecked():
            self.astar = rover_pathplanning.AstarPathPlanning_sim_v2(self.SV)
            print("Method change to Astar_V2")
        elif self.gui.radio_a_v3.isChecked():
            self.astar = rover_pathplanning.AstarPathPlanning_sim_v3(self.SV)
            print("Method change to Astar_V3")


    # def update_shared_config(self):
    #     if not self.SV.shared_config.empty(): self.SV.shared_config.get()    
    #     self.SV.shared_config.put([
    #         self.gui.spin_time_delay.value(),
    #         self.SV.rover_x,
    #         self.SV.rover_y,
    #         self.run_flag,
    #         self.SV.lidar_scan_radius
    #     ])

    def button_start_clicked(self):
        astar_thread = Astar_thread(self.SV, self.astar, self.CF)
        astar_thread.start()


    def button_pause_clicked(self):
        pass

    def button_reset_clicked(self):

        self.gui.spin_start_x.setValue(self.SV.AS.start_x)
        self.gui.spin_start_y.setValue(self.SV.AS.start_y)
        self.gui.spin_end_x.setValue(self.SV.AS.end_x)
        self.gui.spin_end_y.setValue(self.SV.AS.end_y)
        self.change_start_end_yaw_point()


    def mouse_move(self, event):
        pos = event[0] # using signal proxy turns original arguments into a tuple
        if self.map_plot_widget.sceneBoundingRect().contains(pos):
            mousePoint = self.mouse_view_box.mapSceneToView(pos)
            self.SV.GUI.mouse_x, self.SV.GUI.mouse_y = int(mousePoint.x()), int(mousePoint.y())
            self.gui.lcd_mouse_x.display(self.SV.GUI.mouse_x)
            self.gui.lcd_mouse_y.display(self.SV.GUI.mouse_y)
    

    def mouse_press(self, event):
        print('pressed {}'.format(np.random.random_integers(0, 10, 1)))

    # def mouseReleseEvent(self, event):
    #     pass

class Astar_thread(threading.Thread):
    def __init__(self, SharedVariables, astar, CF):
        super().__init__(daemon=True)
        self.SV = SharedVariables
        self.CF = CF
        self.astar = astar


    def run(self):
        self.astar.planning()
        self.SV.GUI.route_plot.setData(self.SV.AS.route_x, self.SV.AS.route_y)
        self.SV.GUI.gui.lcd_astar_planning_time.display(self.SV.AS.astar_planning_time)
        self.CF.fitting()
        self.SV.GUI.fitted_route_plot.setData(self.SV.CF.fitted_route_x, self.SV.CF.fitted_route_y)
        self.plot_forwrd_backward()

    def plot_forwrd_backward(self):
        forward_list_x = list()
        forward_list_y = list()
        backward_list_x = list() 
        backward_list_y = list()
        for i, j in enumerate(self.SV.AS.forward_backward_record):
            if j == 'forward':
                forward_list_x.append(self.SV.AS.route_x[i])
                forward_list_y.append(self.SV.AS.route_y[i])
            else:
                backward_list_x.append(self.SV.AS.route_x[i])
                backward_list_y.append(self.SV.AS.route_y[i])
        self.SV.GUI.forward_plot.setData(forward_list_x, forward_list_y)
        self.SV.GUI.backward_plot.setData(backward_list_x, backward_list_y)


if __name__ == "__main__":
    App()