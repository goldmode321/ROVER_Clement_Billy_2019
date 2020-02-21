import numpy as np
import pyqtgraph
from PyQt5 import QtGui, QtCore, QtWidgets
import  traceback
import os, sys
import multiprocessing
from queue import Queue


import gui.simulator as sim_ui
import rover_car_model


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
        self.rover_size = 20
        self.obstacle_size = 20
        self.lidar_scan_radius = 600

        self.shared_lidar_data = Queue()
        self.shared_map = Queue() # Map
        self.shared_config = Queue() # time delay, rover_x, rover_y, run_flag, lidar_scan_radius

        self.MAP = Map()
        self.gui = None

        self.G_cost_factor = 1
        self.H_cost_factor = 1
        self.show_progress = False
        self.route_x = list()
        self.route_y = list()
        self.route_plot = None


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


class Map:
    def __init__(self, name='maze1'):
        self.map_name = name
        self.map_width = 2000
        self.map_length = 2000
        self.start_x = 0
        self.start_y = 0
        self.global_obstacle_x = np.array([])
        self.global_obstacle_y = np.array([])
        self.global_obstacle = np.array([])
        self.obs_dict = {'maze1':self.maze1, 'maze2':self.maze2}
        self.generate_map()


    def generate_map(self):
        self.obs_dict[self.map_name]()

    def maze1(self):
        self.map_width = int(self.map_width)
        self.map_length = int(self.map_length)
        wall_bottom_x = wall_top_x = np.linspace(0, self.map_length, self.map_length, dtype='int')
        wall_top_y = np.zeros(self.map_length, dtype='int')
        wall_bottom_y = wall_top_y + self.map_width

        wall_right_x = np.zeros(self.map_width, dtype='int') + self.map_length
        wall_right_y = np.linspace(0, self.map_width, self.map_width, dtype='int')

        wall_left_x = np.zeros(self.map_width, dtype='int')
        wall_left_y = np.linspace(0, self.map_width, self.map_width, dtype='int')

        wall1_x = np.zeros(int(self.map_width*2/5), dtype='int') + int(self.map_length*4/5)
        wall1_y = np.linspace(self.map_width*2/5, self.map_width*4/5, int(self.map_width*2/5), dtype='int')

        wall2_x = np.linspace(self.map_length*2/5, self.map_length*4/5, int(self.map_length*2/5), dtype='int')
        wall2_y = np.zeros(int(self.map_length*2/5), dtype='int') + int(self.map_width*3/5)

        wall3_x = np.zeros(int(self.map_width*1/5), dtype='int') + int(self.map_length*2/5)
        wall3_y = np.linspace(self.map_width*2/5, self.map_width*3/5, int(self.map_width*1/5), dtype='int')

        wall4_x = np.linspace(self.map_length*1/5, self.map_length*3/5, int(self.map_length*2/5), dtype='int')
        wall4_y = np.zeros(int(self.map_length*2/5), dtype='int') + int(self.map_width/5)

        obs_x = [wall_bottom_x, wall_top_x, wall_left_x, wall_right_x, wall1_x, wall2_x, wall3_x, wall4_x]
        obs_y = [wall_bottom_y, wall_top_y, wall_left_y, wall_right_y, wall1_y, wall2_y, wall3_y, wall4_y]

        for i, j in zip(obs_x, obs_y):
            self.global_obstacle_x = np.append(self.global_obstacle_x, i)
            self.global_obstacle_y = np.append(self.global_obstacle_y, j)
        self.start_x = self.map_length/10
        self.start_y = self.map_width*9/10
        self.end_x = self.map_length*9/10
        self.end_y = self.map_width/10
        self.global_obstacle = np.stack((self.global_obstacle_x, self.global_obstacle_y), axis=1)


    def maze2(self):
        self.map_width = 2000
        self.map_length = 4000
        self.map_width = int(self.map_width)
        self.map_length = int(self.map_length)
        wall_bottom_x = wall_top_x = np.linspace(0, self.map_length, self.map_length, dtype='int')
        wall_top_y = np.zeros(self.map_length, dtype='int')
        wall_bottom_y = wall_top_y + self.map_width

        wall_right_x = np.zeros(self.map_width, dtype='int') + self.map_length
        wall_right_y = np.linspace(0, self.map_width, self.map_width, dtype='int')

        wall_left_x = np.zeros(self.map_width, dtype='int')
        wall_left_y = np.linspace(0, self.map_width, self.map_width, dtype='int')

        wall1_x = np.linspace(self.map_width*1/5, self.map_width*4/5, int(self.map_width*3/5), dtype='int')
        wall1_y = np.zeros(int(self.map_length*3/5), dtype='int') + int(self.map_width/5)

        wall2_x = np.linspace(self.map_width*1/5, self.map_width*4/5, int(self.map_width*3/5), dtype='int')
        wall2_y = np.zeros(int(self.map_length*3/5), dtype='int') + int(self.map_width*4/5)

        wall3_x = np.zeros(int(self.map_width*3/5), dtype='int') + int(self.map_length*3/5)
        wall3_y = np.linspace(self.map_width*1/5, self.map_width*4/5, int(self.map_width*3/5), dtype='int')

        obs_x = [wall_bottom_x, wall_top_x, wall_left_x, wall_right_x, wall1_x, wall2_x, wall3_x]
        obs_y = [wall_bottom_y, wall_top_y, wall_left_y, wall_right_y, wall1_y, wall2_y, wall3_y]

        for i, j in zip(obs_x, obs_y):
            self.global_obstacle_x = np.append(self.global_obstacle_x, i)
            self.global_obstacle_y = np.append(self.global_obstacle_y, j)
        self.start_x = self.map_length/10
        self.start_y = self.map_width*1/2
        self.end_x = self.map_length*9/10
        self.end_y = self.map_width/2
        self.global_obstacle = np.stack((self.global_obstacle_x, self.global_obstacle_y), axis=1)

class AstarPathPlanning:
    def __init__(self, SharedVariable):
        self.SV = SharedVariable
        self.motion = [
            [1, 0],
            [1, 1],
            [1, -1],
            [0, 1],
            [0, -1],
            [-1, 0],
            [-1, 1],
            [-1, -1]
        ]
        self.node_calculated = dict() # Close set
        # self.calculated_path_dictionary = dict() # Open set
        self.node_use_for_calculation = dict() # Open set
        # self.node_best_path = dict()


    class Node:
        ''' Define parameters in a Node (point) : node_x, node_y, cost, node ID'''
        def __init__(self, x, y, cost, last_node_id):
            self.x, self.y, self.cost, self.last_node_id = x, y, cost, last_node_id 

    def calculate_cost(self, node):
        # distance from node to start point
        g_cost = self.SV.G_cost_factor * round(np.hypot(node.x - self.SV.MAP.start_x, node.y - self.SV.MAP.start_y), 2)
        # (Heuristic) Distance from node to end point
        h_cost = self.SV.H_cost_factor * round(np.hypot(node.x - self.SV.MAP.end_x, node.y - self.SV.MAP.end_y), 2)
        return g_cost + h_cost

    def node_invaild(self, node):
        return True if np.min(np.hypot(self.SV.MAP.global_obstacle_x - node.x, self.SV.MAP.global_obstacle_y - node.y)) < \
            self.SV.rover_size + self.SV.obstacle_size else False

    def repeated_node(self, node):
        '''To check if node is calcualted (In close set)'''
        return True if str(node.x) + ',' + str(node.y) in self.node_calculated else False

    def reset(self):
        self.node_use_for_calculation = dict()
        self.node_calculated = dict()

    def planning(self):
        # start point
        self.reset()
        current_node = self.Node(
            self.SV.MAP.start_x,
            self.SV.MAP.start_y,
            0,
            str(self.SV.MAP.start_x) + ',' + str(self.SV.MAP.start_y)
        )

        reach_target = False
        self.node_use_for_calculation[str(current_node.x) + ',' + str(current_node.y)] = current_node
        while not reach_target:
            # print(len(self.node_use_for_calculation))
            if len(self.node_use_for_calculation) == 0:
                self.SV.route_x, self.SV.route_y = self.calculate_path(current_node, self.node_calculated)
                print("No route to go to target")
                break
            # Open set, which is used for calculation, find lowest cost node for calculation
            current_node_id = min(self.node_use_for_calculation, key=lambda i: self.node_use_for_calculation[i].cost)
            current_node = self.node_use_for_calculation[current_node_id]
            if self.SV.show_progress:
                self.SV.route_x, self.SV.route_y = self.calculate_path(current_node, self.node_calculated)
                self.SV.route_plot.setData(self.SV.route_x, self.SV.route_y)
                pyqtgraph.QtGui.QApplication.processEvents()
                pass
            if np.hypot(current_node.x - self.SV.MAP.end_x, current_node.y - self.SV.MAP.end_y) <= self.SV.step_unit:
                print("Target reached")
                self.SV.route_x, self.SV.route_y = self.calculate_path(current_node, self.node_calculated)
                reach_target = True
                break
            # print(current_node_id)

            # Remove this node from open_set
            del self.node_use_for_calculation[current_node_id]
            # Then add it to close_set, which is the one that already calculated
            self.node_calculated[current_node_id] = current_node
            # Node calculation
            for motion in self.motion:
                new_node = self.Node(
                    current_node.x + self.SV.step_unit * motion[0],
                    current_node.y + self.SV.step_unit * motion[1],
                    0,
                    current_node_id
                )
                if self.node_invaild(new_node):
                    continue
                if self.repeated_node(new_node):
                    continue
                new_node.cost = self.calculate_cost(new_node)
                if str(new_node.x) + ',' + str(new_node.y) not in self.node_use_for_calculation:
                    # Save it if it is a completely new node
                   self.node_use_for_calculation[str(new_node.x) + ',' + str(new_node.y)] = new_node
                else: # If not new node, see if it is the best path for now (lower cost)
                    if self.node_use_for_calculation[str(new_node.x) + ',' + str(new_node.y)].cost > new_node.cost:
                        self.node_use_for_calculation[str(new_node.x) + ',' + str(new_node.y)] = new_node
                        print(self.node_use_for_calculation)
                    elif self.node_use_for_calculation[str(new_node.x) + ',' + str(new_node.y)].cost == new_node.cost:
                        pass
                        # print("unchanged ( {} , {} ) {}".format(new_node.x, new_node.y, new_node.cost))


    def calculate_path(self, current_node, node_calculated):
        '''Start from current node to start point'''
        route_x = [current_node.x]
        route_y = [current_node.y]
        last_node_id = current_node.last_node_id
        while last_node_id != str(self.SV.MAP.start_x) + ',' + str(self.SV.MAP.start_y):
            new_node = node_calculated[last_node_id]
            route_x.append(new_node.x)
            route_y.append(new_node.y)
            last_node_id = new_node.last_node_id
        new_node = node_calculated[last_node_id]
        route_x.append(new_node.x)
        route_y.append(new_node.y)
        return route_x, route_y


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

class App:
    def __init__(self):
        os.environ["QT_AUTO_SCREEN_SCALE_FACTOR"] = "1"
        app = QtWidgets.QApplication(sys.argv)
        app.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
        self.MainWindow = QtWidgets.QMainWindow()

        self.gui = sim_ui.Ui_MainWindow()
        self.gui.setupUi(self.MainWindow)

        self.SV = SharedVariable()
        self.SV.gui = self.gui
        self.Pen = Pens()
        map_name_list = [
            'maze1',
            'maze2',
        ]

        self.rover_size = 50
        self.px_mode = True
        self.run_flag = False


        for name in map_name_list: self.gui.combo_map.addItem(name)
        print(self.gui.combo_map.currentText())


        self.gui.spin_start_x.setValue(self.SV.MAP.start_x)
        self.gui.spin_start_y.setValue(self.SV.MAP.start_y)
        self.gui.spin_end_x.setValue(self.SV.MAP.end_x)
        self.gui.spin_end_y.setValue(self.SV.MAP.end_y)

        self.lidar = SimulatedLidar(self.SV)
        self.astar = AstarPathPlanning(self.SV)



        self.map_plot_widget = pyqtgraph.PlotWidget(background='w')
        self.route_plot = self.map_plot_widget.plot(pen=self.Pen.route_pen, symbol='s')
        self.global_obstacle_plot = pyqtgraph.ScatterPlotItem(
            symbol='o',
            size=self.SV.obstacle_size,
            brush=(0, 0, 0),
            pen=self.Pen.transparent,
        )
        self.local_obstacle_plot = pyqtgraph.ScatterPlotItem(
            symbol='o',
            size=self.SV.obstacle_size,
            brush=(0, 89, 255),
            pen=self.Pen.transparent,
        )
        self.end_point_plot = pyqtgraph.ScatterPlotItem(
            symbol='x',
            size=self.SV.rover_size*2,
            brush=(255, 0, 0)
        )
        self.rover_plot = pyqtgraph.ScatterPlotItem(
            symbol='o',
            size=self.rover_size,
            brush=(0, 255 ,0)
        )

        self.map_plot_widget.addItem(self.global_obstacle_plot)
        self.map_plot_widget.addItem(self.local_obstacle_plot)
        self.map_plot_widget.addItem(self.end_point_plot)
        self.map_plot_widget.addItem(self.rover_plot)
        self.gui.horizontalLayout_3.addWidget(self.map_plot_widget)


        self.gui.spin_start_x.valueChanged.connect(self.change_start_end_point)
        self.gui.spin_start_y.valueChanged.connect(self.change_start_end_point)
        self.gui.spin_end_x.valueChanged.connect(self.change_start_end_point)
        self.gui.spin_end_y.valueChanged.connect(self.change_start_end_point)
        self.gui.spin_rover_radius.valueChanged.connect(self.change_rover_size)
        self.gui.spin_safe_radius.valueChanged.connect(self.change_obstacle_size)
        self.gui.check_pxmode.clicked.connect(lambda: self.change_pxmode())
        self.gui.button_start.clicked.connect(self.button_start_clicked)
        self.gui.button_pause.clicked.connect(self.button_pause_clicked)
        self.gui.button_reset.clicked.connect(self.button_reset_clicked)
        self.gui.dspin_gcost.valueChanged.connect(self.change_g_h_cost)
        self.gui.dspin_hcost.valueChanged.connect(self.change_g_h_cost)
        self.gui.check_progress.toggled.connect(self.change_show_progress)
        self.gui.spin_unitstep.valueChanged.connect(self.change_step_unit)
        self.gui.combo_map.currentIndexChanged.connect(self.change_map)

        self.plot_map()

        self.SV.route_plot = self.route_plot

        self.MainWindow.show()
        sys.exit(app.exec_())

    def change_map(self):
        self.SV.MAP.map_name = self.gui.combo_map.currentText()
        self.SV.MAP.generate_map()
        self.plot_map()

    def plot_map(self):
        self.global_obstacle_plot.setData(self.SV.MAP.global_obstacle_x, self.SV.MAP.global_obstacle_y)
        self.rover_plot.setData([self.SV.MAP.start_x], [self.SV.MAP.start_y])
        self.end_point_plot.setData([self.SV.MAP.end_x], [self.SV.MAP.end_y])

    def change_start_end_point(self):
        if np.min(np.hypot(self.SV.MAP.global_obstacle_x - self.gui.spin_start_x.value(),\
            self.SV.MAP.global_obstacle_y - self.gui.spin_start_y.value())) <= self.SV.obstacle_size + self.SV.rover_size or\
                np.min(np.hypot(self.SV.MAP.global_obstacle_x - self.gui.spin_end_x.value(),\
                    self.SV.MAP.global_obstacle_y - self.gui.spin_start_y.value())) <= self.SV.obstacle_size + self.SV.rover_size:
            print('ROVER or End Point hit obstacle')
        else:
            self.SV.MAP.start_x = self.SV.rover_x = self.gui.spin_start_x.value()
            self.SV.MAP.start_y = self.SV.rover_y = self.gui.spin_start_y.value()
            self.SV.MAP.end_x = self.gui.spin_end_x.value()
            self.SV.MAP.end_y = self.gui.spin_end_y.value()
            self.rover_plot.setData([self.SV.MAP.start_x], [self.SV.MAP.start_y])
            self.end_point_plot.setData([self.SV.MAP.end_x], [self.SV.MAP.end_y])

    def change_g_h_cost(self):
        self.SV.G_cost_factor = self.gui.dspin_gcost.value()
        self.SV.H_cost_factor = self.gui.dspin_hcost.value()

    def change_step_unit(self):
        self.SV.step_unit = self.gui.spin_unitstep.value()

    def change_show_progress(self):
        self.SV.show_progress = False if self.SV.show_progress else True
        print("Show progress {}".format(self.SV.show_progress))

    def change_rover_size(self):
        self.SV.rover_size = self.gui.spin_rover_radius.value()
        self.rover_plot.setSize(self.SV.rover_size)

    def change_obstacle_size(self):
        self.SV.obstacle_size = self.gui.spin_safe_radius.value()
        self.global_obstacle_plot.setSize(self.SV.obstacle_size)
        self.local_obstacle_plot.setSize(self.SV.obstacle_size)

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

    def update_shared_config(self):
        if not self.SV.shared_config.empty(): self.SV.shared_config.get()    
        self.SV.shared_config.put([
            self.gui.spin_time_delay.value(),
            self.SV.rover_x,
            self.SV.rover_y,
            self.run_flag,
            self.SV.lidar_scan_radius
        ])

    def button_start_clicked(self):
        self.astar.planning()
        self.route_plot.setData(self.SV.route_x, self.SV.route_y)

    def button_pause_clicked(self):
        pass

    def button_reset_clicked(self):
        self.SV.MAP = Map(self.gui.combo_map.currentText())
        self.gui.spin_start_x.setValue(self.SV.MAP.start_x)
        self.gui.spin_start_y.setValue(self.SV.MAP.start_y)
        self.gui.spin_end_x.setValue(self.SV.MAP.end_x)
        self.gui.spin_end_y.setValue(self.SV.MAP.end_y)
        self.change_start_end_point()






if __name__ == "__main__":
    App()