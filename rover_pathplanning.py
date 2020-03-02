import time
import numpy as np
import pyqtgraph
import scipy.spatial
from rover_curve_fitting import Bspline


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
        g_cost = self.SV.AS.G_cost_factor * round(np.hypot(node.x - self.SV.AS.start_x, node.y - self.SV.AS.start_y), 2)
        # (Heuristic) Distance from node to end point
        h_cost = self.SV.AS.H_cost_factor * round(np.hypot(node.x - self.SV.AS.end_x, node.y - self.SV.AS.end_y), 2)
        return g_cost + h_cost

    def node_invaild(self, node):
        return True if np.min(np.hypot(self.SV.GOBS.global_obstacle_x - node.x, self.SV.GOBS.global_obstacle_y - node.y)) < \
            self.SV.AS.rover_size + self.SV.AS.obstacle_size else False

    def repeated_node(self, node):
        '''To check if node is calcualted (In close set)'''
        return True if str(node.x) + ',' + str(node.y) in self.node_calculated else False

    def reset(self):
        self.node_use_for_calculation = dict()
        self.node_calculated = dict()

    def planning(self):
        start_time = time.time()
        # start point
        self.reset()
        current_node = self.Node(
            self.SV.AS.start_x,
            self.SV.AS.start_y,
            0,
            str(self.SV.AS.start_x) + ',' + str(self.SV.AS.start_y)
        )
        target_node = self.Node(
            self.SV.AS.end_x,
            self.SV.AS.end_y,
            0,
            ""
        )
        reach_target = False
        self.node_use_for_calculation[str(current_node.x) + ',' + str(current_node.y)] = current_node
        while not reach_target:
            # print(len(self.node_use_for_calculation))
            if len(self.node_use_for_calculation) == 0:
                self.SV.AS.route_x, self.SV.AS.route_y = self.calculate_path(current_node, self.node_calculated)
                print("No route to go to target")
                break
            # Open set, which is used for calculation, find lowest cost node for calculation
            current_node_id = min(self.node_use_for_calculation, key=lambda i: self.node_use_for_calculation[i].cost)
            current_node = self.node_use_for_calculation[current_node_id]

            # Remove this node from open_set
            del self.node_use_for_calculation[current_node_id]
            # Then add it to close_set, which is the one that already calculated
            self.node_calculated[current_node_id] = current_node

            # if self.SV.show_progress:
            #     self.SV.route_x, self.SV.route_y = self.calculate_path(current_node, self.node_calculated)
            #     self.SV.route_plot.setData(self.SV.route_x, self.SV.route_y)
            #     pyqtgraph.QtGui.QApplication.processEvents()
            #     pass
            if np.hypot(current_node.x - self.SV.AS.end_x, current_node.y - self.SV.AS.end_y) <= self.SV.AS.step_unit:
                print("Target reached")
                target_node.last_node_id = str(current_node.x) + ',' + str(current_node.y)
                self.SV.AS.route_x, self.SV.AS.route_y = self.calculate_path(target_node, self.node_calculated)
                reach_target = True
                break
            # print(current_node_id)


            # Node calculation
            for motion in self.motion:
                new_node = self.Node(
                    current_node.x + self.SV.AS.step_unit * motion[0],
                    current_node.y + self.SV.AS.step_unit * motion[1],
                    current_node.cost,
                    current_node_id
                )
                if self.node_invaild(new_node):
                    continue
                if self.repeated_node(new_node):
                    continue
                new_node.cost += np.hypot(motion[0], motion[1])
                if str(new_node.x) + ',' + str(new_node.y) not in self.node_use_for_calculation:
                    # Save it if it is a completely new node
                   self.node_use_for_calculation[str(new_node.x) + ',' + str(new_node.y)] = new_node
                else: # If not new node, see if it is the best path for now (lower cost)
                    if self.node_use_for_calculation[str(new_node.x) + ',' + str(new_node.y)].cost > new_node.cost:
                        self.node_use_for_calculation[str(new_node.x) + ',' + str(new_node.y)] = new_node
                    elif self.node_use_for_calculation[str(new_node.x) + ',' + str(new_node.y)].cost == new_node.cost:
                        pass
        end_time = time.time()
        self.SV.AS.astar_planning_time = end_time - start_time


    def calculate_path(self, current_node, node_calculated):
        '''Start from current node to start point'''
        route_x = [current_node.x]
        route_y = [current_node.y]
        last_node_id = current_node.last_node_id
        while True:
            if last_node_id == str(self.SV.AS.start_x) + ',' + str(self.SV.AS.start_y):
                route_x.append(self.SV.AS.start_x)
                route_y.append(self.SV.AS.start_y)
                break
            new_node = node_calculated[last_node_id]
            route_x.append(new_node.x)
            route_y.append(new_node.y)
            last_node_id = new_node.last_node_id
        return route_x, route_y




class AstarPathPlanning_sim:
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
        g_cost = self.SV.AS.G_cost_factor * round(np.hypot(node.x - self.SV.AS.start_x, node.y - self.SV.AS.start_y), 2)
        # (Heuristic) Distance from node to end point
        h_cost = self.SV.AS.H_cost_factor * round(np.hypot(node.x - self.SV.AS.end_x, node.y - self.SV.AS.end_y), 2)
        return g_cost + h_cost

    def node_invaild(self, node):
        return True if np.min(np.hypot(self.SV.GOBS.global_obstacle_x - node.x, self.SV.GOBS.global_obstacle_y - node.y)) < \
            self.SV.AS.rover_size + self.SV.AS.obstacle_size else False

    def repeated_node(self, node):
        '''To check if node is calcualted (In close set)'''
        return True if str(node.x) + ',' + str(node.y) in self.node_calculated else False

    def reset(self):
        self.node_use_for_calculation = dict()
        self.node_calculated = dict()

    def planning(self, show_progress=False):
        start_time = time.time()
        # start point
        self.reset()
        current_node = self.Node(
            self.SV.AS.start_x,
            self.SV.AS.start_y,
            0,
            str(self.SV.AS.start_x) + ',' + str(self.SV.AS.start_y)
        )
        target_node = self.Node(
            self.SV.AS.end_x,
            self.SV.AS.end_y,
            0,
            ""
        )
        reach_target = False
        self.node_use_for_calculation[str(current_node.x) + ',' + str(current_node.y)] = current_node
        while not reach_target:
            # print(len(self.node_use_for_calculation))
            if len(self.node_use_for_calculation) == 0:
                self.SV.AS.route_x, self.SV.AS.route_y = self.calculate_path(current_node, self.node_calculated)
                print("No route to go to target")
                break
            # Open set, which is used for calculation, find lowest cost node for calculation
            current_node_id = min(self.node_use_for_calculation, key=lambda i: self.node_use_for_calculation[i].cost)
            current_node = self.node_use_for_calculation[current_node_id]

            # Remove this node from open_set
            del self.node_use_for_calculation[current_node_id]
            # Then add it to close_set, which is the one that already calculated
            self.node_calculated[current_node_id] = current_node

            if show_progress:
                self.SV.AS.route_x, self.SV.AS.route_y = self.calculate_path(current_node, self.node_calculated)
                self.SV.GUI.route_plot.setData(self.SV.AS.route_x, self.SV.AS.route_y)
                pyqtgraph.QtGui.QApplication.processEvents()
                pass
            if np.hypot(current_node.x - self.SV.AS.end_x, current_node.y - self.SV.AS.end_y) <= self.SV.AS.step_unit:
                print("Target reached")
                target_node.last_node_id = str(current_node.x) + ',' + str(current_node.y)
                self.SV.AS.route_x, self.SV.AS.route_y = self.calculate_path(target_node, self.node_calculated)
                reach_target = True
                break
            # print(current_node_id)


            # Node calculation
            for motion in self.motion:
                new_node = self.Node(
                    current_node.x + self.SV.AS.step_unit * motion[0],
                    current_node.y + self.SV.AS.step_unit * motion[1],
                    current_node.cost,
                    current_node_id
                )
                if self.node_invaild(new_node):
                    continue
                if self.repeated_node(new_node):
                    continue
                new_node.cost += np.hypot(motion[0], motion[1])
                if str(new_node.x) + ',' + str(new_node.y) not in self.node_use_for_calculation:
                    # Save it if it is a completely new node
                   self.node_use_for_calculation[str(new_node.x) + ',' + str(new_node.y)] = new_node
                else: # If not new node, see if it is the best path for now (lower cost)
                    if self.node_use_for_calculation[str(new_node.x) + ',' + str(new_node.y)].cost > new_node.cost:
                        self.node_use_for_calculation[str(new_node.x) + ',' + str(new_node.y)] = new_node
                    elif self.node_use_for_calculation[str(new_node.x) + ',' + str(new_node.y)].cost == new_node.cost:
                        pass
        end_time = time.time()
        self.SV.AS.astar_planning_time = end_time - start_time


    def calculate_path(self, current_node, node_calculated):
        '''Start from current node to start point'''
        route_x = [current_node.x]
        route_y = [current_node.y]
        last_node_id = current_node.last_node_id
        while True:
            if last_node_id == str(self.SV.AS.start_x) + ',' + str(self.SV.AS.start_y):
                route_x.append(self.SV.AS.start_x)
                route_y.append(self.SV.AS.start_y)
                break
            new_node = node_calculated[last_node_id]
            route_x.append(new_node.x)
            route_y.append(new_node.y)
            last_node_id = new_node.last_node_id
        return route_x, route_y





class AstarPathPlanning_v2:
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
        self.vector = []
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
        target_node = self.Node(
            self.SV.MAP.end_x,
            self.SV.MAP.end_y,
            0,
            ""
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

            # Remove this node from open_set
            del self.node_use_for_calculation[current_node_id]
            # Then add it to close_set, which is the one that already calculated
            self.node_calculated[current_node_id] = current_node

            if self.SV.show_progress:
                self.SV.route_x, self.SV.route_y = self.calculate_path(current_node, self.node_calculated)
                self.SV.route_plot.setData(self.SV.route_x, self.SV.route_y)
                pyqtgraph.QtGui.QApplication.processEvents()
                pass
            if np.hypot(current_node.x - self.SV.MAP.end_x, current_node.y - self.SV.MAP.end_y) <= self.SV.step_unit:
                print("Target reached")
                target_node.last_node_id = str(current_node.x) + ',' + str(current_node.y)
                self.SV.route_x, self.SV.route_y = self.calculate_path(target_node, self.node_calculated)
                reach_target = True
                break
            # print(current_node_id)


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
        while True:
            if last_node_id == str(self.SV.MAP.start_x) + ',' + str(self.SV.MAP.start_y):
                break
            new_node = node_calculated[last_node_id]
            route_x.append(new_node.x)
            route_y.append(new_node.y)
            last_node_id = new_node.last_node_id
        return route_x, route_y

