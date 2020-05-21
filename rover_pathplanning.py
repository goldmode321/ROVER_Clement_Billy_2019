import time
import numpy as np
import pyqtgraph
import scipy.spatial
import threading
import itertools

from rover_curve_fitting import Bspline

class AstarPathPlanning:
    def __init__(self, SharedVariable):
        self.SV = SharedVariable
        self.AS = self.SV.AS

        self.motion = [
            [0, 'forward'],
            [30, 'forward'],
            [330, 'forward'],
            [180, 'backward'],
            [150, 'backward'],
            [210, 'backward']
        ]
        self.motion_dict = {
            0:[1, 0],
            30:[1, 0.5],
            60:[0.5, 1],
            90:[0, 1],
            120:[-0.5, 1],
            150:[-1, 0.5],
            180:[-1, 0],
            210:[-1, -0.5],
            240:[-0.5, -1],
            270:[0, -1],
            300:[0.5, -1],
            330:[1, -0.5],
        }
        self.node_calculated = dict() # Close set
        # self.calculated_path_dictionary = dict() # Open set
        self.node_use_for_calculation = dict() # Open set
        # self.node_best_path = dict()


    class Node:
        ''' Define parameters in a Node (point) : node_x, node_y, cost, node ID'''
        def __init__(self, x, y, cost, last_node_id, attitude):
            self.x, self.y, self.cost, self.last_node_id, self.attitude = x, y, cost, last_node_id, attitude

    def calculate_cost(self, node, movement, motion):
        node.cost += round(np.hypot(movement[0], movement[1]), 1) + (0 if motion[1] == 'forward' else 1)*self.AS.step_unit

        return node.cost

    def node_invaild(self, node):
        return True if np.min(np.hypot(self.SV.GOBS.global_obstacle_x - node.x, self.SV.GOBS.global_obstacle_y - node.y)) < \
            self.AS.rover_size + self.AS.obstacle_size else False

    def repeated_node(self, node):
        '''To check if node is calcualted (In close set)'''
        return True if str(node.x) + ',' + str(node.y) in self.node_calculated else False

    def reset(self):
        self.node_use_for_calculation = dict()
        self.node_calculated = dict()

    def _show_progress(self, current_node, node_calculated=None):
        if self.SV.GUI.show_progress:
            node_calculated = self.node_calculated if node_calculated is None else node_calculated
            self.calculate_path(current_node, node_calculated)
            # self.SV.GUI.route_plot.setData(self.AS.route_x, self.AS.route_y)
            # pyqtgraph.QtGui.QApplication.processEvents()
            time.sleep(self.SV.GUI.show_progress_delay)


    def planning(self):
        start_time = time.time()
        # start point
        self.reset()
        self.AS.attitude[0] = int(self.AS.attitude[0]/30)*30 # fit into motion dictionary
        current_node = self.Node(
            self.AS.start_x,
            self.AS.start_y,
            0,
            str(self.AS.start_x) + ',' + str(self.AS.start_y),
            self.AS.attitude
        )
        target_node = self.Node(
            self.AS.end_x,
            self.AS.end_y,
            0,
            "",
            self.AS.attitude
        )
        self.AS.reach_target = reach_target = False
        self.node_use_for_calculation[str(current_node.x) + ',' + str(current_node.y)] = current_node
        while not reach_target:
            # print(len(self.node_use_for_calculation))
            if len(self.node_use_for_calculation) == 0:
                self.calculate_path(current_node, self.node_calculated)
                print("No route to go to target")
                self.AS.reach_target = False
                break
            # Open set, which is used for calculation, find lowest cost node for calculation
            current_node_id = min(self.node_use_for_calculation, key=lambda i: self.node_use_for_calculation[i].cost)
            current_node = self.node_use_for_calculation[current_node_id]

            # Remove this node from open_set
            del self.node_use_for_calculation[current_node_id]
            # Then add it to close_set, which is the one that already calculated
            self.node_calculated[current_node_id] = current_node

            self._show_progress(current_node)

            if np.hypot(current_node.x - self.AS.end_x, current_node.y - self.AS.end_y) <= self.AS.step_unit:
                print("Target reached")
                target_node.last_node_id = str(current_node.x) + ',' + str(current_node.y)
                target_node.attitude = current_node.attitude
                self.calculate_path(target_node, self.node_calculated)
                self.reach_target = reach_target = True
                break
            # print(current_node_id)


            # Node calculation
            for motion in self.motion:
                movement = self.motion_dict[(current_node.attitude[0]+motion[0])%360]
                new_node = self.Node(
                    current_node.x + self.AS.step_unit * movement[0],
                    current_node.y + self.AS.step_unit * movement[1],
                    current_node.cost,
                    current_node_id,
                    [(current_node.attitude[0]+motion[0])%360, "forward" if motion[1] == current_node.attitude[1] else "backward"]
                )
                if self.node_invaild(new_node):
                    continue
                if self.repeated_node(new_node):
                    continue
                # new_node.cost += np.hypot(movement[0], movement[1])
                new_node.cost = self.calculate_cost(new_node, movement, motion)
                if str(new_node.x) + ',' + str(new_node.y) not in self.node_use_for_calculation:
                    # Save it if it is a completely new node
                   self.node_use_for_calculation[str(new_node.x) + ',' + str(new_node.y)] = new_node
                else: # If not new node, see if it is the best path for now (lower cost)
                    if self.node_use_for_calculation[str(new_node.x) + ',' + str(new_node.y)].cost > new_node.cost:
                        self.node_use_for_calculation[str(new_node.x) + ',' + str(new_node.y)] = new_node
                    elif self.node_use_for_calculation[str(new_node.x) + ',' + str(new_node.y)].cost == new_node.cost:
                        pass

        end_time = time.time()
        self.AS.astar_planning_time = end_time - start_time

    def calculate_path(self, current_node, node_calculated):
        '''Start from current node to start point'''
        route_x = [current_node.x]
        route_y = [current_node.y]
        forward_backward_list = [current_node.attitude[1]]
        last_node_id = current_node.last_node_id
        while True:
            if last_node_id == str(self.AS.start_x) + ',' + str(self.AS.start_y):
                route_x.append(self.AS.start_x)
                route_y.append(self.AS.start_y)
                forward_backward_list.append(self.AS.attitude[1])
                route_x.reverse()
                route_y.reverse()
                forward_backward_list.reverse()
                break
            new_node = node_calculated[last_node_id]
            route_x.append(new_node.x)
            route_y.append(new_node.y)
            forward_backward_list.append(new_node.attitude[1])
            last_node_id = new_node.last_node_id
        self.AS.route_x, self.AS.route_y, self.AS.forward_backward_record = route_x, route_y, forward_backward_list




class AstarPathPlanning_ori:
    def __init__(self, SharedVariable):
        self.SV = SharedVariable
        self.AS = self.SV.AS

        # self.motion = [
        #     [0, 'forward'],
        #     [45, 'forward'],
        #     [90, 'forward'],
        #     [315, 'forward'],
        #     [180, 'forward'],
        #     [135, 'forward'],
        #     [225, 'forward'],
        #     [270, 'forward'],
        # ]
        self.motion = [
            [0, 'forward'],
            [11.25, 'forward'],
            [22.5, 'forward'],
            [33.75, 'forward'],
            [45, 'forward'],
            [56.25, 'forward'],
            [67.5, 'forward'],
            [78.75, 'forward'],
            [90, 'forward'],
            [101.25, 'forward'],
            [112.5, 'forward'],
            [123.75, 'forward'],
            [135, 'forward'],
            [146.25, 'forward'],
            [157.5, 'forward'],
            [168.75, 'forward'],
            [180, 'forward'],
            [191.25, 'forward'],
            [202.5, 'forward'],
            [213.75, 'forward'],
            [225, 'forward'],
            [236.25, 'forward'],
            [247.5, 'forward'],
            [258.75, 'forward'],
            [270, 'forward'],
            [281.25, 'forward'],
            [292.5, 'forward'],
            [303.75, 'forward'],
            [315, 'forward'],
            [326.25, 'forward'],
            [337.5, 'forward'],
            [348.75, 'forward'],
        ]
        # self.motion_dict = {
        #     0:[1, 0],
        #     45:[1, 1],
        #     90:[0, 1],
        #     135:[-1, 1],
        #     180:[-1, 0],
        #     225:[-1, -1],
        #     270:[0, -1],
        #     315:[1, -1],
        # }
        self.motion_dict = {
            0:[1, 0],
            11.25:[1, 0.25],
            22.5:[1, 0.5],
            33.75:[1, 0.75],
            45:[1, 1],
            56.25:[0.75, 1],
            67.5:[0.5, 1],
            78.75:[0.25, 1],
            90:[0, 1],
            101.25:[-0.25 ,1],
            112.5:[-0.5, 1],
            123.75:[-0.75, 1],
            135:[-1, 1],
            146.25:[-1, 0.75],
            157.5:[-1, 0.5],
            168.75:[-1, 0.25],
            180:[-1, 0],
            191.25:[-1, -0.25],
            202.5:[-1, -0.5],
            213.75:[-1, -0.75],
            225:[-1, -1],
            236.25:[-0.75, -1],
            247.5:[-0.5, -1],
            258.75:[-0.25, -1],
            270:[0, -1],
            281.25:[0.25, -1],
            292.5:[0.5, -1],
            303.75:[0.75, -1],
            315:[1, -1],
            326.25:[1, -0.75],
            337.5:[1, -0.5],
            348.75:[1, -0.75],
        }
        self.node_calculated = dict() # Close set
        # self.calculated_path_dictionary = dict() # Open set
        self.node_use_for_calculation = dict() # Open set
        # self.node_best_path = dict()


    class Node:
        ''' Define parameters in a Node (point) : node_x, node_y, cost, node ID'''
        def __init__(self, x, y, cost, last_node_id, attitude, num_of_step):
            self.x, self.y, self.cost, self.last_node_id, self.attitude, self.num_of_step = x, y, cost, last_node_id, attitude, num_of_step

    def calculate_cost(self, node, movement, motion):
        # node.cost += round(np.hypot(movement[0], movement[1]), 1) + (0 if motion[1] == 'forward' else 1)*self.AS.step_unit
        #      + np.hypot(node.x - self.AS.end_x, node.y - self.AS.end_y)*0.5
        node.cost = (round(np.hypot(movement[0], movement[1]), 2)*self.AS.step_unit) + node.num_of_step*self.AS.step_unit\
             + np.hypot(node.x - self.AS.end_x, node.y - self.AS.end_y)*1
        return node.cost

    def node_invaild(self, node):
        return True if np.min(np.hypot(self.SV.GOBS.global_obstacle_x - node.x, self.SV.GOBS.global_obstacle_y - node.y)) < \
            self.AS.rover_size + self.AS.obstacle_size else False

    def repeated_node(self, node):
        '''To check if node is calcualted (In close set)'''
        return True if str(node.x) + ',' + str(node.y) in self.node_calculated else False

    def reset(self):
        self.node_use_for_calculation = dict()
        self.node_calculated = dict()

    def _show_progress(self, current_node, node_calculated=None):
        if self.SV.GUI.show_progress:
            node_calculated = self.node_calculated if node_calculated is None else node_calculated
            self.calculate_path(current_node, node_calculated)
            # self.SV.GUI.route_plot.setData(self.AS.route_x, self.AS.route_y)
            # pyqtgraph.QtGui.QApplication.processEvents()
            time.sleep(self.SV.GUI.show_progress_delay)


    def planning(self):
        start_time = time.time()
        # start point
        self.reset()
        self.AS.attitude[0] = int(self.AS.attitude[0]/30)*30 # fit into motion dictionary
        current_node = self.Node(
            self.AS.start_x,
            self.AS.start_y,
            0,
            str(self.AS.start_x) + ',' + str(self.AS.start_y),
            self.AS.attitude,
            0
        )
        target_node = self.Node(
            self.AS.end_x,
            self.AS.end_y,
            0,
            "",
            self.AS.attitude,
            0
        )
        self.AS.reach_target = reach_target = False
        self.node_use_for_calculation[str(current_node.x) + ',' + str(current_node.y)] = current_node
        total_step = 0
        while not reach_target:
            total_step += 1
            # print(len(self.node_use_for_calculation))
            if len(self.node_use_for_calculation) == 0:
                self.calculate_path(current_node, self.node_calculated)
                print("No route to go to target")
                self.AS.reach_target = False
                break
            # Open set, which is used for calculation, find lowest cost node for calculation
            current_node_id = min(self.node_use_for_calculation, key=lambda i: self.node_use_for_calculation[i].cost)
            current_node = self.node_use_for_calculation[current_node_id]

            # Remove this node from open_set
            del self.node_use_for_calculation[current_node_id]
            # Then add it to close_set, which is the one that already calculated
            self.node_calculated[current_node_id] = current_node



            if np.hypot(current_node.x - self.AS.end_x, current_node.y - self.AS.end_y) <= self.AS.step_unit:
                print("Target reached")
                target_node.last_node_id = str(current_node.x) + ',' + str(current_node.y)
                target_node.attitude = current_node.attitude
                self.calculate_path(target_node, self.node_calculated)
                self.reach_target = reach_target = True
                break
            # print(current_node_id)

            self._show_progress(current_node)

            # Node calculation
            for motion in self.motion:
                movement = self.motion_dict[motion[0]]
                new_node = self.Node(
                    current_node.x + self.AS.step_unit * movement[0],
                    current_node.y + self.AS.step_unit * movement[1],
                    current_node.cost,
                    current_node_id,
                    [motion[0], "forward" if motion[1] == current_node.attitude[1] else "backward"],
                    total_step,
                )
                if self.node_invaild(new_node):
                    continue
                if self.repeated_node(new_node):
                    continue
                # new_node.cost += np.hypot(movement[0], movement[1])
                new_node.cost = self.calculate_cost(new_node, movement, motion)
                if str(new_node.x) + ',' + str(new_node.y) not in self.node_use_for_calculation:
                    # Save it if it is a completely new node
                   self.node_use_for_calculation[str(new_node.x) + ',' + str(new_node.y)] = new_node
                else: # If not new node, see if it is the best path for now (lower cost)
                    if self.node_use_for_calculation[str(new_node.x) + ',' + str(new_node.y)].cost > new_node.cost:
                        self.node_use_for_calculation[str(new_node.x) + ',' + str(new_node.y)] = new_node
                    elif self.node_use_for_calculation[str(new_node.x) + ',' + str(new_node.y)].cost == new_node.cost:
                        pass

        end_time = time.time()
        self.AS.astar_planning_time = end_time - start_time

    def calculate_path(self, current_node, node_calculated):
        '''Start from current node to start point'''
        route_x = [current_node.x]
        route_y = [current_node.y]
        forward_backward_list = [current_node.attitude[1]]
        last_node_id = current_node.last_node_id
        while True:
            if last_node_id == str(self.AS.start_x) + ',' + str(self.AS.start_y):
                route_x.append(self.AS.start_x)
                route_y.append(self.AS.start_y)
                forward_backward_list.append(self.AS.attitude[1])
                route_x.reverse()
                route_y.reverse()
                forward_backward_list.reverse()
                break
            new_node = node_calculated[last_node_id]
            route_x.append(new_node.x)
            route_y.append(new_node.y)
            forward_backward_list.append(new_node.attitude[1])
            last_node_id = new_node.last_node_id
        self.AS.route_x, self.AS.route_y, self.AS.forward_backward_record = route_x, route_y, forward_backward_list


class AstarPathPlanningThread(AstarPathPlanning, threading.Thread):
    def __init__(self, SharedVariable):
        AstarPathPlanning.__init__(self, SharedVariable)
        threading.Thread.__init__(self, daemon=True)
        self.start()
    def run(self):
        self.planning()
        


class AstarPathPlanning_sim(AstarPathPlanning_ori):
    def __init__(self, SharedVariable):
        self.SV = SharedVariable
        super().__init__(self.SV)



class AstarPathPlanning_sim_v2(AstarPathPlanning_ori):
    def __init__(self, SharedVariable):
        self.SV = SharedVariable
        super().__init__(self.SV)

        self.motion = [
            [0, 'forward'],
            [30, 'forward'],
            [330, 'forward'],
            # [180, 'backward'],
            [150, 'backward'],
            [210, 'backward']
        ]
        self.motion_dict = {
            0:[1, 0],
            30:[1, 0.5],
            60:[0.5, 1],
            90:[0, 1],
            120:[-0.5, 1],
            150:[-1, 0.5],
            180:[-1, 0],
            210:[-1, -0.5],
            240:[-0.5, -1],
            270:[0, -1],
            300:[0.5, -1],
            330:[1, -0.5],
        }
        self.node_calculated = dict() # Close set
        # self.calculated_path_dictionary = dict() # Open set
        self.node_use_for_calculation = dict() # Open set
        # self.node_best_path = dict()


    class Node:
        ''' Define parameters in a Node (point) : node_x, node_y, cost, node ID'''
        def __init__(self, x, y, cost, last_node_id, attitude):
            self.x, self.y, self.cost, self.last_node_id, self.attitude = x, y, cost, last_node_id, attitude

    def calculate_cost(self, node, movement, motion):
        node.cost += round(np.hypot(movement[0], movement[1]), 1) + (0 if motion[1] == 'forward' else 1)*self.AS.step_unit

        return node.cost


    def planning(self):
        start_time = time.time()
        # start point
        self.reset()
        current_node = self.Node(
            self.AS.start_x,
            self.AS.start_y,
            0,
            str(self.AS.start_x) + ',' + str(self.AS.start_y),
            self.AS.attitude
        )
        target_node = self.Node(
            self.AS.end_x,
            self.AS.end_y,
            0,
            "",
            self.AS.attitude
        )
        reach_target = False
        self.node_use_for_calculation[str(current_node.x) + ',' + str(current_node.y)] = current_node
        while not reach_target:
            # print(len(self.node_use_for_calculation))
            if len(self.node_use_for_calculation) == 0:
                self.calculate_path(current_node, self.node_calculated)
                print("No route to go to target")
                break
            # Open set, which is used for calculation, find lowest cost node for calculation
            current_node_id = min(self.node_use_for_calculation, key=lambda i: self.node_use_for_calculation[i].cost)
            current_node = self.node_use_for_calculation[current_node_id]

            # Remove this node from open_set
            del self.node_use_for_calculation[current_node_id]
            # Then add it to close_set, which is the one that already calculated
            self.node_calculated[current_node_id] = current_node

            self._show_progress(current_node)

            if np.hypot(current_node.x - self.AS.end_x, current_node.y - self.AS.end_y) <= self.AS.step_unit:
                print("Target reached")
                target_node.last_node_id = str(current_node.x) + ',' + str(current_node.y)
                target_node.attitude = current_node.attitude
                self.calculate_path(target_node, self.node_calculated)
                reach_target = True
                break
            # print(current_node_id)


            # Node calculation
            for motion in self.motion:
                movement = self.motion_dict[(current_node.attitude[0]+motion[0])%360]
                new_node = self.Node(
                    current_node.x + self.AS.step_unit * movement[0],
                    current_node.y + self.AS.step_unit * movement[1],
                    current_node.cost,
                    current_node_id,
                    [(current_node.attitude[0]+motion[0])%360, "forward" if motion[1] == current_node.attitude[1] else "backward"]
                )
                if self.node_invaild(new_node):
                    continue
                if self.repeated_node(new_node):
                    continue
                # new_node.cost += np.hypot(movement[0], movement[1])
                new_node.cost = self.calculate_cost(new_node, movement, motion)
                if str(new_node.x) + ',' + str(new_node.y) not in self.node_use_for_calculation:
                    # Save it if it is a completely new node
                   self.node_use_for_calculation[str(new_node.x) + ',' + str(new_node.y)] = new_node
                else: # If not new node, see if it is the best path for now (lower cost)
                    if self.node_use_for_calculation[str(new_node.x) + ',' + str(new_node.y)].cost > new_node.cost:
                        self.node_use_for_calculation[str(new_node.x) + ',' + str(new_node.y)] = new_node
                    elif self.node_use_for_calculation[str(new_node.x) + ',' + str(new_node.y)].cost == new_node.cost:
                        pass

        end_time = time.time()
        self.AS.astar_planning_time = end_time - start_time

    def calculate_path(self, current_node, node_calculated):
        '''Start from current node to start point'''
        route_x = [current_node.x]
        route_y = [current_node.y]
        forward_backward_list = [current_node.attitude[1]]
        last_node_id = current_node.last_node_id
        while True:
            if last_node_id == str(self.AS.start_x) + ',' + str(self.AS.start_y):
                route_x.append(self.AS.start_x)
                route_y.append(self.AS.start_y)
                forward_backward_list.append(self.AS.attitude[1])
                route_x.reverse()
                route_y.reverse()
                forward_backward_list.reverse()
                break
            new_node = node_calculated[last_node_id]
            route_x.append(new_node.x)
            route_y.append(new_node.y)
            forward_backward_list.append(new_node.attitude[1])
            last_node_id = new_node.last_node_id
        self.AS.route_x, self.AS.route_y, self.AS.forward_backward_record = route_x, route_y, forward_backward_list



class AstarPathPlanning_sim_v3(AstarPathPlanning_sim_v2):
    def __init__(self, SharedVariable):
        self.SV = SharedVariable
        super().__init__(self.SV)

        self.open_path_dictionary = dict()
        self.close_path_dictionary = dict()
        self.path_id = 0

    class Path:
        def __init__(self, path_id, node_calculated, new_node):
            self.path_id, self.node_calculated, self.new_node = path_id, node_calculated, new_node


    def reset(self):
        super().reset()
        self.open_path_dictionary = dict()
        self.close_path_dictionary = dict()
        self.path_id = 0

    def calculate_cost(self, node, movement, motion):
        node.cost = round(np.hypot(node.x - self.AS.end_x, node.y - self.AS.end_y), 2) + len(self.node_calculated)*self.AS.step_unit
        node.cost += 0 if motion[1] == 'forward' else 100*self.AS.step_unit
        return node.cost

    def repeated_path(self):
        repeated_path_dict = {}
        for key, path in self.open_path_dictionary.items():
            repeated_path_dict = 1

    def planning(self):
        start_time = time.time()
        # start point
        self.reset()

        current_node = self.Node(
            self.AS.start_x,
            self.AS.start_y,
            0,
            str(self.AS.start_x) + ',' + str(self.AS.start_y),
            self.AS.attitude
        )
        target_node = self.Node(
            self.AS.end_x,
            self.AS.end_y,
            0,
            "",
            self.AS.attitude
        )

        reach_target = False
        
        # self.node_use_for_calculation[str(current_node.x) + ',' + str(current_node.y)] = current_node
        current_path = self.Path(self.path_id, self.node_calculated, current_node)
        self.open_path_dictionary[self.path_id] = current_path

        while not reach_target:

            if len(self.open_path_dictionary) == 0:
                self.calculate_path(current_node, self.node_calculated)
                print("No route to go to target")
                break
            # Open set, which is used for calculation, find lowest cost node for calculation


            current_path_id = min(self.open_path_dictionary, key=lambda i: self.open_path_dictionary[i].new_node.cost)
            current_path = self.open_path_dictionary[current_path_id]


            current_node = current_path.new_node
            current_node_id = str(current_node.x) + ',' + str(current_node.y)
            self.node_calculated = current_path.node_calculated

            # Remove this node from open_set
            # del self.node_use_for_calculation[current_node_id]
            del self.open_path_dictionary[current_path_id]

            # Then add it to close_set, which is the one that already calculated
            # self.node_calculated[current_node_id] = current_node
            self.close_path_dictionary[current_path_id] = current_path
            current_path.node_calculated[current_node_id] = current_node

            if np.hypot(current_node.x - self.AS.end_x, current_node.y - self.AS.end_y) <= self.AS.step_unit:
                print("Target reached")
                target_node.last_node_id = str(current_node.x) + ',' + str(current_node.y)
                target_node.attitude = current_node.attitude
                self.calculate_path(target_node, self.node_calculated)
                reach_target = True
                break
            # print(current_node_id)
            self._show_progress(current_node)

            # Node calculation
            for motion in self.motion:
                movement = self.motion_dict[(current_node.attitude[0]+motion[0])%360]
                new_node = self.Node(
                    current_node.x + self.AS.step_unit * movement[0],
                    current_node.y + self.AS.step_unit * movement[1],
                    current_node.cost,
                    current_node_id,
                    [(current_node.attitude[0]+motion[0])%360, motion[1]]
                )
                if self.node_invaild(new_node):
                    continue
                if self.repeated_node(new_node):
                    continue
                new_node.cost = self.calculate_cost(new_node, movement, motion)
                # if new_node.attitude == current_node.attitude:
                #     self.open_path_dictionary[current_path_id] = self.Path(self.path_id, current_path.node_calculated, new_node)
                # else:
                #     self.path_id += 1
                #     self.open_path_dictionary[self.path_id] = self.Path(self.path_id, current_path.node_calculated, new_node)
                self.path_id += 1
                self.open_path_dictionary[self.path_id] = self.Path(self.path_id, current_path.node_calculated, new_node)
                # self._show_progress(new_node, current_path.node_calculated)

        end_time = time.time()
        self.AS.astar_planning_time = end_time - start_time


