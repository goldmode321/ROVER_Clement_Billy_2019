import numpy as np


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
        self.node_use_for_calculation = dict() # Open set


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
            if len(self.node_use_for_calculation) == 0:
                self.calculate_path(current_node, self.node_calculated)
                print("No route to go to target")
                break
            # Open set, which is used for calculation, find lowest cost node for calculation
            current_node_id = min(self.node_use_for_calculation, key=lambda i: self.node_use_for_calculation[i].cost)
            current_node = self.node_use_for_calculation[current_node_id]

            if np.hypot(current_node.x - self.SV.MAP.end_x, current_node.y - self.SV.MAP.end_y) <= self.SV.step_unit:
                print("Target reached")
                self.calculate_path(current_node, self.node_calculated)
                reach_target = True
                break

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
                else: 
                    # If not new node, see if it is the best path for now (lower cost)
                    if self.node_use_for_calculation[str(new_node.x) + ',' + str(new_node.y)].cost > new_node.cost:
                        self.node_use_for_calculation[str(new_node.x) + ',' + str(new_node.y)] = new_node


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
        self.SV.route_x, self.SV.route_y = route_x, route_y

