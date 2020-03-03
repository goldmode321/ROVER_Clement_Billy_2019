import numpy as np

class Map_sim:
    def __init__(self, SharedVariable, name='maze1'):
        self.SV = SharedVariable
        self.map_name = name
        self.SV.AS.start_x = 0
        self.SV.AS.start_y = 0
        self.SV.GOBS.global_obstacle_x = np.array([])
        self.SV.GOBS.global_obstacle_y = np.array([])
        self.SV.GOBS.global_obstacle = np.array([])
        self.obs_dict = {'maze1':self.maze1, 'maze2':self.maze2}
        self.generate_map()


    def generate_map(self):
        self.obs_dict[self.map_name]()

    def maze1(self):
        self.map_width = 2000
        self.map_length = 2000
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

        self.SV.GOBS.global_obstacle_x = self.SV.GOBS.global_obstacle_y = np.array([])

        for i, j in zip(obs_x, obs_y):
            self.SV.GOBS.global_obstacle_x = np.append(self.SV.GOBS.global_obstacle_x, i)
            self.SV.GOBS.global_obstacle_y = np.append(self.SV.GOBS.global_obstacle_y, j)
        self.SV.AS.start_x = self.map_length/10
        self.SV.AS.start_y = self.map_width*9/10
        self.SV.AS.end_x = self.map_length*9/10
        self.SV.AS.end_y = self.map_width/10
        self.SV.GOBS.global_obstacle = np.stack((self.SV.GOBS.global_obstacle_x, self.SV.GOBS.global_obstacle_y), axis=1)


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

        wall1_x = np.linspace(self.map_length*1/5, self.map_length*4/5, int(self.map_length*3/5), dtype='int')
        wall1_y = np.zeros(int(self.map_length*3/5), dtype='int') + int(self.map_width/5)

        wall2_x = np.linspace(self.map_length*1/5, self.map_length*4/5, int(self.map_length*3/5), dtype='int')
        wall2_y = np.zeros(int(self.map_length*3/5), dtype='int') + int(self.map_width*4/5)

        wall3_x = np.zeros(int(self.map_width*3/5), dtype='int') + int(self.map_length*3/5)
        wall3_y = np.linspace(self.map_width*1/5, self.map_width*4/5, int(self.map_width*3/5), dtype='int')

        obs_x = [wall_bottom_x, wall_top_x, wall_left_x, wall_right_x, wall1_x, wall2_x, wall3_x]
        obs_y = [wall_bottom_y, wall_top_y, wall_left_y, wall_right_y, wall1_y, wall2_y, wall3_y]

        self.SV.GOBS.global_obstacle_x = self.SV.GOBS.global_obstacle_y = np.array([])

        for i, j in zip(obs_x, obs_y):
            self.SV.GOBS.global_obstacle_x = np.append(self.SV.GOBS.global_obstacle_x, i)
            self.SV.GOBS.global_obstacle_y = np.append(self.SV.GOBS.global_obstacle_y, j)
        self.SV.AS.start_x = self.map_length/10
        self.SV.AS.start_y = self.map_width*1/2
        self.SV.AS.end_x = self.map_length*9/10
        self.SV.AS.end_y = self.map_width/2
        self.SV.GOBS.global_obstacle = np.stack((self.SV.GOBS.global_obstacle_x, self.SV.GOBS.global_obstacle_y), axis=1)