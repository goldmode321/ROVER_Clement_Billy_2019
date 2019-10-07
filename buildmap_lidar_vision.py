import rplidar
import xmlrpc.client
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import multiprocessing
import threading
import math
import time
import queue
import traceback
import numpy



def main():
    vision_module =  xmlrpc.client.ServerProxy("http://192.168.5.101:8080")
    if vision_module.get_status()[0] != 1:
        print("Vision module auto reset")
        vision_module.reset()
        time.sleep(3)
    choose_vision_mode_run = True
    choose_vision_mapid_run = True
    main_run = False

    while choose_vision_mode_run:
        try:
            vision_mode = input("Enter vision mode (bm - build map, um - use map, kbm - keep building map) : ")
            if vision_mode is None:
                pass
            elif vision_mode in ['bm', 'um', 'kbm']:
                choose_vision_mode_run = False
        except KeyboardInterrupt:
            print("Shutdown by KeyboardInterrupt")
            choose_vision_mapid_run = False
            choose_vision_mode_run = False

    while choose_vision_mapid_run:
        try:
            vision_mapid = int(input("Enter vision mapid (Please use integer) : "))
            if vision_mode == 'bm':
                vision_set_start_message = vision_module.set_start(1, [vision_mapid])
            elif vision_mode == 'um':
                vision_set_start_message = vision_module.set_start(0, [vision_mapid])
            else:
                vision_set_start_message = vision_module.set_start(2, [vision_mapid])
            print(vision_set_start_message)
            if vision_set_start_message[0] == -5:
                print("MapID : {} is not exist")
            elif vision_set_start_message[0] == 0:
                print(vision_module.get_status())
                choose_vision_mapid_run = False
                main_run = True
        except ValueError:
            pass
        except KeyboardInterrupt:
            print("Shutdown by KeyboardInterrupt")
            choose_vision_mapid_run = False

    if main_run:
        # pool = multiprocessing.Pool(multiprocessing.cpu_count() - 1)
        # pool = multiprocessing.Pool(1)
        lidar_data = multiprocessing.Queue()
        get_lidar_data_run = multiprocessing.Queue()
        vision_data = multiprocessing.Queue()
        get_vision_data_run = multiprocessing.Queue()
        polar_plot_run = multiprocessing.Queue()

        lidar_process = multiprocessing.Process(target=get_lidar_data, args=(lidar_data, get_lidar_data_run))
        lidar_process.start()
        print("Lidar process on")

        vision_process = multiprocessing.Process(target=get_vision_data, args=(vision_data, get_vision_data_run, vision_mode, vision_mapid))
        vision_process.start()
        print("Vision process on")

        polar_plot_process = multiprocessing.Process(target=polar_plot, args=(lidar_data, vision_data, polar_plot_run))
        polar_plot_process.start()
        print("Polar_plot_process on")

        # polar_plot(lidar_data, vision_data, polar_plot_run)
        # time.sleep(60)
        while main_run:
            try:
                input("Press any key to shutdown, try not to use KeyboardInterrupt")
                main_run = False
            except KeyboardInterrupt:
                main_run = False
        get_lidar_data_run.put(False)
        get_vision_data_run.put(False)
        polar_plot_run.put(False)

        lidar_process.join()
        vision_process.join()
        polar_plot_process.join()
        # pool.close()
        # pool.join()


def get_lidar_data(lidar_data, get_lidar_data_run):
    lidar = rplidar.RPLidar("COM12")
    time.sleep(1)
    run = True
    # print(lidar.get_info())
    # print(run)
    while run:
        try:
            for data in lidar.iter_scans(300):
                # print(data)

                # make sure lidar_data queue is always the lastest lidar data
                # if there is data left, take out the rest. If no data left,
                # just input lidar data.
                if lidar_data.qsize() > 0:
                    # print("size > 0")
                    lidar_data.get()
                    lidar_data.put(data)
                else:
                    lidar_data.put(data)
                # Check if queue, get_lidar_data_run has an input of Flase.
                try:
                    run = get_lidar_data_run.get(False)
                    if not run:
                        lidar.stop()
                        print("Lidar stop")
                        break
                except queue.Empty:
                    pass

        except KeyboardInterrupt:
            lidar.stop()
            print("Lidar stopped by KeyboardInterrupt")
        except:
            # print("OMG")
            traceback.print_exc()
            lidar.stop()
        finally:
            run = False
            lidar.disconnect()
            print("Lidar disconnect")


def get_vision_data(vision_data, get_vision_data_run, vision_mode, vision_mapid):
    run = False
    vision_module =  xmlrpc.client.ServerProxy("http://192.168.5.101:8080")
    if vision_module.alive() == [0, 'Alive']:
        run = True
    else:
        print("Vision module is not alive, program stop")

    while run:
        try:
            # status = vision_module.get_status()
            pose = vision_module.get_pose()
            # make sure lidar_data queue is always the lastest vision data
            # if there is data left, take out the rest. If no data left,
            # just input lidar data.
            if vision_data.qsize() > 0:
                vision_data.get()
                vision_data.put([pose[3], pose[4], pose[5], pose[0]])
            else:
                vision_data.put([pose[3], pose[4], pose[5], pose[0]])
            # print(pose)
            time.sleep(0.1)
            # vision_x = pose[3]
            # vision_y = pose[4]
            # vision_theta = pose[5]
            try:
                run = get_vision_data_run.get(False)
                if not run:
                    if vision_mode in ['bm', 'kbm']:
                        print("Vision module saving map {}, please do not interrupt".format(vision_mapid))
                        print(vision_module.save_db())
                    print("Vision module reseting, please wait 3 second")
                    print(vision_module.reset())
                    time.sleep(3)
                    print("Vision stop")
            except queue.Empty:
                pass
        except KeyboardInterrupt:
            run = False
            print("KeyboardInterrupt Vision module reseting, please wait 3 second")
            print(vision_module.reset())
            time.sleep(3)
            print("Vision stop")



def polar_plot(lidar_data, vision_data, polar_plot_run):
    run = True
    figure1 = plt.figure(1, figsize=[15,6])
    # figure2 = plt.figure(2)
    # [0.1, 0.1, 0.8, 0.8] make polar plt show in center of this figure.
    # polar_plot = figure1.add_axes([0.1, 0.1, 0.8, 0.8], projection='polar')
    # global_map_plot = figure2.add_axes([0, 0, 0.8, 0.8], projection='rectilinear')
    polar_plot = plt.subplot(1, 2, 1, projection = 'polar')
    global_map_plot = plt.subplot(1, 2, 2)
    plt.pause(0.0001)
    while run:
        try:
            # If block is positive num, then block until item receive or timeout second
            li_data = lidar_data.get(block=1, timeout=1)
            vi_data = vision_data.get(block=1, timeout=1)
            print(vi_data)

            lidar_angle = [math.radians(-i[1]) + 0.5*math.pi for i in li_data]
            lidar_radius = [i[2] for i in li_data]
            polar_plot.plot(lidar_angle, lidar_radius, '.')
            polar_plot.set_ylim(0, 1000)
            
            if vi_data[3] == 0:
                vision_angle_radian = math.radians(vi_data[2])
                local_obstacle_x = numpy.cos(numpy.array(lidar_angle)-vision_angle_radian)*numpy.array(lidar_radius) + vi_data[0]
                local_obstacle_y = numpy.sin(numpy.array(lidar_angle)-vision_angle_radian)*numpy.array(lidar_radius) + vi_data[1]
                global_map_plot.plot(local_obstacle_x, local_obstacle_y, 'bo')
                global_map_plot.plot(vi_data[0], vi_data[1], 'ro')
                global_map_plot.arrow(vi_data[0], vi_data[1], 200*math.cos(-vision_angle_radian+0.5*math.pi),\
                     200*math.sin(-vision_angle_radian+0.5*math.pi), width=30)
                # print(200*math.cos(vi_data[2]), 200*math.sin(vi_data[2]))
                # print(vi_data[2])

            
            
            plt.pause(0.001)
            polar_plot.cla()
            if vi_data[3] ==0:
                global_map_plot.cla()

            try:
                run = polar_plot_run.get(False)
                if not run:
                    print("Polar plot stop")
            except queue.Empty:
                pass
        except queue.Empty:
            pass
        except KeyboardInterrupt:
            run = False

class BuildMap():
    def __init__(self):
        self.global_obstacle = []
        self.vision_data = []
        self.vision_status = []
        self.locol_obstacle = []
        self.get_lidar_data_run = False
        self.main_run = False

        self.vision_module = xmlrpc.client.ServerProxy("http://192.168.5.101:8080")
        self.lidar = None
        self.lidar_data = []
        self.lidar_angle = []
        self.lidar_radius = []
        self.vision_x = 0
        self.vision_y = 0

        self.pool = multiprocessing.Pool(processes=3)
        # self.lidar_data = multiprocessing.Array('d', [])
        # self.vision_x = multiprocessing.Value('d', 0)
        # self.vision_x = multiprocessing.Value('d', 0)
        # self.lidar_output = multiprocessing.Queue()


        self.get_lidar_data_in_background()
        self.plotting()
        # self.plotting()


    def plotting(self):
        self.main_run = True
        while self.main_run:
            try:
                self.lidar_angle = [ math.radians(i[1]) for i in self.lidar_data]
                self.lidar_radius = [ i[2] for i in self.lidar_data]
                plt.polar(self.lidar_angle, self.lidar_radius, 'ro')
                plt.pause(0.001)
                plt.cla()
            except:
                self.main_run = False
                self.get_lidar_data_run = False



    # def get_lidar_data_in_background(self):
    #     self.lidar = rplidar.RPLidar("COM12")
    #     self.get_lidar_data_run = True
    #     self.pool.apply(self.get_lidar_data, args = None)
    #     self.pool.join()

    def get_lidar_data_in_background(self):
        self.lidar_data = multiprocessing.Queue()
        self.get_lidar_data_run = multiprocessing.Queue()
        self.get_lidar_data_run.put(True)
        self.lidar_process = multiprocessing.Process(target = self.get_lidar_data, args =(self.lidar_data, self.get_lidar_data_run))
        self.lidar_process.start()

    def get_lidar_data(self, lidar_data, get_lidar_data_run):
        lidar = rplidar.RPLidar("COM12")
        run = True
        # print(lidar.get_info())
        # print(run)
        while run:
            try:
                for data in lidar.iter_scans(300):
                    print(data)
                    # make sure lidar_data queue is always the lastest lidar data
                    # if there is data left, take out the rest. If no data left,
                    # just input lidar data.
                    if lidar_data.qsize() > 0:
                        # print("size > 0")
                        lidar_data.get()
                        lidar_data.put(data)
                    else:
                        lidar_data.put(data)
                    # Check if queue, get_lidar_data_run has an input of Flase.
                    try:
                        run = get_lidar_data_run.get(False)
                        if not run:
                            lidar.stop()
                            print("Lidar stop")
                            break
                    except queue.Empty:
                        pass

            except KeyboardInterrupt:
                lidar.stop()
                print("Lidar stopped by KeyboardInterrupt")
            except:
                # print("OMG")
                traceback.print_exc()
                lidar.stop()
            finally:
                run = False
                lidar.disconnect()
                print("Lidar disconnect")





if __name__ == '__main__':
    main()
