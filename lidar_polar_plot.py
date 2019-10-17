import rplidar
import xmlrpc.client as xc
import matplotlib.pyplot as plt
import multiprocessing
import threading
import math
import time
import queue
import traceback
import rover_socket



# def get_lidar_data(lidar_data, get_lidar_data_run):
#     lidar = rplidar.RPLidar("COM12")
#     run = True
#     while run:
#         try:
#             # input of iter_scans is the max_buffer_size store in serial port.
#             # If computer is not fast enough, the buffer will accamulate, and clear when 
#             # max_buffer_size reach. Lower max_buffer_size should avoid lag for data.
#             # But short of max_buffer_size also cause lag
#             for data in lidar.iter_scans(300):

#                 # make sure lidar_data queue is always the lastest lidar data
#                 # if there is data left, take out the rest. If no data left,
#                 # just input lidar data.
#                 if lidar_data.qsize() > 0:
#                     lidar_data.get()
#                     lidar_data.put(data)
#                 else:
#                     lidar_data.put(data)
#                 # Check if queue, get_lidar_data_run has an input of Flase.
#                 try:
#                     run = get_lidar_data_run.get(False)
#                     if not run:
#                         lidar.stop()
#                         print("Lidar stop")
#                         break
#                 except queue.Empty:
#                     pass

#         except KeyboardInterrupt:
#             lidar.stop()
#             print("Lidar stopped by KeyboardInterrupt")
#         except:
#             traceback.print_exc()
#             lidar.stop()
#         finally:
#             run = False
#             lidar.disconnect()
#             print("Lidar disconnect")


def get_lidar_data(lidar_data, get_lidar_data_run):
    lidar = rover_socket.UDP_client(50010, 0, '192.168.5.2')
    run = True
    while run:
        try:
            lidar.send_list([1])
            data = lidar.recv_list(65535)
            if data is not None:
                if lidar_data.qsize() > 0:
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
            lidar.close
            print("Lidar stopped by KeyboardInterrupt")
        except:
            traceback.print_exc()
            lidar.close()
        finally:
            run = False
            print("Lidar disconnect")



def plotting():
    run = True
    figure1 = plt.figure(1)
    # [0.1, 0.1, 0.8, 0.8] make polar plt show in center of this figure.
    polar_plot = figure1.add_axes([0.1, 0.1, 0.8, 0.8], projection='polar')
    while run:
        try:
            # If block is positive num, then block until item receive or timeout second
            data = lidar_data.get(block=1, timeout=1)
            
            lidar_angle = [math.radians(-i[1]) + 0.5*math.pi for i in data]
            lidar_radius = [i[2] for i in data]
            polar_plot.plot(lidar_angle, lidar_radius, 'ro')
            plt.ylim(0, 2000) # Polar plot radius
            plt.pause(0.001)
            polar_plot.cla()
        except queue.Empty:
            pass
        except KeyboardInterrupt:
            run = False




if __name__ == '__main__':
    # BuildMap()

    lidar_data = multiprocessing.Queue()
    get_lidar_data_run = multiprocessing.Queue()
    get_lidar_data_run.put(True)

    lidar_process = multiprocessing.Process(target = get_lidar_data, args = (lidar_data, get_lidar_data_run))
    lidar_process.start()
    print("lidar_process on")
    plotting()
    get_lidar_data_run.put(False)
    lidar_process.join()