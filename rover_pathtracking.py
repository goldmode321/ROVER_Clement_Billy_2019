import numpy as np
import rover_stanley_controller
import threading
import time
import traceback



class PathTracking:
    '''Path tracking method = ["Stanley"]
        simplified as p_t
    '''
    def __init__(self, SharedVariables, method="stanley"):
        self.p_t_method = {
            "stanley": rover_stanley_controller.StanleyController,
            "stanley_sim": rover_stanley_controller.StanleyController_sim,
        }
        self.SV = SharedVariables
        try:
            self.p_t = self.p_t_method[method](self.SV)
            self.p_t_thread_manual = self.PathTrackingThreadManual(self.SV, self.p_t)
            self.p_t_thread_auto = self.PathTrackingThreadAuto(self.SV, self.p_t)
            self.p_t_thread_sim = self.PathTrackingThreadSim(self.SV, self.p_t)
            self.SV.ROV.path_tracking_ready = True
            print("Path Tracking is ready !!")
        except:
            traceback.print_exc()
            self.SV.ROV.path_tracking_ready = False
            print("Path Tracking fail initiating")
    
    def manualControl(self):
        if self.p_t_thread_auto.run_flag:
            self.p_t_thread_auto.stop()

        if self.p_t_thread_manual.run_flag:
            print("Path Tracking Manual Mode is already ran")
        else:
            self.p_t_thread_manual = self.PathTrackingThreadManual(self.SV, self.p_t)
            self.p_t_thread_manual.start()

    def autoControl(self):
        if self.p_t_thread_manual.run_flag:
            self.p_t_thread_manual.stop()

        if self.p_t_thread_auto.run_flag:
            print("Path Tracking Auto Mode is already ran")
        else:
            self.p_t_thread_auto = self.PathTrackingThreadAuto(self.SV, self.p_t)
            self.p_t_thread_auto.start()


    def switchMethod(self, method):
        if self.p_t_thread_manual.run_flag or self.p_t_thread_auto.run_flag:
            print("Please stop path tracking first before switch method")
        else:
            try:
                self.p_t = self.p_t_method[method](self.SV)
            except:
                print("name error")

    def stop(self):
        if self.p_t_thread_auto.run_flag:
            self.p_t_thread_auto.stop()
        elif self.p_t_thread_manual.run_flag:
            self.p_t_thread_manual.stop()
        elif self.p_t_thread_sim.run_flag:
            self.p_t_thread_sim.stop()

    def simulate(self):
        self.p_t_thread_sim.start()

    class PathTrackingThreadManual(threading.Thread):
        def __init__(self, SharedVariables, path_tracking):
            super().__init__(daemon=True)
            self.SV = SharedVariables
            self.CC = self.SV.CC
            self.PT = self.SV.PT
            self.path_tracking = path_tracking
            self.run_flag = False

        def stop(self):
            self.run_flag = False
            self.join()

        def resetRecord(self):
            self.PT.target_index = 0
            self.PT.tracking_route_x = []
            self.PT.tracking_route_y = []
            self.PT.tracking_steering_rad = []
            self.PT.tracking_steering_deg = []
            self.PT.tracking_target_steering_deg = []
            self.PT.tracking_real_steer = []
            self.PT.tracking_steering_deg = []
            self.PT.tracking_yaw = []
            self.PT.tracking_theta_e_deg = []

        def record(self):
            if len(self.PT.tracking_route_x) > 1000:
                del self.PT.tracking_route_x[0], self.PT.tracking_route_y[0], \
                    self.PT.tracking_target_steering_deg[0], self.PT.tracking_yaw[0], \
                        self.PT.tracking_real_steer[0], self.PT.tracking_steering_deg[0], \
                            self.PT.tracking_theta_e_deg[0]

                self.PT.tracking_route_x.append(self.PT.current_x)
                self.PT.tracking_route_y.append(self.PT.current_y)
                self.PT.tracking_target_steering_deg.append(self.PT.steering_target_angle_deg)
                self.PT.tracking_yaw.append(self.PT.current_yaw)
                self.PT.tracking_real_steer.append(self.PT.real_steer_deg)
                self.PT.tracking_steering_deg.append(self.PT.steering_angle_deg)
                self.PT.tracking_theta_e_deg.append(self.PT.theta_e_deg)
            else:
                self.PT.tracking_route_x.append(self.PT.current_x)
                self.PT.tracking_route_y.append(self.PT.current_y)
                self.PT.tracking_target_steering_deg.append(self.PT.steering_target_angle_deg)
                self.PT.tracking_yaw.append(self.PT.current_yaw)
                self.PT.tracking_real_steer.append(self.PT.real_steer_deg)
                self.PT.tracking_steering_deg.append(self.PT.steering_angle_deg)
                self.PT.tracking_theta_e_deg.append(self.PT.theta_e_deg)

        def run(self):
            self.run_flag = True
            self.path_tracking.updateState_real()
            self.resetRecord()
            while self.run_flag:
                self.path_tracking.next_target(manual_mode=True)
                self.path_tracking.calculateCommand()
                self.path_tracking.updateState_real()
                self.CC.car_control_steer = 400 + int(self.PT.real_steer_deg*85/30)
                # self.record()
                time.sleep(0.1)



    class PathTrackingThreadAuto(PathTrackingThreadManual):
        def __init__(self, SharedVariables, path_tracking):
            self.SV = SharedVariables
            self.CC = self.SV.CC
            self.PT = self.SV.PT
            self.path_tracking = path_tracking
            super().__init__(self.SV, self.path_tracking)

        def run(self):
            self.run_flag = True
            self.path_tracking.updateState_real()
            self.resetRecord()
            while self.run_flag:
                self.path_tracking.next_target()
                self.path_tracking.calculateCommand()
                self.path_tracking.updateState_real()
                self.CC.car_control_steer = 405 - int(self.PT.real_steer_deg*85/30)
                # self.record()

                time.sleep(0.1)
                if self.PT.target_position == [self.SV.CF.fitted_route_x[-1], self.SV.CF.fitted_route_y[-1]]:
                    self.run_flag = False

    class PathTrackingThreadSim(PathTrackingThreadManual):
        def __init__(self, SharedVariables, path_tracking):
            self.SV = SharedVariables
            self.PT = self.SV.PT
            self.path_tracking = path_tracking
            super().__init__(self.SV, self.path_tracking)

        def run(self):
            self.run_flag = True
            self.resetRecord()
            while self.run_flag:
                self.path_tracking.next_target()
                self.path_tracking.calculateCommand()
                self.path_tracking.update_state()
                # self.record()

                time.sleep(0.1)
                if self.PT.target_position == [self.SV.CF.fitted_route_x[-1], self.SV.CF.fitted_route_y[-1]]:
                    self.run_flag = False
