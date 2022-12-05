import numpy as np
from collections import deque
import numpy as np

def get_distance_from_point_to_line(point, line_point1, line_point2):
    #对于两点坐标为同一点时,返回点与点的距离
    if np.linalg.norm(np.array([line_point1[0]-line_point2[0], line_point1[1]-line_point2[1]])) < 0.01:
        point_array = np.array(point[:2])
        point1_array = np.array(line_point1[:2])
        return np.linalg.norm(point_array -point1_array )
    #计算直线的三个参数
    A = line_point2[1] - line_point1[1]
    B = line_point1[0] - line_point2[0]
    C = (line_point1[1] - line_point2[1]) * line_point1[0] + \
        (line_point2[0] - line_point1[0]) * line_point1[1]
    #根据点到直线的距离公式计算距离
    distance = (A * point[0] + B * point[1] + C) / np.sqrt(A**2 + B**2)
    return distance

class Controller2D(object):
    def __init__(self, waypoints):
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi
        self.v_err_previous          = 0

        self.v_err_i = deque(maxlen = 10)

    def update_values(self, x, y, yaw, speed):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed

    def update_desired_speed(self):
        wp = self._waypoints.copy()
        wp[:, 0] -= self._current_x
        wp[:, 1] -= self._current_y
        dist = wp[:, 0] ** 2 + wp[:, 1] ** 2
        min_idx = dist.argmin()
        self._desired_speed = self._waypoints[min_idx][2]
        self._ahead_waypoints = self._waypoints[min_idx:]
       # if min_idx > len(self._waypoints) - 4:
       #     self._desired_speed = 0


    def update_waypoints(self, new_waypoints):
        #print('update_waypoints', new_waypoints[:2], len(new_waypoints))
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        waypoints       = self._ahead_waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0
       
        ######################################################
        ######################################################
        # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
        ######################################################
        ######################################################

        v_err = v_desired - v
        v_err_d = v_err - self.v_err_previous

        # self.v_err_i = deque(maxlen = 10)
        self.v_err_i.append(np.clip(v_err, -0.2, 0.2))
        v_err_i = sum(self.v_err_i)

        Kp = 0.6
        Ki = 0.1
        Kd = +0.3
        
        acc_delta = Kp * v_err + Ki * v_err_i  + Kd * v_err_d

        throttle_output = 0
        brake_output    = 0
        feed_forward = np.log(v_desired + 1) / 3.6

        acc = feed_forward + acc_delta
        if acc > 0:
            throttle_output = acc
        else:
            brake_output = -acc

        self.v_err_previous = v_err

        ######################################################
        ######################################################
        # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
        ######################################################
        ######################################################
        L=3.0
        front_center_x = x + np.cos(yaw) * L / 2
        front_center_y = y + np.sin(yaw) * L / 2
        
        K_v = 0.5
        #ld = max(3*L, K_v*v)

        wpt = waypoints.copy()
        wpt[:, 0] -= front_center_x
        wpt[:, 1] -= front_center_y
        dist = np.abs(wpt[:, 0]**2 + wpt[:, 1] ** 2)# - ld ** 2)
        nearest_idx = dist.argmin()
        if nearest_idx == len(waypoints) -1:
            nearest_idx -= 1

        nearest_next_idx = (nearest_idx + 1) % (len(waypoints))

        ref_point_x, ref_point_y, _ = waypoints[nearest_idx]
        ref_next_point_x, ref_next_point_y, _ = waypoints[nearest_next_idx]

        reference_line_yaw = np.arctan2(ref_next_point_y - ref_point_y, ref_next_point_x - ref_point_x)

        cross_track_error = get_distance_from_point_to_line([front_center_x, front_center_y], 
                                                             waypoints[nearest_idx],
                                                             waypoints[nearest_next_idx])

        #heading error
        psai = reference_line_yaw - yaw
        if psai > np.pi:
            psai -= 2 * np.pi
        elif psai < -np.pi:
            psai += 2 * np.pi

        Ks = 8 #4
        Ke = 10.0  #7.0 # 0.5
        steer_output = psai + np.arctan2(Ke * cross_track_error, Ks + v)

        ######################################################
        # SET CONTROLS OUTPUT
        ######################################################
        self.set_throttle(throttle_output)  # in percent (0 to 1)
        self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
        self.set_brake(brake_output)        # in percent (0 to 1)

        self.v_previous = v  # Store forward speed to be used in next step
