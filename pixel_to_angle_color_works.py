import math
import pyrealsense2 as rs
import cv2
import numpy as np

import serial
import time
from PyQt5.QtCore import QTimer

# command delimiter for the serial communication to Arduino
SERIAL_MSG_DELIMITER = 200


class ConnectionError(RuntimeError):
    pass


class Arm:
    def __init__(self, port, baud=9600, update_freq=30):
        try:
            self._arduino = serial.Serial(port, baud, timeout=1)
        except serial.SerialException as e:
            raise ConnectionError(e)

        time.sleep(0.01)  # TODO: is this needed?

        self._encoded_msg = None
        self._update_freq = update_freq

        # update at a set frequency
        t = QTimer()
        t.setInterval(1000 / self._update_freq)
        t.timeout.connect(self.send_message)
        self._timer = t
        t.start()

    # @param servos list of three int values between 0 and 180
    def set_servo_values(self, servos):
        if len(servos) != 4:
            raise RuntimeError("Invalid number of servos")
        self._encoded_msg = [SERIAL_MSG_DELIMITER] + list(servos)
        self.send_message()

    def send_message(self):
        print(self._encoded_msg)
        if self._encoded_msg:
            self._arduino.write(self._encoded_msg)


try:
    # define ardunio port
    arm = Arm(port="COM6")
    connected = arm != None
    # angle range( 25 - 165, 55 - 140, 55 - 145)
    arm.set_servo_values([90, 100, 80, 110])

except ConnectionError as e:
    print("failed to connect to Arduino")


def law_of_cosines(a, b, c):
    """ Performs the Law of Cosines on the given side lengths """
    return math.degrees(math.acos((c ** 2 - b ** 2 - a ** 2) / (-2.0 * a * b)))


# if we return (-100, -100) it means that the arm shouldn't move
def get_rot_angle(x, y):
    """ Returns the angles necessary to reach the given coordinate point """

    # take gripper in account since the triangle is calculated till the screw
    x -= 0.10

    # convert to mm
    x *= 1000
    y *= 1000

    print(str(x) + "," + str(y))
    distance_to_goal = math.sqrt(x * x + y * y)
    print("distance_to_goal" + str(distance_to_goal))
    if distance_to_goal == 0:
        print("1")
        return (-100, -100)
    angle_from_horizontal = math.degrees(math.asin(y / distance_to_goal))
    print(angle_from_horizontal)
    try:
        # use low of cosines to calculate angles of arm
        angle_a = law_of_cosines(135, distance_to_goal, 148)
        angle_b = law_of_cosines(135, 148, distance_to_goal)

        #   /b\
        #  /    \
        # /a

        servo_angle_a = 90 - (angle_a + angle_from_horizontal)
        servo_angle_b = angle_b - servo_angle_a + angle_from_horizontal
        print("servo_angle_a = " + str(servo_angle_a))
        print("servo_angle_b = " + str(servo_angle_b))

        # convert angle to Ardunio calibration
        servo_a_angle = servo_angle_a + 100
        servo_b_angle = servo_angle_b + 42
        return (servo_a_angle, servo_b_angle)
    except ValueError:
        print("3")
        return (-100, -100)


x_g = 0
y_g = 0
x_dest = 0
y_dest = 0
move_now = 0

def mouse_drawing(event, x, y, flags, params):
    global x_g, y_g, z_g, x_dest, y_dest, move_now
    if event == cv2.EVENT_MOUSEMOVE:
        x_g = x
        y_g = y

    elif event == cv2.EVENT_LBUTTONDBLCLK:
        x_dest = x
        y_dest = y
        move_now = 1



points3d = [0, 0, 0]

depth_min = 0.01  # meter
depth_max = 1.0  # meter

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)

# Start streaming
profile = pipeline.start(config)
good_to_go = 1
count = 0
try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        infra_frame = frames.get_infrared_frame(1)

        depth_intrin = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        color_intrin = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        depth_to_color_extrin = depth_frame.profile.as_video_stream_profile().get_extrinsics_to(
            profile.get_stream(rs.stream.color))
        color_to_depth_extrin = color_frame.profile.as_video_stream_profile().get_extrinsics_to(
            profile.get_stream(rs.stream.depth))

        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        infra_image = np.asanyarray(infra_frame.get_data())

        # Depth scale - units of the values inside a depth frame, i.e how to convert the value to units of 1 meter
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        images = color_image
        depth = frames.get_depth_frame()

        # Show images
        font = cv2.FONT_HERSHEY_COMPLEX
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)

        # get mouse pointer pixel x, y
        cv2.setMouseCallback('RealSense', mouse_drawing)

        pointer3d = [0,0,0]

        # delay after each click
        if (not good_to_go):
            count += 1
            if (count == 100):
                arm.set_servo_values([90, 100, 80, 110])
                good_to_go = 1
                count = 0

        if (move_now & good_to_go):
            color_pixel = [x_dest, y_dest]


            # convert color pixel to depth pixel
            depth_pixel = rs.rs2_project_color_pixel_to_depth_pixel(
                depth_frame.get_data(), depth_scale,
                depth_min, depth_max,
                depth_intrin, color_intrin, depth_to_color_extrin, color_to_depth_extrin, color_pixel)

            depth_pixel = [int(round(elem)) for elem in depth_pixel]
            x_d = depth_pixel[0]
            y_d = depth_pixel[1]

            dest = 0
            try:
                dest = depth_frame.get_distance(x_d, y_d)
            except RuntimeError:
                print("out of range")

            # get pixel x, y, z coordinates
            points3d = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, dest)

            # transfer from camera axis to arm axis
            points3d[0] -= 0.02 #infra
            points3d[1] += 0.04 #infra
            points3d[2] += 0.06 #infra
            points3d = [round(elem, 2) for elem in points3d]
            print(points3d)

            # calc base angle
            alpha = (180 / math.pi) * math.atan2(points3d[2], points3d[0])
            base_angle = 180 - alpha

            # multiple by ratio to get servo angle
            factor = 2
            if (base_angle < 90):
                angle_fix = factor * (90 - base_angle)
                base_angle = 90 - angle_fix
            else:
                angle_fix = factor * (base_angle - 90)
                base_angle = 90 + angle_fix
            base_angle = int(round(base_angle))
            print(base_angle)

            # calc arm angles
            # convert to 2d problem
            x_arm = math.sqrt(points3d[0] * points3d[0] + points3d[2] * points3d[2])
            y_arm = points3d[1] * (-1)
            print((x_arm, y_arm))
            (a, b) = get_rot_angle(x_arm, y_arm)
            a = int(round(a))
            b = int(round(b))
            if (points3d[2] > 0.1):
                try:
                    arm.set_servo_values([base_angle, b, a, 0])
                    time.sleep(1)
                    arm.set_servo_values([base_angle, b, a, 0])
                    time.sleep(2)
                    arm.set_servo_values([90, 100, 80, 0])
                    time.sleep(1)
                except ValueError:
                    print("not is range, try again")
            else:
                print("too close, yep try again")

            good_to_go = 0
            move_now = 0

        font = cv2.FONT_HERSHEY_COMPLEX
        label = str(pointer3d[0]) + "," + str(pointer3d[1]) + "," + str(pointer3d[2])
        label2 = str(points3d[0]) + "," + str(points3d[1]) + "," + str(points3d[2])
        cv2.putText(images, label, (50, 50), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(images, label2, (50, 80), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow('RealSense', images)
        key = cv2.waitKey(1)

finally:
    # Stop streaming
    pipeline.stop()
