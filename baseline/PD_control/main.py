#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.iodevices import UARTDevice
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time

#==========[Initialize]==========
#==========[sensors]==========
ev3 = EV3Brick()
gyro = GyroSensor(Port.S1)
ser = UARTDevice(Port.S2, baudrate=115200)
color = ColorSensor(Port.S3)
ultra = UltrasonicSensor(Port.S4)


#==========[motors]==========

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=115)


#==========[camera_chase]==========
def process_uart_data(data):
    try:
        # 데이터를 문자열로 디코드 (키워드 인자 제거)
        data_str = data.decode().strip()
        if not data_str:
            pass

        # 문자열에서 리스트 파싱
        data_str = data_str.strip("[]")
        parsed_list = [int(value.strip()) for value in data_str.split(",")]

        # 파싱된 결과 반환
        return parsed_list
    except:
        # 에러 처리
        return [-1,-1] # -1이 나오면 무시하는 코드 사용

def pd_control(cam_data, kp, kd, power):
    global previous_error
    error = cam_data - threshold
    derivative = error - previous_error
    output = (kp * error) + (kd * derivative)
    robot.drive(power, output)
    previous_error = error


#==========[setup]==========
ev3.speaker.beep()
threshold = 80
previous_error = 0


# #==========[main loop]==========
while True:
    try:
        color_threshold = color.rgb()
        color_threshold = list(color_threshold)
        distance = ultra.distance() 
        if distance <= 100:
            robot.straight(-150)
            robot.turn(-100)
        robot.straight(80)
        robot.turn(30)
        data = ser.read_all()
        filter_result = process_uart_data(data)
        if filter_result[0]!= -1 and filter_result[1]!= -1:
            print(filter_result)
            pd_control(filter_result[0], kp=0.5, kd=0.1, power=100)
        wait(10)
    except:
        pass