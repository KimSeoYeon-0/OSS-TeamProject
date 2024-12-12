#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.iodevices import UARTDevice
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait
import time

#==========[Initialize]==========
ev3 = EV3Brick()
gyro = GyroSensor(Port.S3)
ser = UARTDevice(Port.S4, baudrate=115200)

#==========[motors]==========
shooting_motor = Motor(Port.C)
grab_motor = Motor(Port.B)

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=115)
robot.settings(500, 300, 1000, 1000)  # 속도 및 가속도 설정

#==========[utility functions]==========
def process_uart_data(data):
    try:
        data_str = data.decode().strip()
        if not data_str:
            return [-1, -1]
        
        data_str = data_str.strip("[]")
        parsed_list = [int(value.strip()) for value in data_str.split(",")]
        if len(parsed_list) != 2:
            return [-1, -1]
        return parsed_list
    except:
        return [-1, -1]

def pd_control(cam_data, kp, kd, power):
    global previous_error
    error = cam_data - threshold
    derivative = error - previous_error
    output = (kp * error) + (kd * derivative)
    robot.drive(power, output)
    previous_error = error

def shoot(command):
    if command == 'zero':
        print("슈팅 초기화 시작")
        shooting_motor.run_until_stalled(-100, Stop.COAST, duty_limit=60) 
        shooting_motor.reset_angle(0)
        print("슈팅 초기화 완료")
    elif command == 'shoot':
        print("슈팅 동작 시작")
        shooting_motor.run(2000)  
 
        print("슈팅 동작 진행 중")
        time.sleep(0.25)
        shooting_motor.stop()
        print("슈팅 동작 완료")


def grab(command):
    if command == 'motion3':  # 공을 잡기
        grab_motor.run_target(1000, 0, Stop.COAST)
        grab_motor.reset_angle(0)
    elif command == 'motion1':  # 팔 열기
        grab_motor.run_target(500, -90, Stop.COAST)
    elif command == 'motion2':  # 팔 위치 조정
        grab_motor.run_target(500, -100, Stop.COAST)

def turn_to_angle(target_angle, speed=50):
    
    while True:
        current_angle = gyro.angle()
        error = target_angle - current_angle

        if abs(error) < 2:  # 각도 오차가 2도 이하이면 정지
            break

        turn_rate = speed if error > 0 else -speed  # 회전 방향 설정
        robot.drive(0, turn_rate)
        wait(10) 

    robot.stop()

def search_for_ball():
    print("공을 탐색 중...")
    robot.drive(0, 50)  # 천천히 회전
    start_time = time.time()
    while time.time() - start_time < 10:  # 한 바퀴 돌며 공 탐색
        data = ser.read_all()
        filter_result = process_uart_data(data)
        if filter_result[0] != -1 and filter_result[1] != -1:
            robot.stop()
           
            return filter_result
    print("공을 찾지 못했습니다.")
    return [-1, -1]

#==========[setup]==========
ev3.speaker.beep()
threshold = 40
previous_error = 0
gyro.reset_angle(0)
initial_angle = gyro.angle()
shoot('zero')
grab('motion1')  # 팔을 열어 초기 상태로 시작
print("초기 위치 설정 완료")

#==========[main loop]==========
while True:
    print("메인 루프 시작")
    data = ser.read_all()
    filter_result = process_uart_data(data)

    if filter_result[0] != -1 and filter_result[1] != -1:
       
        if filter_result[1] > 105:  # 공이 가까워졌을 때
            print("공에 접근 중...")
            robot.straight(100)  # 천천히 공에 다가감

            grab('motion3')  # 공 잡기
            time.sleep(1)  # 동작 완료 대기
          
            turn_to_angle(initial_angle, speed=30)  # 부드럽게 원래 각도로 복귀
            time.sleep(1)
            grab('motion1')  # 팔 열기
            print("팔 열기 완료")
            time.sleep(0.5)
            shoot('shoot')  # 슈팅
            print("슈팅 완료")
            time.sleep(1)
            shoot('zero')  # 슈팅 초기화
            print("슈팅 초기화 완료")
            time.sleep(1)

            # 상태 초기화를 위한 추가 동작
            grab('motion1')
            print("팔 초기화 완료")
            time.sleep(1)

            # 원위치로 돌아가 다시 준비 상태로 복귀
            robot.straight(-100)  # 원래 위치로 후진
            print("시작 위치로 복귀 완료")
            time.sleep(1)

            gyro.reset_angle(0)  # 자이로 센서 초기화
            print("자이로 센서 초기화 완료")

            print("초기 상태로 복귀 완료")

            # 초기화 후 상태 재설정
            previous_error = 0  # PD 제어 초기화
            robot.stop()  # 로봇 정지
            continue
        else:
            print("PD 제어를 사용하여 방향 조정 중...")
            pd_control(filter_result[0], kp=0.5, kd=0.1, power=100)
    else:
        print("공을 탐색 중...")
        filter_result = search_for_ball()
        if filter_result[0] != -1 and filter_result[1] != -1:
            pd_control(filter_result[0], kp=0.5, kd=0.1, power=100)

    wait(50)
    print("루프 반복 완료")
