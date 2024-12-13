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
color_sensor = ColorSensor(Port.S1)

#==========[motors]==========
shooting_motor = Motor(Port.C)
grab_motor = Motor(Port.B)

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=115)
robot.settings(1200, 800, 1000, 1000)  # 속도 및 가속도 설정

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
        shooting_motor.run_until_stalled(-200, Stop.COAST, duty_limit=60) 
        shooting_motor.reset_angle(0)
    elif command == 'shoot':
        shooting_motor.run(4500)  # 더 높은 속도와 각도로 슈팅 설정
        time.sleep(0.25)
        shooting_motor.stop()

def grab(command):
    if command == 'motion3':  # 공을 잡기
        grab_motor.run_target(1500, 0, Stop.COAST) # hold도 써봐
        grab_motor.reset_angle(0)
    elif command == 'motion1':  # 팔 열기
        grab_motor.run_target(1200, -90, Stop.COAST)
    elif command == 'motion2':  # 팔 위치 조정
        grab_motor.run_target(1200, -100, Stop.COAST)

def turn_to_angle(target_angle, speed=40):
    """
    자이로 센서를 사용하여 원하는 각도로 부드럽게 회전합니다.
    :param target_angle: 목표 각도
    :param speed: 최대 회전 속도 (기본값 50)
    """
    while True:
        current_angle = gyro.angle()
        error = target_angle - current_angle

        if abs(error) < 2:  # 각도 오차가 2도 이하이면 정지
            break

        turn_rate = speed if error > 0 else -speed  # 회전 방향 설정
        robot.drive(0, turn_rate)
        wait(10)  # 짧은 대기 시간으로 제어 반복

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

def shoot_towards_opponent():
    
    target_angle = initial_angle % 360  
    current_angle = gyro.angle() % 360  

    # 최소 회전 각도 계산
    angle_difference = (target_angle - current_angle + 360) % 360
    if angle_difference > 180:          
        angle_difference -= 360

    
    turn_to_angle(current_angle + angle_difference, speed=30)  # 최소 회전만 수행

    
    grab('motion1')  # 팔 열기
    time.sleep(1)
    shoot('shoot')  # 공 슈팅
    time.sleep(1)
   
    

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
    detected_color = color_sensor.color()
    if detected_color == Color.GREEN:  # 초록색 감지 시 후진
        robot.stop()
        robot.straight(-100)
        continue

    print("메인 루프 시작")
    data = ser.read_all()
    filter_result = process_uart_data(data)

    if filter_result[0] != -1 and filter_result[1] != -1:
       
        if filter_result[1] > 105:  
            print("공에 접근 중...")
            robot.straight(100)  
            #robot.stop?

           

            grab('motion3')  # 공 잡기
            time.sleep(1)  # 동작 완료 대기
           
             
           
            shoot_towards_opponent()
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