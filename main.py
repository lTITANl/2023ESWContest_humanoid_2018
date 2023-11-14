import serial
import cv2
import time
from vision import *

temp_count = 0


class Track_MOVEMENT:
    def __init__(self): 
        self.h_angle = None # 현재 머리를 얼마나 숙였는지 저장하는 변수 
        self.ball_distance = None # 공과의 거리를 저장하는 변수 
        self.rotate_angle = None # 회전해야하는 각도
        self.device = '/dev/ttyS0' # 로봇 움직이기 위한 디바이스
        self.device_handle = None
        self.vision = Vision() # vision 클래스를 이용\
        self.movement = {
    "왼쪽턴10": 1,
    "보행횟수6_전진종종걸음": 2,
    "오른쪽턴10": 3,
    "왼쪽턴3": 4,
    "적외선거리값_읽기_등": 5,
    "오른쪽턴3": 6,
    "왼쪽턴20": 7,
    "전진종종걸음": 8,
    "오른쪽턴20": 9,
    "전진달리기50": 10,
    "연속전진": 11,
    "연속후진": 12,
    "오른쪽옆으로70연속": 13,
    "왼쪽옆으로70연속": 14,
    "왼쪽옆으로20": 15,
    "POWER_관련_기능들": 16,
    "머리왼쪽90도": 17,
    "자이로OFF_에러음_등": 18,
    "오른쪽턴60": 19,
    "오른쪽옆으로20": 20,
    "머리좌우중앙": 21,
    "왼쪽턴45": 22,
    "에러음_All_motor_mode2_등": 23,
    "오른쪽턴45": 24,
    "왼쪽턴60": 25,
    "기본자세_등": 26,
    "머리오른쪽90도": 27,
    "머리왼쪽45도": 28,
    "전방하향80도": 29,
    "머리오른쪽45도": 30,
    "전방하향60도": 31,
    "후진종종걸음": 32,
}
    #def donggihwa(self, ): # vision 함수로부터 angle, 거리등의 기본적인 정보를 받아옴
        self.set_env()

    def set_env(self):
        #시리얼 오픈 
        #핸들 저장
        self.device_handle = serial.Serial(self.device, 4800, timeout=0.01)
        self.device_handle.flush()
        time.sleep(0.5)

    def TX_data(self, opcode):
        self.device_handle.write(serial.to_bytes([opcode]))
        time.sleep(0.5)

    def TX_data_len(self, opcodes, len):
        for i in range(0, len):
            self.device_handle.write(serial.to_bytes([opcode]))
    
    #def find_ball(self):

    def sett(self):
        self.TX_data(250)
################################################################
    def stop(self):
        self.TX_data(26)
        time.sleep(0.7)
    def walk(self): #걷기 // 단발성임 // 계속 반복문으로 보내줘야됨 
        self.TX_data(11)
        time.sleep(0.5)
        self.stop()
        time.sleep(0.5)

    def _walk(self): #걷기 // 단발성임 // 계속 반복문으로 보내줘야됨 
        self.TX_data(11)
        time.sleep(0.5)
    def back(self): #걷기 // 단발성임 // 계속 반복문으로 보내줘야됨 
        self.TX_data(12)
        time.sleep(0.5)
        self.stop()

    def right_side_walk(self):
        self.TX_data(13)
        time.sleep(0.5)
        self.stop()
        time.sleep(0.5)

################################################################    
# 돌기 
    def right_turn_nano(self):
        self.TX_data(6)
        time.sleep(0.5)
    def right_turn_10(self):
        self.TX_data(9)
        time.sleep(0.5)

    def left_turn_45(self):
        self.TX_data(22)
        time.sleep(0.5)

    def left_turn_nano(self):
        self.TX_data(4)
        time.sleep(0.5)
    def left_turn_10(self):
        self.TX_data(7)
        time.sleep(0.5)

    def right_turn_45(self):
        self.TX_data(24)
        time.sleep(0.5)
#######################################################ßßß#########
#고개 숙이기 각도별 (앞으로)

    def front_head_60(self):
        self.vision.set_alpha(60)
        self.TX_data(31)
        time.sleep(0.5)

    def front_head_80(self):
        self.vision.set_alpha(80)
        self.TX_data(29)
        time.sleep(0.5)

    def front_head_30(self):
        self.vision.set_alpha(30)
        self.TX_data(34)
        time.sleep(0.5)

    def front_head_45(self):
        self.vision.set_alpha(45)
        self.TX_data(33)
        time.sleep(0.5)

    def head_set_center(self):
        self.vision.set_alpha(90)
        self.TX_data(35)
        time.sleep(0.5)

#################################
#고개 숙이기(횡이동)
    
    def left_head_45(self):
        self.TX_data(self.movement['머리왼쪽45도'])
        time.sleep(0.5)

    def left_head_90(self):
        self.TX_data(self.movement['머리왼쪽90도'])
        time.sleep(0.5)

################################################################
    def rotate_head(self, angle): 
        self.h_angle = angle # 현재의 머리 각도를 저장함
        self.TX_data(21)



class GOLF:
    def __init__(self):
        self.move = Track_MOVEMENT()
        self.vision = self.move.vision
        self.move.head_set_center()
        self.W_View_size=600
        self.H_View_size=400
        self.center_x = self.W_View_size / 2
        self.center_y = self.H_View_size / 2
        self.how_musch_rotate = 0
        time.sleep(2)

    def whole_vertical(self):
        self.move.front_head_80()
        if(self.vision.get_ball()):
            print("[*]found ball!!")
            self.move.front_head_60()
            return 1
                
        self.move.front_head_60()

        if(self.vision.get_ball()):
            print("[*]found ball!!")
            return 1
        self.move.front_head_45()
        if(self.vision.get_ball()):
            print("[*] found ball")
            return 1

        self.move.front_head_30()
        if(self.vision.get_ball()):
            print("[*] found ball")
            return 1
            

        return 0

    def find_ball(self):
        while self.vision.ball == False:
            if(self.whole_vertical()): break

            self.move.left_turn_45()
            if(self.whole_vertical()):break

            self.move.right_turn_45()
            self.move.right_turn_45()
            if(self.whole_vertical()):break

    def check_straight(self):
        if (-2<= self.vision.get_rotation_angle() <=2):
            return True
        return False


    def ball_check(self):
        return self.vision.get_ball()

    def set_straight(self):
        while self.check_straight() == False:
            a = self.vision.get_rotation_angle()
            if 1 <= a <=5:

                self.move.right_turn_nano()
            
            elif 5<= a <= 20:
                self.move.right_turn_10()

            elif 30 >= a >= 20:

                self.move.right_turn_45()

            if (-1>= a >=-20):

                self.move.left_turn_nano()

            elif -5>= a >= -20:
                self.move.left_turn_10()

            elif (-30 <a <= -20):

                self.move.left_head_45()

            time.sleep(1)

            if(self.ball_check() == False):
                self.find_ball()

    def set_interval(self):
        while True:
            if(self.vision.get_alpha() == 60):
                self.move.front_head_45()
            if(self.vision.get_alpha() == 45): #고개가 45도일때와 30도일때 인식하는 거리가 다르다. 상대적인 값 대입 
                if(self.vision.get_R() <= 17):
                    self.move.back()
                    self.set_straight()
                elif(self.vision.get_R() >= 20):
                    self.close_walk()
                    self.set_straight()
                    return
                else:
                    return
            if(self.vision.get_alpha() == 30):
                if(self.vision.get_R() <=21):
                    self.move.back()
                    self.set_straight()
                elif(self.vision.get_R() >= 27):
                    self.move.close_walk()
                    self.set_straight()
                    return
                else:
                    return

    def ball_auto_find(self):
        if(self.ball_check() == False):
            self.find_ball()

    def close_walk(self):
        self.move.walk()
        self.ball_auto_find()
        self.set_straight()
        if(self.vision.get_alpha() >= 80):
            self.move.front_head_60()
        x, y = self.vision.get_xy()
        global distance
        if(y > self.center_y and self.vision.get_alpha() >= 60):
            self.move.front_head_45()
        elif(y >  self.center_y and self.vision.get_alpha() >= 45):
            self.move.front_head_30()
        time.sleep(0.5)
        self.ball_auto_find()
        time.sleep(0.5)

    def long_walk(self):
        self.set_straight()
        self.move._walk()
        time.sleep(4)
        self.move.stop()
        self.set_straight()
        time.sleep(1)
        x, y = self.vision.get_xy()
        if(self.vision.get_alpha() >= 80):
            self.move.front_head_60()
        elif(y > self.center_y and self.vision.get_alpha() >= 60):
            self.move.front_head_45()
        elif(y >  self.center_y and self.vision.get_alpha() >= 45):
            self.move.front_head_30()
        if(self.ball_check() == False):
            self.find_ball()

if __name__ == "__main__":
    g = GOLF()
    g.find_ball() 
    distance = g.vision.get_R() 
    print(distance)

    added_angle = 0
    angle = g.vision.set_strike_angle()
    angle = 180 - angle - 90
    while distance >= 30:
        if(distance > 50):
            g.long_walk()
        else:
            g.close_walk()
        time.sleep(1)
        distance = g.vision.get_R()

    g.set_straight()
    g.set_interval()

    


