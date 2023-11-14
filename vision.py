import cv2
import numpy as np
import math
import threading
import time
import picamera
import picamera.array
from HSVConfig import * 



class FieldProcessing:
    def __init__(self):
        # 초록색과 회색 영역 인식을 위한 HSV 범위 설정
        self.green_lower = np.array([35, 50, 50]) # 색깔 마스킹 값 사전 저장 
        self.green_upper = np.array([75, 255, 255])
        self.gray_lower = np.array([0, 0, 40])
        self.gray_upper = np.array([180, 50, 220])
        self.yellow_lower = np.array([20, 140, 100], np.uint8)
        self.yellow_upper = np.array([30, 255, 255], np.uint8)
    
    def process_green_field(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv, self.green_lower, self.green_upper)
        gray_mask = cv2.inRange(hsv, self.gray_lower, self.gray_upper)

        # 회색 영역을 초록색 마스크에서 제외
        combined_mask = cv2.bitwise_and(green_mask, green_mask, mask=~gray_mask)

        # 경기장의 초록색 영역의 경계 찾기
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
            cv2.drawContours(frame, [approx], 0, (0, 255, 0), 5)

        # 회색 영역(벙커)의 경계 찾기
        gray_contours, _ = cv2.findContours(gray_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in gray_contours:
            cv2.drawContours(frame, [cnt], -1, (0, 0, 255), 2)

        return frame


    def find_yellow_circle_center(self, frame):
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(hsv_frame, self.yellow_lower, self.yellow_upper)
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                print(cX,cY)
                return (cX, cY)
        return None


class Vision:
    def __init__(self, W_View_size=600, H_View_size=400, hsv_Lower=0, hsv_Upper=0, KNOWN_WIDTH=4.5, FOV=62.2, h=30, alpha=45, h_min=[146], h_max=[179], s_min=[110], s_max=[255], v_min=[169], v_max=[255]):
        self.W_View_size = W_View_size
        self.H_View_size = H_View_size
        self.hsv_Lower = hsv_Lower
        self.hsv_Upper = hsv_Upper
        self.KNOWN_WIDTH = KNOWN_WIDTH
        self.FOV = FOV #시야각
        self.h = h #로봇 키 
        self.alpha = alpha
        self.h_min = h_min
        self.h_max = h_max
        self.s_min = s_min
        self.s_max = s_max
        self.v_min = v_min
        self.v_max = v_max
        self.ball = False
        self.R = None
        self.rotation_angle = None
        self.x = 0
        self.y = 0
        self.flag_x = 0
        self.flag_y = 0
        self.set_hsv_values()
        self.field_processor = FieldProcessing()

        #pi camera 설정 
        self.camera = picamera.PiCamera()
        self.camera.resolution = (self.W_View_size, self.H_View_size)
        self.camera.framerate = 64
        self.camera.sensor_mode = 2
        self.rawCapture = picamera.array.PiRGBArray(self.camera, size=(self.W_View_size, self.H_View_size))
        time.sleep(0.5)
        #파이 카메라 버퍼 할당 대기 

        self.thread = threading.Thread(target=self.run)
        self.thread.start()
        self.frame = None
        self.lock = threading.Lock()

    def set_hsv_values(self):
        self.hsv_Lower = (self.h_min[0], self.s_min[0], self.v_min[0])
        self.hsv_Upper = (self.h_max[0], self.s_max[0], self.v_max[0])

    def set_alpha(self, alpha):
        with self.lock:
            self.alpha = alpha

    def get_alpha(self):
        with self.lock:
            return self.alpha

    def get_R(self):
        with self.lock:
            return self.R
            
    def get_ball(self):
        with self.lock:
            return self.ball
    def get_xy(self):
        with self.lock:
            return (self.x, self.y)

    def tilt_point_around_x_axis(point, tilt_angle_degrees):
        # 각도를 라디안으로 변환
        tilt_angle_radians = np.radians(tilt_angle_degrees)
        
        # x축을 중심으로 기울이는 회전 행렬
        rotation_matrix = np.array([
            [1, 0, 0],
            [0, np.cos(tilt_angle_radians), -np.sin(tilt_angle_radians)],
            [0, np.sin(tilt_angle_radians), np.cos(tilt_angle_radians)]
        ])
        
        # 주어진 점의 좌표를 확장하여 3D 벡터로 변환
        point_vector = np.array([point[0], point[1], point[2]])
        
        # 회전 행렬을 점에 적용하여 새 좌표 계산
        new_point = rotation_matrix.dot(point_vector)
        
        return new_point

    def calculate_angle_between_lines(self,ball_x, ball_y, flag_x, flag_y, center_x):
        vector_ball = np.array([ball_x - flag_x, ball_y - flag_y])

        vector_flag = np.array([center_x-ball_x, self.H_View_size-ball_y])

        # 두 벡터의 내적 계산
        dot_product = np.dot(vector_ball, vector_flag)

        # 두 벡터의 크기 계산
        magnitude_ball = np.linalg.norm(vector_ball)
        magnitude_flag = np.linalg.norm(vector_flag)

        # 코사인 각도 계산
        cos_angle = dot_product / (magnitude_ball * magnitude_flag)

        # 각도를 라디안에서 도(degree)로 변환
        angle = math.degrees(np.arccos(cos_angle))

        return angle

    def set_strike_angle(self): # 두개의 사이각 계산 # 골대 안잡히면 -> 진행 방향에 있는 좌표로 깃발 좌표 대체 
        center_x = self.W_View_size / 2
        center_y = self.H_View_size / 2
        x,y = self.get_xy()
        try:
            angle = self.calculate_angle_between_lines(x, y, self.flag_x, self.flag_y, center_x)
        except:
            return None
        return angle


    def calculate_angle(self, v1, v2, theta):
        """
        v1, v2: 2차원 이미지 상의 두 벡터
        theta: z축을 중심으로 회전시키고 싶은 각도
        """
        # 2차원 벡터를 3차원 공간으로 변환
        v1_3d = np.array([v1[0], v1[1], 0])
        v2_3d = np.array([v2[0], v2[1], 0])
        
        # z축을 중심으로 회전 변환 행렬 생성
        rotation_matrix = np.array([
            [math.cos(math.radians(theta)), -math.sin(math.radians(theta)), 0],
            [math.sin(math.radians(theta)), math.cos(math.radians(theta)), 0],
            [0, 0, 1]
        ])
        
        # 회전 변환 행렬을 사용하여 3차원 공간에서의 두 벡터를 회전시켜 보정
        v1_corrected_3d = np.dot(rotation_matrix, v1_3d)
        v2_corrected_3d = np.dot(rotation_matrix, v2_3d)
        
        # 보정된 3차원 벡터를 2차원으로 투영
        v1_corrected_2d = v1_corrected_3d[:2]
        v2_corrected_2d = v2_corrected_3d[:2]
        
        # 두 벡터의 내적을 사용하여 코사인 값을 구함
        dot_product = np.dot(v1_corrected_2d, v2_corrected_2d)
        magnitude_v1 = np.linalg.norm(v1_corrected_2d)
        magnitude_v2 = np.linalg.norm(v2_corrected_2d)
        cos_angle = dot_product / (magnitude_v1 * magnitude_v2)
        
        # 코사인 값을 사용하여 각도를 구함
        angle_rad = math.acos(cos_angle)
        angle_deg = math.degrees(angle_rad)
        
        return angle_deg

    def is_point_inside_boundary(point, contour): # 그 좌표가 사용 가능한 경계 안에 있는지 여부를 확인한다.
        return cv2.pointPolygonTest(contour, point, False) > 0


    def get_rotation_angle(self):
        with self.lock:
            return self.rotation_angle

    def process_frame(self):
        #프레임 처리

        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(hsv, self.field_processor.yellow_lower, self.field_processor.yellow_upper)

        # 노란색 객체의 윤곽선 검출
        yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in yellow_contours:
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                # 중심점에 점 찍기
                cv2.circle(self.frame, (cX, cY), 5, (0, 0, 255), -1)

        # 노란색 객체의 윤곽선 그리기
        cv2.drawContours(self.frame, yellow_contours, -1, (0, 255, 255), 2)
        frame = self.field_processor.process_green_field(self.frame)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_Lower, self.hsv_Upper)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if cnts:
            try:
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                self.x = x
                self.y = y
            except:
                return 0
            if(radius > 10):
                self.ball = True
        

            d = self.h * math.tan(math.radians(self.alpha))

            center_x = self.W_View_size / 2
            center_y = self.H_View_size / 2
            p_pixel_x = abs(x - center_x)
            p_pixel_y = abs(y - center_y)
            theta_x = p_pixel_x * (self.FOV / self.W_View_size)
            theta_y = p_pixel_y * (self.FOV / self.H_View_size)
            
            ball_vector = np.array([x - center_x, (y - center_y)])
            top_vector = np.array([0, center_y])
            theta_deg = self.calculate_angle(ball_vector,top_vector, self.get_alpha())
            theta_deg_before = self.calculate_angle(ball_vector, top_vector, 0)
            yellow_flag_position = self.field_processor.find_yellow_circle_center(self.frame)
            
            if yellow_flag_position is not None:
                # 노란색 깃발의 위치에 원을 그립니다.
                self.flag_x = yellow_flag_position[0]
                self.flag_y = yellow_flag_position[1]
                cv2.line(frame, (int(self.flag_x), int(self.flag_y)), (int(x), int(y)), (0, 255, 0), 4)
            real_t = 180 - (theta_x + theta_deg)
            with self.lock:
                try:
                    self.R = (d/math.sin(math.radians(real_t))) * math.sin(math.radians(theta_deg))
                    #self.R = d / np.cos(theta_deg)

                    self.rotation_angle = theta_x if x > center_x else -theta_x
                except:
                    pass
            try:
                cv2.circle(frame, (int(x), int(y)), int(radius), (255, 255, 255), 2)
                cv2.putText(frame, f"Distance: {self.R:.2f}cm", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(frame, f"Rotation Angle: {self.rotation_angle:.2f} degrees", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.line(frame, (int(center_x), int(self.H_View_size)), (int(x), int(y)), (255, 0, 0), 2)
        # 화면 중앙과 x좌표가 같고 y좌표는 맨 위인 좌표를 이은 선을 그림
    
                cv2.imshow('Frame', frame)

                if cv2.waitKey(30) & 0xFF == ord('q'):
                    self.camera.release()
                    cv2.destroyAllWindows()
                    exit()

            except:
                pass
        else:
            self.ball = False

        
    def run(self):
        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            self.frame = frame.array
            if self.frame is None:
                print("nononono")
                continue
            else:
                self.process_frame()
                self.rawCapture.truncate(0)
        field_processor.camera.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    vision = Vision()
    # 다른 클래스에서 Vision 클래스의 인스턴스 변수에 접근하는 예시
    while True:
        R = vision.get_R()
        rotation_angle = vision.get_rotation_angle()
        print("[*] goal and ball angle :" + str(180 - vision.set_strike_angle()))
