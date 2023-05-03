from unittest.mock import MagicMock, patch, Mock,call
from io import StringIO
import sys, os, unittest, socket, threading
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(os.path.dirname(__file__)))))
from run5_CAD import Main
import numpy as np
from SD.Basemodel.Actor import Actor
from SD.Basemodel.Sensor import Sensor
from SD.Basemodel.ObjectDetector import ObjectDetector
from SD.Decoder import H264decoder
from SD.Decoder.h264_39 import h264decoder
from SD.ObjectDetector.YOLOv5 import YOLOv5
from SD.Tello.Tello8889Actor import Tello8889Actor
from SD.Tello.Tello8889Sensor import Tello8889Sensor
from SD.Tello.Tello11111Sensor import Tello11111Sensor
from SD.Test.TelloVirtualController import TelloVirtualController
from SD.Plan.Planner5 import Planner
from SD.Calculation.ValueChecker import is_tof_val, is_sdk_val
from SD.Calculation.ValueChanger import change_mm_to_cm, change_val_to_coor, change_cmd_for_tello, change_windows_to_window, change_to_safe_cmd



#===========================================================================================================
class TestMain(unittest.TestCase):
    @patch('socket.socket', spec=socket.socket)
    @patch('socks.socksocket', spec=socket.socket)
    def test_TID5001(self, mock_socket, mock_socksocket):
        # Mock socket instance
        mock_socket_instance = MagicMock(spec=socket.socket)
        mock_socket.return_value = mock_socket_instance
        mock_socksocket.return_value = mock_socket_instance
        mock_socket_instance.recvfrom = MagicMock(return_value=(b'response', ('192.168.10.1', 8889)))
        mock_socket_instance.bind = MagicMock()
        mock_socket_instance.sendto = MagicMock()

        # Initialize Main instance
        main = Main(True)

        # Check attributes
        self.assertIsNotNone(main.stop_event)
        self.assertEqual(main.tello_address, ('192.168.10.1', 8889))
        self.assertEqual(main.is_takeoff, False)
        self.assertIsNotNone(main.socket8889)
        self.assertIsNotNone(main.planner)
        self.assertIsNotNone(main.tello8889sensor)
        self.assertIsNotNone(main.tello11111sensor)
        self.assertIsNotNone(main.tello8889actor)
        self.assertIsNotNone(main.virtual_controller)

#===========================================================================================================                         
class TestPlanner(unittest.TestCase):
    
    def setUp(self):
        self.main_mock = MagicMock()
        self.main_mock.stop_event = threading.Event()
        self.main_mock.stop_event.is_set = MagicMock(return_value=False)
        self.main_mock.virtual_controller = True  # virtual_controller 속성을 True로 변경
        self.main_mock.tello_address = ('192.168.10.1', 8889)
        self.main_mock.socket8889 = MagicMock()
        self.planner = Planner(self.main_mock, True)
        self.frame = MagicMock()
        self.frame.size = (640, 480)
        self.frame_image = MagicMock()

    #__init__
    def test_TID5002(self):
        self.assertIsNotNone(self.planner)
        self.assertEqual(self.planner.stop_event, self.main_mock.stop_event)
        self.assertEqual(self.planner.socket8889, self.main_mock.socket8889)
        self.assertEqual(self.planner.tello_address, self.main_mock.tello_address)
        self.assertEqual(self.planner.threshold_distance, 60)
        self.assertEqual(self.planner.base_move_distance, 60)
        self.assertEqual(self.planner.safe_constant, 20)
        self.assertEqual(self.planner._Planner__cmd_queue, [])
        self.assertIsNone(self.planner._Planner__info_8889Sensor_cmd)
        self.assertTrue(self.planner._Planner__thr_stay_connection.is_alive())

    #__redraw_frame
    def test_TID5003(self):
        test_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        test_tof = 100  # Example Time-of-Flight value
        self.planner.set_info_11111Sensor_frame(test_frame)
        self.planner.set_info_8889Sensor_tof(test_tof)

        frame, tof, object_coor = self.planner._Planner__redraw_frame()

        self.assertIsNone(frame)
        self.assertIsNone(tof)
        self.assertIsNone(object_coor)
   
   #__create_avd_cmd
    def test_TID5004(self):
        real_coor = (50, (30, -20), (20, 10))
        avd_cmd = self.planner._Planner__create_avd_cmd(real_coor)
        self.assertEqual(avd_cmd, "up 60")

    #_create_real_core
    def test_TID5005(self):
        object_core = ((50, 30), (100, 80))  # Example window coordinates
        tof = 40
        screen_size = (640, 480)
        expected_result = (40, (-9.442708333333334, 7.130208333333333), (1.9270833333333333, 1000))
        result = self.planner._Planner__create_real_coor(object_core, tof, screen_size)
        self.assertEqual(result, expected_result)

    #get_info_11111Sensor_coor
    def test_TID5006(self):
        self.assertIsNone(self.planner.get_info_11111Sensor_coor())
        self.planner._Planner__info_11111Sensor_coor = 'ready'
        self.assertEqual(self.planner.get_info_11111Sensor_coor(), 'ready')

    #set_info_11111Sensor_coor
    def test_TID5007(self):
        self.assertIsNone(self.planner._Planner__info_11111Sensor_coor)
        self.planner.get_info_11111Sensor_coor()
        self.assertEqual(self.planner._Planner__info_11111Sensor_coor, None)  
    

   
#===================================================
class MockTello8889Sensor(Tello8889Sensor):
    def __init__(self, test_val):
        self.test_val = test_val
        self.result = self.__func_sensor(test_val)
        
    
    def __func_sensor(self,test_val):
        data = self.take_data_from_sensor(test_val)
        info = self.change_data_to_info(data)
        result = self.save_to_planner(info)
        
        return result
        
    def take_data_from_sensor(self,test_val): 
        """
        센서로부터 data를 가져온다
        """
        data = test_val
        return data
    
    def change_data_to_info(self, data: bytes):
        """
        data를 Planner가 이해할 수 있는 info로 변경한다
        """
        info:str = data.decode('utf-8')
        return info
    
    def save_to_planner(self, info: str):
        """
        info를 Planner에 저장한다
        """
        
        if is_tof_val(info):
            #ToF 값은 "tof 100" 형태로 들어온다
            info = change_mm_to_cm(int(info.split()[-1]))
            if info > 60:
                info = 1000
            return info
        
        else: #cmd return 값이면
            return info
    
class MockPlanner(Planner):
    def __init__(self, middle_val):
        self.threshold_distance = 60
        self.safe_constant = 20
        self.base_move_distance = 60
        self.middle_val = middle_val 
        self.result = self.__func_planner()

    def __func_planner(self):
        frame, tof, object_coor =  self.__redraw_frame() #좌표받아오기
                
        screen_size = (960, 720)
        real_coor = self.__create_real_coor(object_coor, tof, screen_size)                        
        
        #3) 3차원 좌표를 바탕으로 회피 명령 생성
        avd_cmd = self.__create_avd_cmd(real_coor)
        return avd_cmd
                

    def __redraw_frame(self):
        frame = None
        tof = int(self.middle_val)
        
        if frame is not None and type(frame)!=str and type(self.__YOLOv5)!=str and frame.size != 0:     
            #YOLO에 frame을 전달하여, 객체인식이 적용된 이미지를 전달받음
            image, object_coor = self.__YOLOv5.detect_from_frame(frame, tof)
            
            return frame, tof, object_coor
        
        return frame, tof, ((40,0),(100,100))

    def __create_real_coor(self, object_coor, tof, screen_size):
        object_val = (tof, object_coor, screen_size) 
        object_coor = change_val_to_coor(object_val)
        return object_coor

    def __create_avd_cmd(self,real_coor:tuple): 
        """
        -입력값 - object_coor: (tof값[cm], 물체중심의 가로좌표[cm], 물체중심의 세로좌표[cm], 물체의 가로길이[cm], 물체의 세로길이[cm])
        -출력값 - avd_cmd: str (방향, 이동거리를 포함)
        """
        if real_coor is None or real_coor[0]>self.threshold_distance:
            return None
        
        tof_val = real_coor[0]
        real_center_coor = real_coor[1]
        real_length = real_coor[2]
        
        #계산된 회피거리를 저장할 변수
        horizontal_move = None
        vertical_move = None
        
        #물체의 좌표 + 텔로의 안전보정치 = 회피할 거리
        object_left_coor = real_center_coor[0] - real_length[0]//2 - self.safe_constant
        if object_left_coor < -100:
            object_left_coor = -100
        object_left_coor = -1*object_left_coor #크기비교를 위해 양수로 변환
            
        object_right_coor = real_center_coor[0] + real_length[0]//2 + self.safe_constant
        if object_right_coor > 100:
            object_right_coor = 100
        
        object_up_coor = real_center_coor[1] + real_length[1]//2 + self.safe_constant
        if object_up_coor > 100:
            object_up_coor = 100
        
        object_down_coor = real_center_coor[1] - real_length[1]//2 - self.safe_constant
        if object_down_coor < -100:
            object_down_coor = -100
        object_down_coor = -1*object_down_coor #크기비교를 위해 양수로 변환
        
        #어느 방향이 더 조금 움직여서 회피가 가능한지 계산
        if object_left_coor < object_right_coor:
            horizontal_move = -1 * object_left_coor
        else:
            horizontal_move = object_right_coor
            
        if abs(horizontal_move) == 100:
            horizontal_move = 0
        
            
        if object_down_coor < object_up_coor:
            vertical_move = -1 * object_down_coor
        else:
            vertical_move = object_up_coor
        
        if abs(vertical_move) == 100:
            vertical_move = 0
            
            
        #회피명령 생성 / 최소 움직임은 base_move_distance = 40cm
        avd_cmd = None
        
        if horizontal_move == 0 and vertical_move == 0:
            avoid_distance = self.threshold_distance - tof_val
            if avoid_distance < self.base_move_distance:
                avoid_distance = self.base_move_distance
                
            avd_cmd = "back {}".format(avoid_distance)
        
        elif horizontal_move == 0 and vertical_move != 0:
            if vertical_move < 0:
                if abs(vertical_move) < self.base_move_distance:
                    vertical_move = -1 * self.base_move_distance
                avd_cmd = "down {}".format(-1*vertical_move)
            
            else:
                if abs(vertical_move) < self.base_move_distance:
                    vertical_move = self.base_move_distance
                avd_cmd = "up {}".format(vertical_move)
        
        elif horizontal_move != 0 and vertical_move == 0:
            if horizontal_move < 0:
                if abs(horizontal_move) < self.base_move_distance:
                    horizontal_move = -1 * self.base_move_distance
                avd_cmd = "left {}".format(-1*horizontal_move)
            
            else:
                if abs(horizontal_move) < self.base_move_distance:
                    horizontal_move = self.base_move_distance
                avd_cmd = "right {}".format(horizontal_move)
        
        else:
            if abs(horizontal_move) < abs(vertical_move):
                if horizontal_move < 0:
                    if abs(horizontal_move) < self.base_move_distance:
                        horizontal_move = -1 * self.base_move_distance
                    avd_cmd = "left {}".format(-1*horizontal_move)
            
                else:
                    if abs(horizontal_move) < self.base_move_distance:
                        horizontal_move = self.base_move_distance
                    avd_cmd = "right {}".format(horizontal_move)
            
            else:
                if vertical_move < 0:
                    if abs(vertical_move) < self.base_move_distance:
                        vertical_move = -1 * self.base_move_distance
                    avd_cmd = "down {}".format(-1*vertical_move)
            
                else:
                    if abs(vertical_move) < self.base_move_distance:
                        vertical_move = self.base_move_distance
                    avd_cmd = "up {}".format(vertical_move)
        
        return avd_cmd

class MockTello8889Actor(Tello8889Actor):
    def __init__(self, middle_val, tof):
        self.__threshold_distance = 50 #회피를 수행할 거리(cm)
        self.middle_val = middle_val
        self.tof = tof
        self.result = self.__func_actor()

    def __func_actor(self):
        cmd = self.take_cmd_from_planner()
        
        safe_cmd = self.change_cmd_is_safe(cmd)
        drone_cmd = self.change_cmd_for_drone(safe_cmd)
        result =  self.send_to_actuator(drone_cmd)
        return result
    
    def take_cmd_from_planner(self): 
        """
        Planner로부터 cmd를 가져온다
        """
        cmd = self.middle_val
        return cmd

    def change_cmd_is_safe(self, cmd): 
        """
        cmd가 충돌이 발생하지 않는 명령으로 변환한다
        """
        safe_cmd = change_to_safe_cmd(cmd, self.tof, self.__threshold_distance)
        return safe_cmd

    def change_cmd_for_drone(self, cmd):
        """
        cmd를 Drone이 이해할 수 있는 cmd로 변경한다
        """
        drone_cmd = change_cmd_for_tello(cmd)
        return drone_cmd

    def send_to_actuator(self, cmd):
        """
        cmd를 Actuator에게 전송한다
        """
        if cmd:
            decode_cmd = cmd.decode()
            return decode_cmd
        else:
            return None
        


class TestScenario_CAD(unittest.TestCase):

    def test_TID5008(self):
        test_tof = b"100"
        self.tello8889sensor = MockTello8889Sensor(test_tof)
        tof = self.tello8889sensor.result
        self.assertEqual(tof, "100")
        
        self.planner = MockPlanner(tof)
        pre_cmd = self.planner.result
        self.assertEqual(pre_cmd, None)
        
        self.tello8889actor = MockTello8889Actor(pre_cmd, tof)
        cmd = self.tello8889actor.result
        self.assertEqual(cmd, None)
        
    def test_TID5009(self):
        test_tof = b"50"
        self.tello8889sensor = MockTello8889Sensor(test_tof)
        tof = self.tello8889sensor.result
        self.assertEqual(tof, "50")
        
        self.planner = MockPlanner(tof)
        pre_cmd = self.planner.result
        self.assertEqual(pre_cmd, "right 60")
        
        self.tello8889actor = MockTello8889Actor(pre_cmd, tof)
        cmd = self.tello8889actor.result
        self.assertEqual(cmd, "rc 60 0 0 0")
        
    def test_TID5010(self):
        test_tof = b"10"
        self.tello8889sensor = MockTello8889Sensor(test_tof)
        tof = self.tello8889sensor.result
        self.assertEqual(tof, "10")
        
        self.planner = MockPlanner(tof)
        pre_cmd = self.planner.result
        self.assertEqual(pre_cmd, "right 60")
        
        self.tello8889actor = MockTello8889Actor(pre_cmd, tof)
        cmd = self.tello8889actor.result
        self.assertEqual(cmd, "rc 60 0 0 0")
        
        
        
#===========================================================================================================                             
if __name__ == "__main__":
    unittest.main()