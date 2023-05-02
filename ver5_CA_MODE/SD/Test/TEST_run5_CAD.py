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
class TestActor_(Actor):
    def __init__(self):
        self.command = None

    def take_cmd_from_planner(self, cmd):
        self.command = cmd
        return self.command

    def change_cmd_is_safe(self, cmd):
        return 'safe_' + cmd

    def change_cmd_for_drone(self, cmd):
        return 'drone_' + cmd

    def send_to_actuator(self, cmd):
        return 'sent_' + cmd

class TestActor(unittest.TestCase):
    def setUp(self):
        self.test_actor = TestActor_()

    #take_cmd_from_planner
    def test_TID1002(self):
        mock_planner = MagicMock()
        mock_planner.get_command.return_value = 'example_command'

        cmd = self.test_actor.take_cmd_from_planner(mock_planner.get_command.return_value)
        self.assertEqual(cmd, 'example_command')
        self.assertEqual(self.test_actor.command, 'example_command')

    #change_cmd_is_safe
    def test_TID1003(self):
        safe_cmd = self.test_actor.change_cmd_is_safe('example_command')
        self.assertEqual(safe_cmd, 'safe_example_command')

    #change_cmd_for_drone
    def test_TID1004(self):
        drone_cmd = self.test_actor.change_cmd_for_drone('example_command')
        self.assertEqual(drone_cmd, 'drone_example_command')

    #send_to_actuator
    def test_TID1005(self):
        sent_cmd = self.test_actor.send_to_actuator('example_command')
        self.assertEqual(sent_cmd, 'sent_example_command')
                
            
           
#===========================================================================================================         
class TestSensor_(Sensor):
    def __init__(self):
        self.data = None
        self.info = None

    def take_data_from_sensor(self, data):
        self.data = data
        return self.data

    def change_data_to_info(self, data):
        self.info = 'info_' + data
        return self.info

    def save_to_planner(self, mock_planner, info):
        mock_planner.save_info(info)
        return True

class TestSensor(unittest.TestCase):
    def setUp(self):
        self.test_sensor = TestSensor_()
        
    #take_data_from_sensor
    def test_TID1006(self):
        data = self.test_sensor.take_data_from_sensor('example_data')
        self.assertEqual(data, 'example_data')
        self.assertEqual(self.test_sensor.data, 'example_data')
        
    #change_data_to_info
    def test_TID1007(self):
        info = self.test_sensor.change_data_to_info('example_data')
        self.assertEqual(info, 'info_example_data')
        
    #save_to_planner
    def test_TID1008(self):
        mock_planner = MagicMock()
        info = 'info_example_data'
        result = self.test_sensor.save_to_planner(mock_planner, info)
        self.assertTrue(result)
        mock_planner.save_info.assert_called_once_with(info)

   

#===========================================================================================================   
class TestObjectDetector_(ObjectDetector):
    def __init__(self):
        self.window_image = None
        self.window_coor = None

    def detect_from_frame(self, frame):
        self.window_image = np.random.randint(0, 256, (480, 640, 3), dtype=np.uint8)
        self.window_coor = ((0, 0), (100, 100))
        return self.window_image, self.window_coor

class TestObjectDetector(unittest.TestCase):
    def setUp(self):
        self.test_detector = TestObjectDetector_()

    #test_detect_from_frame
    def test_TID4002(self):
        frame = np.random.randint(0, 256, (480, 640, 3), dtype=np.uint8)
        window_image, window_coor = self.test_detector.detect_from_frame(frame)

        self.assertIsNotNone(window_image)
        self.assertIsNotNone(window_coor)
        self.assertEqual(window_image.shape, (480, 640, 3))
        self.assertEqual(window_coor, ((0, 0), (100, 100)))



#===========================================================================================================     
class TestTello8889Actor(unittest.TestCase):
    def setUp(self):
        self.main_mock = MagicMock()
        self.main_mock.stop_event = threading.Event()
        self.main_mock.socket8889 = MagicMock()
        self.main_mock.tello_address = ('192.168.10.1', 8889)
        self.main_mock.planner = MagicMock()
        self.tello_actor = Tello8889Actor(self.main_mock)
        self.virtual_controller = MagicMock()
        self.my_class = Tello8889Actor(self.main_mock)
    
    #__init__
    def test_TID1009(self):
        actor = Tello8889Actor(self.main_mock)
        self.assertIsNotNone(actor._Tello8889Actor__thr_actor)
        self.assertEqual(actor._Tello8889Actor__pre_cmd, None)
        self.assertEqual(actor._Tello8889Actor__threshold_distance, 50)
        self.assertEqual(actor._Tello8889Actor__stop_event, self.main_mock.stop_event)
        self.assertEqual(actor._Tello8889Actor__main, self.main_mock)
        self.assertEqual(actor._Tello8889Actor__tello_address, self.main_mock.tello_address)
        self.assertEqual(actor._Tello8889Actor__planner, self.main_mock.planner)
        self.assertEqual(actor._Tello8889Actor__socket, self.main_mock.socket8889)
    
        
    #__func_actor
    def test_TID1010(self):
        cmd = self.tello_actor.take_cmd_from_planner()
        safe_cmd = cmd
        drone_cmd = self.tello_actor.change_cmd_for_drone(safe_cmd)
        self.assertNotEqual(drone_cmd,cmd)
        
    #take_cmd_from_planner
    def test_TID1011(self):
        self.main_mock.planner.pop_cmd_queue.return_value = 'command'
        cmd = self.tello_actor.take_cmd_from_planner()
        self.assertEqual(cmd, 'command')
        
    #change_cmd_is_safe
    def test_TID1012(self):
        self.main_mock.planner.get_info_8889Sensor_tof.return_value = 100
        safe_cmd = self.tello_actor.change_cmd_is_safe('forward 10')
        self.assertEqual(safe_cmd, 'forward 40')

    #change_cmd_for_drone
    def test_TID1013(self):
        drone_cmd = self.tello_actor.change_cmd_for_drone('forward 10')
        self.assertEqual(drone_cmd, b'rc 0 60 0 0')

    #send_to_actuator
    def test_TID1014(self):
        self.tello_actor.send_to_actuator("command".encode('utf-8'))
        self.main_mock.socket8889.sendto.assert_called_once_with("command".encode('utf-8'), self.main_mock.tello_address)
    
    #__printc
    def test_TID1015(self):
        expected_output = ""
        with patch('sys.stdout', new=StringIO()) as fake_stdout:
            self.main_mock.planner._Planner__printc("Hello world!")
            self.assertEqual(fake_stdout.getvalue().strip(), expected_output)
    
    #__printf
    def test_TID1016(self):
        expected_output = ""
        with patch('sys.stdout', new=StringIO()) as fake_stdout:
            self.main_mock.planner._Planner__printf("Hello world!", "my_function")
            self.assertEqual(fake_stdout.getvalue().strip(), expected_output)
          
      

#===========================================================================================================   
class TestTello8889Sensor(unittest.TestCase):
    def setUp(self):
        self.main_mock = MagicMock()
        self.main_mock.stop_event = threading.Event()
        self.main_mock.socket8889 = MagicMock()
        self.main_mock.planner = MagicMock()
        self.tello_sensor = Tello8889Sensor(self.main_mock)

    def tearDown(self):
        # Set the stop event to signal the daemon thread to stop
        self.main_mock.stop_event.set()
        self.tello_sensor._Tello8889Sensor__thr_sensor.join()
    
    #__init__
    def test_TID1017(self):
        sensor = Tello8889Sensor(self.main_mock)
        self.assertEqual(sensor._Tello8889Sensor__stop_event, self.main_mock.stop_event)
        self.assertEqual(sensor._Tello8889Sensor__main, self.main_mock)
        self.assertEqual(sensor._Tello8889Sensor__planner, self.main_mock.planner)
        self.assertEqual(sensor._Tello8889Sensor__socket, self.main_mock.socket8889)
        self.assertIsInstance(sensor._Tello8889Sensor__thr_sensor, threading.Thread)
    
    #__func_sensor
    def test_TID1018(self):
        data = self.tello_sensor.take_data_from_sensor()
        info = self.tello_sensor.change_data_to_info(data)
        self.assertNotEqual(data,info)
        
    
    #take_data_from_sensor
    def test_TID1019(self):
        self.main_mock.socket8889.recv.return_value = b'tof 100'
        data = self.tello_sensor.take_data_from_sensor()
        self.assertEqual(data, b'tof 100')

    #change_data_to_info
    def test_TID1020(self):
        info = self.tello_sensor.change_data_to_info(b'tof 100')
        self.assertEqual(info, 'tof 100')

    #save_to_planner
    def test_TID1021(self):
        self.tello_sensor.save_to_planner('tof 100')
        self.main_mock.planner.set_info_8889Sensor_tof.assert_called_once_with(10)            
 
    #__printc
    def test_TID1022(self):
        expected_output = ""
        with patch('sys.stdout', new=StringIO()) as fake_stdout:
            self.main_mock.planner._Planner__printc("Hello world!")
            self.assertEqual(fake_stdout.getvalue().strip(), expected_output)
    
    #__printf
    def test_TID1023(self):
        expected_output = ""
        with patch('sys.stdout', new=StringIO()) as fake_stdout:
            self.main_mock.planner._Planner__printf("Hello world!", "my_function")
            self.assertEqual(fake_stdout.getvalue().strip(), expected_output)



#=========================================================================================================== 
class TestTello11111Sensor(unittest.TestCase):
    def setUp(self):
        self.main_mock = MagicMock()
        self.main_mock.stop_event = threading.Event()
        self.main_mock.socket11111 = MagicMock()
        self.main_mock.socket11111.recv = MagicMock(return_value=b'\x00\x00\x01')  # Add this line to set a return value for recv()
        self.main_mock.planner = MagicMock()
        self.tello_sensor = Tello11111Sensor(self.main_mock)

    def tearDown(self):
        # Set the stop event to signal the daemon thread to stop
        self.main_mock.stop_event.set()
        self.tello_sensor._Tello11111Sensor__thr_sensor.join()
    
    #__init__
    def test_TID3002(self):
        sensor = self.tello_sensor
        self.assertEqual(sensor._Tello11111Sensor__stop_event, self.main_mock.stop_event)
        self.assertEqual(sensor._Tello11111Sensor__main, self.main_mock)
        self.assertEqual(sensor._Tello11111Sensor__planner, self.main_mock.planner)
        self.assertEqual(sensor._Tello11111Sensor__socket, self.main_mock.socket11111)
        self.assertIsInstance(sensor._Tello11111Sensor__thr_sensor, threading.Thread)
    
    #__func_sensor
    def test_TID3003(self):
        data = self.tello_sensor.take_data_from_sensor()
        info = self.tello_sensor.change_data_to_info()
        self.assertEqual(data,info)
        
    # take_data_from_sensor
    def test_TID3004(self):
        self.main_mock.socket11111.recv.return_value = b'\x00\x00\x01\x67'
        self.tello_sensor.take_data_from_sensor()
        self.assertEqual(self.tello_sensor._Tello11111Sensor__packet_data, b'\x00\x00\x01\x67')

    # change_data_to_info
    def test_TID3005(self):
        self.main_mock.socket11111.recv.return_value = b'\x00\x00\x01\x67'
        self.tello_sensor.take_data_from_sensor()
        self.tello_sensor._Tello11111Sensor__packet_data = b'\x00\x00\x01\x67'
        
        with unittest.mock.patch('SD.Decoder.H264decoder.decode') as mock_decode:
            self.tello_sensor.change_data_to_info()
            mock_decode.assert_called_once_with(self.tello_sensor._Tello11111Sensor__decoder, b'\x00\x00\x01\x67')

    # save_to_planner
    def test_TID3006(self):
        test_frame = MagicMock()
        self.tello_sensor.save_to_planner(test_frame)
        self.main_mock.planner.set_info_11111Sensor_frame.assert_called_once_with(test_frame)
    
    #__printc
    def test_TID3007(self):
        expected_output = ""
        with patch('sys.stdout', new=StringIO()) as fake_stdout:
            self.main_mock.planner._Planner__printc("Hello world!")
            self.assertEqual(fake_stdout.getvalue().strip(), expected_output)
    
    #__printf
    def test_TID3008(self):
        expected_output = ""
        with patch('sys.stdout', new=StringIO()) as fake_stdout:
            self.main_mock.planner._Planner__printf("Hello world!", "my_function")
            self.assertEqual(fake_stdout.getvalue().strip(), expected_output)
        
        

#===========================================================================================================    
class TestTelloVirtualController(unittest.TestCase):
    
    def setUp(self):
        self.main_mock = MagicMock()
        self.main_mock.socket8889 = MagicMock()
        self.main_mock.tello_address = ('192.168.10.1', 8889)
        self.main_mock.planner = MagicMock()
        self.main_mock.stop_event = threading.Event()
        self.controller = TelloVirtualController(self.main_mock)

    #__init__
    def test_TID1024(self):
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__socket8889'))
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__tello_address'))
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__planner'))
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__stop_event'))
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__thread_stop_event'))
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__cm'))
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__degree'))
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__renewal_tof_time'))
        self.assertIsNotNone(getattr(self.controller, 'root'))
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__text_tof'))
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__text_keyboard'))
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__btn_landing'))
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__btn_takeoff'))
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__keyboard_connection'))
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__thread_update_tof'))
        self.assertIsNotNone(getattr(self.controller, '_TelloVirtualController__thread_print_video'))
    
    #land
    def test_TID1025(self):
        with patch.object(self.controller, 'send_cmd') as mock_send_cmd:
            self.controller.land()
            mock_send_cmd.assert_called_once_with('land')
    
    #takeoff
    def test_TID1026(self):
        with patch.object(self.controller, 'send_cmd') as mock_send_cmd:
            self.controller.takeoff()
            mock_send_cmd.assert_called_once_with('takeoff')
    
    #on_keypress_q
    def test_TID1027(self):
        with patch.object(self.controller, 'send_cmd') as mock_send_cmd:
            self.controller.on_keypress_q(None)
            mock_send_cmd.assert_called_once_with('stop')
    
    #on_keypress_w
    def test_TID1028(self):
        with patch.object(self.controller, 'send_cmd') as mock_send_cmd:
            self.controller.on_keypress_w(None)
            mock_send_cmd.assert_called_once_with('up 50')
    
    #on_keypress_s
    def test_TID1029(self):
        with patch.object(self.controller, 'send_cmd') as mock_send_cmd:
            self.controller.on_keypress_s(None)
            mock_send_cmd.assert_called_once_with('down 50')
    
    #on_keypress_a
    def test_TID1030(self):
        with patch.object(self.controller, 'send_cmd') as mock_send_cmd:
            self.controller.on_keypress_a(None)
            mock_send_cmd.assert_called_once_with('ccw 50')
    
    #on_keypress_d
    def test_TID1031(self):
        with patch.object(self.controller, 'send_cmd') as mock_send_cmd:
            self.controller.on_keypress_d(None)
            mock_send_cmd.assert_called_once_with('cw 50')
    
    #on_keypress_up
    def test_TID1032(self):
        with patch.object(self.controller, 'send_cmd') as mock_send_cmd:
            self.controller.on_keypress_up(None)
            mock_send_cmd.assert_called_once_with('forward 50')
    
    #on_keypress_down
    def test_TID1033(self):
        with patch.object(self.controller, 'send_cmd') as mock_send_cmd:
            self.controller.on_keypress_down(None)
            mock_send_cmd.assert_called_once_with('back 50')
    
    #on_keypress_left
    def test_TID1034(self):
        with patch.object(self.controller, 'send_cmd') as mock_send_cmd:
            self.controller.on_keypress_left(None)
            mock_send_cmd.assert_called_once_with('left 50')
    
    #on_keypress_right
    def test_TID1035(self):
        with patch.object(self.controller, 'send_cmd') as mock_send_cmd:
            self.controller.on_keypress_right(None)
            mock_send_cmd.assert_called_once_with('right 50')
    
    
    def tmove(self, direction, distance): 
        return "{} {}".format(direction, distance)
    
    
    def trotate(self, direction, degree):
        return "{} {}".format(direction, degree)
        
    #move
    def test_TID1036(self):
        val = self.tmove("up", 50)
        self.assertEqual(val, "up 50")
    
    #rotate
    def test_TID1037(self):
        val = self.trotate("cw", 90)
        self.assertEqual(val, "cw 90")
    
    
    #func_update_tof
    def test_TID1038(self):
        tof =  50
        text = "ToF: {} cm".format(tof)
        self.assertEqual(text, "ToF: 50 cm")

    #func_print_video
    def test_TID1039(self):
        photo = "test"
        # Create label widget with photo image
        self.controller.__panel_image = photo
        # Check that the label widget has the expected photo image
        self.assertEqual(self.controller.__panel_image, photo)

            
    #send_cmd
    def test_TID1040(self):
        with patch.object(self.controller, 'insert_controller_queue') as mock_insert_controller_queue:
            self.controller.send_cmd('test_cmd')
            mock_insert_controller_queue.assert_any_call('test_cmd')
            mock_insert_controller_queue.assert_any_call('stop')
            
    #insert_controller_queue
    def test_TID1041(self):
            mock_cmd = "test_command"
            self.controller.insert_controller_queue(mock_cmd)
            self.controller._TelloVirtualController__planner.insert_cmd_queue.assert_called_with(mock_cmd)    

    #OnClose
    @patch("socket.socket")
    @patch("threading.Event")
    @patch("tkinter.Tk")
    def test_TID1042(self, mock_tk, mock_event, mock_socket):
        # Set required attributes for the controller
        self.controller.__socket8889 = Mock()
        self.controller.__tello_address = ("192.168.10.1", 8889)
        self.controller.__thread_stop_event = mock_event.return_value
        self.controller.__stop_event = mock_event.return_value
        self.controller.root = mock_tk

        # Call the onClose method
        self.controller.__socket8889.sendto("land".encode('utf-8'), self.controller.__tello_address)
        self.controller.__socket8889.sendto("motoroff".encode('utf-8'), self.controller.__tello_address)
        
        #update_tof, print_video를 종료
        self.controller.__thread_stop_event.set()
        
        #모든 스레드 종료 명령인 stop_event를 실행
        self.controller.__stop_event.set()
        
        #화면 종료 
        self.controller.root.quit() 

        # Check if socket.sendto was called with expected arguments
        self.controller.__socket8889.sendto.assert_any_call("land".encode("utf-8"), self.controller.__tello_address)
        self.controller.__socket8889.sendto.assert_any_call("motoroff".encode("utf-8"), self.controller.__tello_address)

        # Check if Event.set was called
        mock_event.return_value.set.assert_called()

        # Check if tkinter.Tk.quit was called
        mock_tk.quit.assert_called()

    #printc
    @patch("builtins.print")
    def test_TID1043(self, mock_print):
        msg = "Sample message"
        self.controller._TelloVirtualController__printc(msg)
        mock_print.assert_called_with("[{}] {}".format(self.controller.__class__.__name__, msg))

    #printf
    @patch("builtins.print")
    def test_TID1044(self, mock_print):
        msg = "Sample message"
        fname = "sample_function"
        self.controller._TelloVirtualController__printf(msg, fname)
        mock_print.assert_called_with("[{}] [{}]: {}".format(self.controller.__class__.__name__, fname, msg))

    #printm
    @patch("builtins.print")
    def test_TID1045(self, mock_print):
        key = "A"
        action = "move"
        self.controller.__cm = 50
        self.controller._TelloVirtualController__printm(key, action)
        mock_print.assert_called_with("[{}] KEYBOARD {}: {} {} cm".format(self.controller.__class__.__name__, key, action, self.controller.__cm))

    #printr
    @patch("builtins.print")
    def test_TID1046(self, mock_print):
        key = "Q"
        action = "rotate"
        self.controller.__degree = 50
        self.controller._TelloVirtualController__printr(key, action)
        mock_print.assert_called_with("[{}] KEYBOARD {}: {} {} degree".format(self.controller.__class__.__name__, key, action, self.controller.__degree))



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

    #__func_planner
    def test_TID4004(self):
        with patch('SD.Plan.Planner5.Planner._Planner__redraw_frame') as draw_image_mock:
            self.planner._Planner__redraw_frame()

            # Assert that the draw_image method is called
            draw_image_mock.assert_called_once()
     
    #__func_stay_connection
    def test_TID1048(self):
        self.planner._Planner__virtual_controller = MagicMock()
        self.planner._Planner__virtual_controller.onClose.side_effect = Exception("test")
        
        self.planner.socket8889.sendto("command".encode(),self.planner.tello_address)

        self.assertFalse(self.planner._Planner__virtual_controller.onClose.called)
    
    #__func_request_tof
    def test_TID2003(self):
        self.planner._Planner__virtual_controller = MagicMock()
        self.planner._Planner__virtual_controller.onClose.side_effect = Exception("test")
        
        self.planner.socket8889.sendto("EXT tof?".encode(),self.planner.tello_address)
        self.assertFalse(self.planner._Planner__virtual_controller.onClose.called)
    
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
   
    #_pop_cmd_queue
    def test_TID1049(self):
        self.assertIsNone(self.planner.pop_cmd_queue())
        self.planner._Planner__cmd_queue = ['command1', 'command2']
        self.assertEqual(self.planner.pop_cmd_queue(), 'command1')
        self.assertEqual(self.planner._Planner__cmd_queue, ['command2'])

    #_insert_cmd_queue
    def test_TID1050(self):
        self.planner._Planner__cmd_queue = []
        self.planner.insert_cmd_queue('command1')
        self.assertEqual(self.planner._Planner__cmd_queue, ['command1'])
        self.planner.insert_cmd_queue('command2')
        self.assertEqual(self.planner._Planner__cmd_queue, ['command1', 'command2'])

    #_get_info_8889Sensor_tof
    def test_TID2004(self):
        self.assertIsNone(self.planner.get_info_8889Sensor_tof())
        self.planner.set_info_8889Sensor_tof(100)
        self.assertEqual(self.planner.get_info_8889Sensor_tof(), 100)

    #_set_info_8889Sensor_tof
    def test_TID2005(self):
        self.assertIsNone(self.planner.get_info_8889Sensor_tof())
        self.planner.set_info_8889Sensor_tof(100)
        self.assertEqual(self.planner._Planner__info_8889Sensor_tof, 100) 
    
    #_get_info_8889Sensor_cmd
    def test_TID1051(self):
        self.assertIsNone(self.planner.get_info_8889Sensor_cmd())
        self.planner._Planner__info_8889Sensor_cmd = 'ready'
        self.assertEqual(self.planner.get_info_8889Sensor_cmd(), 'ready')

    #_set_info_8889Sensor_cmd
    def test_TID1052(self):
        self.assertIsNone(self.planner._Planner__info_8889Sensor_cmd)
        self.planner.set_info_8889Sensor_cmd('ready')
        self.assertEqual(self.planner._Planner__info_8889Sensor_cmd, 'ready')

    #get_info_11111Sensor_frame
    def test_TID3012(self):
        self.assertIsNone(self.planner.get_info_11111Sensor_frame())
        self.planner._Planner__info_11111Sensor_frame = 'ready'
        self.assertEqual(self.planner.get_info_11111Sensor_frame(), 'ready')

    #set_info_11111Sensor_frame
    def test_TID3013(self):
        self.assertIsNone(self.planner._Planner__info_11111Sensor_frame)
        self.planner.set_info_11111Sensor_frame('ready')
        self.assertEqual(self.planner._Planner__info_11111Sensor_frame, 'ready')
        
    #get_info_11111Sensor_image
    def test_TID3014(self):
        self.assertIsNone(self.planner.get_info_11111Sensor_image())
        self.planner._Planner__info_11111Sensor_image = 'ready'
        self.assertEqual(self.planner.get_info_11111Sensor_image(), 'ready')

    #set_info_11111Sensor_image
    def test_TID3015(self):
        self.assertIsNone(self.planner._Planner__info_11111Sensor_image)
        self.planner.get_info_11111Sensor_image()
        self.assertEqual(self.planner._Planner__info_11111Sensor_image, None)      

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
    
    #__printc
    def test_TID1053(self):
        expected_output = "[Planner] Hello world!"
        with patch('sys.stdout', new=StringIO()) as fake_stdout:
            self.planner._Planner__printc("Hello world!")
            self.assertEqual(fake_stdout.getvalue().strip(), expected_output)
    
    #_printf
    def test_TID1054(self):
        expected_output = "[Planner] [my_function]: Hello world!"
        with patch('sys.stdout', new=StringIO()) as fake_stdout:
            self.planner._Planner__printf("Hello world!", "my_function")
            self.assertEqual(fake_stdout.getvalue().strip(), expected_output)
      
        

#===========================================================================================================  
class TestYOLOv5(unittest.TestCase):
    @patch('SD.ObjectDetector.YOLOv5.torch.hub.load')
    def setUp(self, mock_torch_hub_load):
        self.yolov5 = YOLOv5()
        labels = np.array([1])
        coors = np.array([[2, 3, 4, 5]])
        self.yolov5._YOLOv5__model = MagicMock()
        self.yolov5._YOLOv5__model.return_value = MagicMock(xyxyn=MagicMock(return_value=(labels, coors)))
        import torch
        self.mock_torch_hub_load = mock_torch_hub_load
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.image = np.zeros((720, 1280, 3), dtype=np.uint8)
        labels = np.array([1])
        coors = np.array([[2, 3, 4, 5]])

    #__init__
    def test_TID4006(self):
        # Test the __init__ method
        self.mock_torch_hub_load.assert_called_once()

    #detect_from_frame
    def test_TID4007(self):
        # Test the detect_from_frame method
        test_frame = np.random.randint(0, 256, (480, 640, 3), dtype=np.uint8)
        tof = 25

        with patch.object(self.yolov5, '_YOLOv5__score_frame', return_value=([], [])) as mock_score_frame:
            image, fusion_window_coor = self.yolov5.detect_from_frame(test_frame, tof)

            mock_score_frame.assert_called_once_with(test_frame)

            self.assertIsNotNone(image)
            self.assertIsNotNone(fusion_window_coor)

    #__calculate_ir_window_coor_tof_none
    def test_TID4008(self):
        # Test the __calculate_ir_window_coor method with tof=None
        tof = None
        height = 480
        width = 640
        expected_result = (None, None)
        result = self.yolov5._YOLOv5__calculate_ir_window_coor(tof, height, width)
        self.assertEqual(result, expected_result)
    
    #__score_frame
    def test_TID4009(self):
        labels, coors = self.yolov5._YOLOv5__score_frame(self.image)
        self.assertNotEqual(labels.size, 0)
        self.assertNotEqual(labels.ndim, 1)
        self.assertNotEqual(coors.ndim, 2)
        self.assertNotEqual(coors.shape[1], 4)
        



#===========================================================================================================    
class TestCheckValues(unittest.TestCase):
    
    #test_is_tof_val
    def test_TID1055(self):
        # Test for valid tof value
        valid_tof_val = "tof 500"
        self.assertTrue(is_tof_val(valid_tof_val))

        # Test for tof value below range
        tof_val_below_range = "tof -10"
        self.assertFalse(is_tof_val(tof_val_below_range))

        # Test for tof value above range
        tof_val_above_range = "tof 9000"
        self.assertFalse(is_tof_val(tof_val_above_range))

        # Test for invalid format
        invalid_format = "tof"
        self.assertFalse(is_tof_val(invalid_format))

        # Test for non-tof value
        non_tof_val = "battery?"
        self.assertFalse(is_tof_val(non_tof_val))

    #test_is_sdk_val
    def test_TID1056(self):
        # Test for valid SDK value
        valid_sdk_val = "command"
        self.assertTrue(is_sdk_val(valid_sdk_val))



#===========================================================================================================  
class TestChangeValues(unittest.TestCase):
    
    #test_change_mm_to_cm
    def test_TID1057(self):
        self.assertEqual(change_mm_to_cm(0), 0)
        self.assertEqual(change_mm_to_cm(5), 0)
        self.assertEqual(change_mm_to_cm(10), 1)
        self.assertEqual(change_mm_to_cm(15), 1)
        self.assertEqual(change_mm_to_cm(100), 10)
        self.assertEqual(change_mm_to_cm(1000), 100)
        self.assertEqual(change_mm_to_cm(12345), 1234)
    
    #test_change_val_to_coor
    def test_TID1058(self):
        object_val3 = (None, ((0, 0), (100, 100)), (640, 480))
        expected_output3 = None
        self.assertEqual(change_val_to_coor(object_val3), expected_output3)
        
    #test_change_cmd_for_tello
    def test_TID1059(self):
        cmd1 = "left 100"
        expected_output2 = b'rc -100 0 0 0'
        self.assertEqual(change_cmd_for_tello(cmd1), expected_output2)
        
        cmd2 = "stop"
        expected_output3 = b'rc 0 0 0 0'
        self.assertEqual(change_cmd_for_tello(cmd2), expected_output3)

        cmd3 = "fly"
        expected_output4 = b'fly'
        self.assertEqual(change_cmd_for_tello(cmd3), expected_output4)
        
        cmd4 = "right 150"
        expected_output5 = b'rc 100 0 0 0'
        self.assertEqual(change_cmd_for_tello(cmd4), expected_output5)
        
        cmd5 = "back 20"
        expected_output6 = b'rc 0 -60 0 0'
        self.assertEqual(change_cmd_for_tello(cmd5), expected_output6)

    #test_change_windows_to_window
    def test_TID1060(self):
        window_list1 = [((100, 100), (200, 200))]
        ir_left_up_coor1 = (150, 150)
        ir_right_down_coor1 = (250, 250)
        expected_output1 = ((100, 100), (200, 200))
        self.assertEqual(change_windows_to_window(window_list1, ir_left_up_coor1, ir_right_down_coor1), expected_output1)

        window_list2 = [((100, 100), (200, 200))]
        ir_left_up_coor4 = (150, 0)
        ir_right_down_coor4 = (250, 100)
        expected_output4 = ((100, 100), (200, 200))
        self.assertEqual(change_windows_to_window(window_list2, ir_left_up_coor4, ir_right_down_coor4), expected_output4)
        
        window_list3 = []
        ir_left_up_coor5 = (100, 100)
        ir_right_down_coor5 = (200, 200)
        expected_output5 = ((-131072, -131072), (131072, 131072))
        self.assertEqual(change_windows_to_window(window_list3, ir_left_up_coor5, ir_right_down_coor5), expected_output5)
    
    #change_to_safe_cmd
    def test_TID1061(self):
        cmd1 = "forward 30"
        tof1 = 150
        threshold1 = 50
        expected1 = "forward 70"
        self.assertEqual(change_to_safe_cmd(cmd1, tof1, threshold1), expected1)

        cmd2 = "forward 50"
        tof2 = 1000
        threshold2 = 50
        expected2 = "forward 50"
        self.assertEqual(change_to_safe_cmd(cmd2, tof2, threshold2), expected2)
        
        cmd3 = "forward 50"
        tof3 = 2000
        threshold3 = 50
        expected3 = "forward 50"
        self.assertEqual(change_to_safe_cmd(cmd3, tof3, threshold3), expected3)

        cmd4 = "turn_left 30"
        tof4 = 150
        threshold4 = 50
        expected4 = "turn_left 30"
        self.assertEqual(change_to_safe_cmd(cmd4, tof4, threshold4), expected4)



#===========================================================================================================  
class TestDecode(unittest.TestCase):
    
    def setUp(self):
        self.decoder = h264decoder.H264Decoder()
        
    def test_TID3016(self):
        # 정상적인 raw H.264 data가 들어왔을 때
        packet_data = b'\x00\x00\x00\x01\x67\x4d\x40\x0c\x03\x00\x00\x03\x00\xf8\x00\x00\x00\x01\x68\xee\x3c\x80'
        frames = H264decoder.decode(self.decoder, packet_data)
        self.assertEqual(len(frames), 0)


#===========================================================================================================                     
class TestScenario_control(unittest.TestCase):
    
    def setUp(self):
        self.__cmd_queue = [] #������ ������ ť
        
    def on_keypress_test(self, test_key, test_direction):
        print(test_key,test_direction)
        self.move(test_direction,50)
    
    def takeoff(self): #return: Tello�� receive 'OK' or 'FALSE'
         self.send_cmd('takeoff')
    
    def land(self): #return: Tello�� receive 'OK' or 'FALSE'
        self.send_cmd('land')
        
    def move(self, direction, distance): 
        self.send_cmd("{} {}".format(direction, distance))

    def send_cmd(self, msg:str):
        # self.__lock.acquire() #�� ȹ��
        try:
            self.insert_controller_queue(msg)
            self.insert_controller_queue("stop")

        except Exception as e:
            print("ERROR {}".format(e),sys._getframe().f_code.co_name)
        # self.__lock.release() #�� ����
    
    def insert_controller_queue(self,cmd):
        self.insert_cmd_queue(cmd)
    
    def insert_cmd_queue(self, info):
        # self.__lock_cmd_queue.acquire()
        self.__cmd_queue.append(info)
        # self.__lock_cmd_queue.release()
    
    def pop_cmd_queue(self):
        # self.__lock_cmd_queue.acquire()
        data = None
        if len(self.__cmd_queue)>0:
            data = self.__cmd_queue.pop(0)
        return data

    def take_cmd_from_planner(self): 
        cmd = self.pop_cmd_queue()
        return cmd
    
    def test_TID1062(self):
        self.on_keypress_test('w', 'up')
        cmd = self.take_cmd_from_planner()
        self.assertEqual('up 50',cmd)
    
    def test_TID1063(self):
        self.on_keypress_test('a', 'ccw')
        cmd = self.take_cmd_from_planner()
        self.assertEqual('ccw 50',cmd)
    
    def test_TID1064(self):
        self.on_keypress_test('d', 'cw')
        cmd = self.take_cmd_from_planner()
        self.assertEqual('cw 50',cmd)
    
    def test_TID1065(self):
        self.on_keypress_test('s', 'down')
        cmd = self.take_cmd_from_planner()
        self.assertEqual('down 50',cmd)
    
    def test_TID1066(self):
        self.takeoff()
        cmd = self.take_cmd_from_planner()
        self.assertEqual('takeoff',cmd)
    
    def test_TID1067(self):
        self.land()
        cmd = self.take_cmd_from_planner()
        self.assertEqual('land',cmd)
    
    def test_sleep(self):
        import time
        time.sleep(1)
        


#===================================================
class TestScenario_tof(unittest.TestCase):
    def setUp(self):
        self.__info_8889Sensor_tof = None
        self.__info_8889Sensor_cmd = None

        
    def take_data_from_sensor(self, data): 
        """
        센서로부터 data를 가져온다
        """
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
            self.set_info_8889Sensor_tof(info)
        
        else: #cmd return 값이면
            self.set_info_8889Sensor_cmd(info)
            print("[Tello8889Sensor]",info)
    
    def set_info_8889Sensor_cmd(self, info):
        # self.__lock_info_8889Sensor_cmd.acquire()
        self.__info_8889Sensor_cmd = info
        # self.__lock_info_8889Sensor_cmd.release()
    
    def set_info_8889Sensor_tof(self, info):
        # self.__lock_info_8889Sensor_tof.acquire()
        self.__info_8889Sensor_tof = info
        # self.__lock_info_8889Sensor_tof.release()
        
    def get_info_8889Sensor_tof(self):
        # self.__lock_info_8889Sensor_tof.acquire()
        info = self.__info_8889Sensor_tof
        # self.__lock_info_8889Sensor_tof.release()
        return info
    
    def test_TID2006(self):
        data = self.take_data_from_sensor(b'tof 500')
        info = self.change_data_to_info(data)
        self.save_to_planner(info)
        tof = self.get_info_8889Sensor_tof()
        self.assertEqual(50, tof)
    
    def test_TID2007(self):
        data = self.take_data_from_sensor(b'tof 1000')
        info = self.change_data_to_info(data)
        self.save_to_planner(info)
        tof = self.get_info_8889Sensor_tof()
        self.assertEqual(1000, tof)
    
    def test_TID2008(self):
        data = self.take_data_from_sensor(b'tof 2000')
        info = self.change_data_to_info(data)
        self.save_to_planner(info)
        tof = self.get_info_8889Sensor_tof()
        self.assertEqual(1000, tof)
        
        
        
#===========================================================================================================                      
class TestScenario_camera(unittest.TestCase):
    
    def setUp(self):
        self.__packet_data = bytes()
        self.__info_11111Sensor_frame = None
        
    def take_data_from_sensor(self, data): 
        data:bytes = data
        self.__packet_data += bytes(data)
    
    def save_to_planner(self, info):
        self.set_info_11111Sensor_frame(info)
        
    def set_info_11111Sensor_frame(self, info):
        # self.__lock_info_11111Sensor_frame.acquire()
        self.__info_11111Sensor_frame = info
        # self.__lock_info_11111Sensor_frame.release()
    
    def get_info_11111Sensor_image(self):
        # self.__lock_info_11111Sensor_image.acquire()
        info = self.__info_11111Sensor_frame
        # self.__lock_info_11111Sensor_image.release()
        return info
    
    def test_TID3017(self):
        info = self.take_data_from_sensor(b'test')
        self.save_to_planner(info)
        image = self.get_info_11111Sensor_image()
        self.assertEqual(None,image)     
    
    def test_TID3018(self):
        info = self.take_data_from_sensor(bytes())
        self.save_to_planner(info)
        image = self.get_info_11111Sensor_image()
        self.assertEqual(None,image)    
        
        

#===========================================================================================================                     
class TestScenario_yolo(unittest.TestCase):
    
    def setUp(self):
       self.__info_11111Sensor_image = None
    
    def detect_from_frame(self, frame, tof): 
        return (frame, tof)
        
    def set_info_11111Sensor_image(self, info):
        # self.__lock_info_11111Sensor_image.acquire()
        self.__info_11111Sensor_image = info
        # self.__lock_info_11111Sensor_image.release()
    
    def get_info_11111Sensor_image(self):
        # self.__lock_info_11111Sensor_image.acquire()
        info = self.__info_11111Sensor_image
        # self.__lock_info_11111Sensor_image.release()
        return info
        
    def test_TID4008(self):
        image, _ = self.detect_from_frame('test', 50)
        self.set_info_11111Sensor_image(image)
        result = self.get_info_11111Sensor_image()
        self.assertEqual('test',result)
      
    def test_TID4009(self):
        image, _ = self.detect_from_frame(None, 50)
        self.set_info_11111Sensor_image(image)
        result = self.get_info_11111Sensor_image()
        self.assertEqual(None,result)

   
#===================================================
class TestScenario_CAD(unittest.TestCase):
    def setUp(self):
        pass
    
    def test_TID5008(self):
        pass
    
    def test_TID5009(self):
        pass
    
    def test_TID5010(self):
        pass
        
        
        
#===========================================================================================================                             
if __name__ == "__main__":
    unittest.main()