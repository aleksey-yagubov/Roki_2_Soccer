from picamera2 import Picamera2
from libcamera import controls
import cv2
import time

class Camera:
    def __init__(self):
        self.frame_number_counter = None
        self.picam2 = None
        self.last_frame_number = 0   
        self.last_frame_time = 0    # frame timestamp in us
        # Legacy name; this is now the single main stream size.
        self.camera_lores = (800, 650)
 
    class Frame_number_counter:
        def __init__(self):
            self.list_of_Timestamps = []
            self.head_of_Timestamps = 0
            self.last_processed_frame_number = -1
            self.target_frame_duration = 16700
            
        def add_farme_number(self, request):
            if self.target_frame_duration * 0.98 < request.get_metadata()['FrameDuration'] < self.target_frame_duration * 1.02 :
                self.list_of_Timestamps.append(request.get_metadata()['SensorTimestamp'])
                overlap = self.last_processed_frame_number - self.head_of_Timestamps
                if overlap >= 0:
                    self.list_of_Timestamps = self.list_of_Timestamps[(overlap + 1):]
                    self.head_of_Timestamps += overlap + 1
                # if len(self.list_of_Timestamps) > 100:
                #     self.list_of_Timestamps = self.list_of_Timestamps[40:]
                #     self.head_of_Timestamps += 40
                
        def do_nothing(self, request):
            pass

    def start(self, exposure = None, gain = None, frame_duration_us = 16700, neural = False):
        self.picam2 = Picamera2(camera_num=0)
        self.frame_number_counter = self.Frame_number_counter()
        self.frame_number_counter.target_frame_duration = frame_duration_us
        self.picam2.pre_callback = self.frame_number_counter.add_farme_number
        config = self.picam2.create_video_configuration(
            # В Picamera2/libcamera RGB888 в Python приходит как BGR-массив, удобный для OpenCV.
            main={"format": "RGB888", "size": self.camera_lores},
            display=None,
        )
        self.picam2.configure(config)
        #if neural:
        #    self.picam2.set_controls({"AeExposureMode":  controls.AeExposureModeEnum.Short})
        #else:
        #    if exposure != None:
        #        self.picam2.set_controls({"ExposureTime": exposure})		# exposure limits: 11 ~ 199953
        #    if gain != None:
        #        self.picam2.set_controls({"AnalogueGain": gain})
        if exposure != None:
            self.picam2.set_controls({"ExposureTime": exposure})		# exposure limits: 11 ~ 199953
        if gain != None:
            self.picam2.set_controls({"AnalogueGain": gain})
        self.picam2.set_controls({"FrameDurationLimits": (frame_duration_us*2, frame_duration_us*2)})
        self.picam2.start()
        time.sleep(1)
        self.picam2.set_controls({"FrameDurationLimits": (frame_duration_us, frame_duration_us)})
        if not neural: self.picam2.set_controls({"AwbEnable": False})
        for _ in range(50):
            if len(self.frame_number_counter.list_of_Timestamps) > 0: break
            time.sleep(0.02)
        self.picam2.pre_callback = self.frame_number_counter.do_nothing
        for _ in range(50):
            request = self.picam2.capture_request()
            if request:
                frame_duration = request.get_metadata()['FrameDuration']
                if (frame_duration_us - 1000) < frame_duration < (frame_duration_us + 1000) :
                    frame_timestamp = request.get_metadata()['SensorTimestamp']
                    self.last_frame_time = frame_timestamp / 1000 
                    self.last_frame_number = 2
                    return True
            time.sleep(0.02)
        return False
        
    def snapshot(self):
        request = self.picam2.capture_request()
        frame_timestamp = request.get_metadata()['SensorTimestamp']
        frame_duration = request.get_metadata()['FrameDuration']
        # try:
        #     frame_number = self.frame_number_counter.list_of_Timestamps.index(frame_timestamp)\
        #                     + self.frame_number_counter.head_of_Timestamps
        #     total_number_of_frames = len(self.frame_number_counter.list_of_Timestamps) + self.frame_number_counter.head_of_Timestamps
        #     self.frame_number_counter.last_processed_frame_number = frame_number
        # except Exception:
        #     frame_number = None
        #     total_number_of_frames = None
        frame_number = self.last_frame_number + int(round((frame_timestamp / 1000 - self.last_frame_time) / frame_duration))
        self.last_frame_number = frame_number
        self.last_frame_time = frame_timestamp / 1000
        #print('frame_number: ', frame_number )
        image = request.make_array("main")
        request.release()
        width, height = self.camera_lores
        image = image[0:height,0:width,0:3]
        #cv2.imshow("Camera", image)
        #cv2.waitKey(10)
        return image, frame_number
    
    def stop(self):
        self.picam2.stop()

if __name__ == '__main__':
    from multiprocessing import Array, Process, Value
    import numpy as np
    frame = Array('B', 1560000)
    trigger = Value('i', 0)
    def display(frame, trigger):
        while True:
            if trigger.value:
                image = np.frombuffer(frame.get_obj(), dtype=np.uint8)
                image = image.reshape(650,800,3)
                cv2.imshow("Camera2", image)
                cv2.waitKey(10)
            time.sleep(0.01)
    
    
    process = Process(target=display, args=(frame, trigger,))
    process.start()
    #process.join()
    #from class_stm_channel import STM_channel
    #stm_channel = STM_channel()
    #stm_channel.mb.ResetIMUCounter()
    camera_frame_timestamps = []    
    single_quat_timestamps =[]
    camera = Camera()
    #quat = stm_channel.read_quaternion_from_imu_in_head(frame_number = None)
    #single_quat_timestamps.append(quat[4] + round(quat[5]/1000000000, 3))
    camera.start() #(exposure = 100000)         # exposure limits: 11 ~ 199953
    start_time = time.perf_counter()
    
    cycles = 150
    for _ in range(cycles):
#         quat = stm_channel.read_quaternion_from_imu_in_head(frame_number = None)
#         single_quat_timestamps.append(quat[4] + round(quat[5]/1000000000, 3))
        image, frame_number = camera.snapshot()
        # im = np.frombuffer(frame.get_obj(), dtype=np.uint8)
        # im = im.reshape(650,800,3)
        # im = image
        frame.value = np.ndarray.tobytes(image)
        trigger.value = 1
        print(len(image), image.shape, image.dtype)
        #quat = stm_channel.read_quaternion_from_imu_in_head(frame_number = frame_number)
        #camera_frame_timestamps.append(round(frame_timestamp/1000000000, 3))
        #print('frame_number: ', frame_number )
        cv2.imshow("Camera", image)
        cv2.waitKey(10)

    #FRAME_RATE = 3
    #FRAME_RATE = int(1000000 // FRAME_RATE)
    #camera.picam2.set_controls({"FrameDurationLimits":(FRAME_RATE,FRAME_RATE)})
    #time.sleep(2)
    
    #for _ in range(cycles):
#         quat = stm_channel.read_quaternion_from_imu_in_head(frame_number = None)
#         single_quat_timestamps.append(quat[4] + round(quat[5]/1000000000, 3))
        #image, frame_number = camera.snapshot()
        #quat = stm_channel.read_quaternion_from_imu_in_head(frame_number = frame_number)
        #camera_frame_timestamps.append(round(frame_timestamp/1000000000, 3))
        #print('frame_number: ', frame_number )
#         cv2.imshow("Camera", image)
#         cv2.waitKey(10)
    time_elapsed = time.perf_counter() - start_time
    camera.stop()
    #print('frame_number: ', frame_number, 'total_number_of_frames: ', total_number_of_frames, 'frame_timestamp: ', frame_timestamp)
    print('Rate : ', int(cycles/ time_elapsed), ' FPS')
    #cv2.destroyAllWindows()
    #print('total number of frames: ', len(camera.frame_number_counter.list_of_Timestamps) + camera.frame_number_counter.head_of_Timestamps)
    #print(camera.frame_number_counter.list_of_Timestamps)
    # for i in range(300):
    #     try:
    #         #print(i)
    #         quat = stm_channel.read_quaternion_from_imu_in_head(frame_number = i)
    #     except Exception: break
    # print('IMU records in stm: ', i)
#         timestamp = quat[4] + round(quat[5]/1000000000, 3)
#         print(timestamp, i)
        #time.sleep(1)
#     for timestamp in single_quat_timestamps:
#         print('single_quat_timestamps:', timestamp)
#     for timestamp in camera_frame_timestamps:
#         print('camera_frame_timestamps:', timestamp)
