# Import packages
import queue
import numpy as np
import cv2
import time

import olympe
from olympe.messages import gimbal
import olympe.messages.ardrone3.Piloting as piloting
from olympe.messages.common.CommonState import BatteryStateChanged
from olympe.messages.ardrone3.PilotingState import (
    PositionChanged,
    FlyingStateChanged,
    AttitudeChanged
    # AlertStateChanged,
    # NavigateHomeStateChanged,
    # GpsLocationChanged
)


class AiANAFI:
    """
    This class provides API to control the DJI Tello Drone
    """

    # Takeoff Altitude (cm)
    takeoff_Alt = 150
    max_Alt = 500
    min_Alt = 100

    command_time = 0.5

    droneState = None
    drone = None

    droneIP = None

    DRONE_RTSP_PORT = 554

    def __init__(self, droneIP):
        """
        Initializes the object

        @param self:
        @param droneIP: The IP address of the drone
        """

        self.droneIP = droneIP
        self.drone = olympe.Drone(self.droneIP)

        self.frameQueue = queue.Queue()

    def connect(self):
        """
        Connects to the drone

        @param self:

        @return: True on success, False otherwise
        """

        self.drone.connect()
        return True

    def disconnect(self):
        """
        Disconnects from the drone

        @param self:

        @return: True on success, False otherwise
        """

        self.drone.disconnect()
        return True

    def arm(self):
        """
        Arm the drone

        Condition: check if drone is connected before arming.

        @param self:

        @return: True on success, False otherwise
        """

        self.drone.start_piloting()
        return True

    def disarm(self):
        """
        Disarm the drone

        Condition: check if drone has landed before disarming.

        @param self:

        @return: True on success, False otherwise
        """

        self.drone.stop_piloting()
        return True

    def takeoff(self):
        """
        Take off the drone to an altitude = takeoff_Alt

        Condition: check if drone is armed before taking off.

        @param self:

        @return: True on success, False otherwise
        """

        assert self.drone(
            piloting.TakeOff()
            >> FlyingStateChanged(state="hovering", _timeout=10)
        ).wait().success()
        return True

    def land(self):
        """
        Land the drone

        Condition:

        @param self:

        @return: True on success, False otherwise
        """

        assert self.drone(piloting.Landing()).wait().success()
        return True

    def rcc(self, LR, FB, UD, Yaw, speedCap):
        """
        Update the motor status according to the remote control command to provided value

        Condition: Keep the altitude between min_Alt and max_Alt

        @param self
        @param LR: Left (+) and Right (-)
        @param FB: Forward (+) and Backward (-)
        @param UD: Upward (+) and Downward (-)
        @param Yaw: Left (+) and Right (-)
        @param speedCap: Set a cap on the maximum speed (e.g. 50 means 50% of speed, 100% means full speed)

        @return: True on success, False otherwise
        """

        # Limit the input values from -1 to 1
        valList = [LR, FB, UD, Yaw]
        LR, FB, UD, Yaw = np.clip(valList, -1, 1)
        # print(f'[DEBUG] Limited Range = {LR}, {FB}, {UD}, {Yaw}')

        # Scale the values according to the Speed Cap set by the user
        LR, FB, UD, Yaw = np.multiply([LR, FB, UD, Yaw], speedCap)
        # print(f'[DEBUG] Scaled Range = {LR}, {FB}, {UD}, {Yaw}')

        # Convert to integer
        LR, FB, UD, Yaw = int(LR), int(FB), int(UD), int(Yaw)
        # print(f'[DEBUG] Integer Range = {LR}, {FB}, {UD}, {Yaw}')

        self.drone.piloting(-LR, FB, -Yaw, UD, self.command_time)
        return True

    def moveUp(self, distance):
        """
        Increase altitude by requested distance

        Condition: limit the altitude to max_Alt

        @param self
        @param distance: Go up by a distance = distance cm

        @return: True on success, False otherwise
        """

        raise NotImplementedError("Function Not Implemented!")

    def moveDown(self, distance):
        """
        Decrease altitude by requested distance

        Condition: limit the altitude to min_Alt

        @param self
        @param distance: Go up by a distance = distance cm

        @return: True on success, False otherwise
        """

        raise NotImplementedError("Function Not Implemented!")

    def moveFwd(self, distance):
        """
        Go forward by requested distance

        Condition:

        @param self
        @param distance: Move forward by a distance = distance cm

        @return: True on success, False otherwise
        """

        raise NotImplementedError("Function Not Implemented!")

    def moveBwd(self, distance):
        """
        Go Backward by requested distance

        Condition:

        @param self
        @param distance: Move backward by a distance = distance cm

        @return: True on success, False otherwise
        """

        raise NotImplementedError("Function Not Implemented!")

    def moveLeft(self, distance):
        """
        Go Left by requested distance

        Condition:

        @param self
        @param distance: Move left by a distance = distance cm

        @return: True on success, False otherwise
        """

        raise NotImplementedError("Function Not Implemented!")

    def moveRight(self, distance):
        """
        Go Right by requested distance

        Condition:

        @param self
        @param distance: Move right by a distance = distance cm

        @return: True on success, False otherwise
        """

        raise NotImplementedError("Function Not Implemented!")

    def moveCW(self, angle):
        """
        Perform clockwise Yaw by requested angle (degrees)

        Condition:

        @param self
        @param angle: Clockwise yaw by an angle = angle degrees

        @return: True on success, False otherwise
        """

        raise NotImplementedError("Function Not Implemented!")

    def moveCCW(self, angle):
        """
        Perform counter-clockwise Yaw by requested angle (degrees)

        Condition:

        @param self
        @param angle: Counter-clockwise yaw by an angle = angle degrees

        @return: True on success, False otherwise
        """

        raise NotImplementedError("Function Not Implemented!")

    def getBattery(self):
        """
        Returns the battery status

        Condition:

        @param self

        @return: The battery status
        """

        return self.drone.get_state(BatteryStateChanged)["percent"]

    def getAltitude(self):
        """
        Returns the drone altitude

        Condition:

        @param self

        @return: The drone altitude
        """

        return self.drone.get_state(PositionChanged)["altitude"]

    def getGPSPosition(self):
        """
        Returns the droneGPS position

        Condition:

        @param self

        @return: The drone GPS position as a Tuple (Latitude, Longitude, Altitude)
        """

        # Check if the drone is connected, otherwise return null
        if not self.isConnected():
            return 0, 0, 0

        try:
            Latitude = self.drone.get_state(PositionChanged)["latitude"]
            Longitude = self.drone.get_state(PositionChanged)["longitude"]
            Altitude = self.drone.get_state(PositionChanged)["altitude"]

            return Latitude, Longitude, Altitude
        except Exception as e:
            print("[ERROR] Unable to retrieve GPT position: ({})".format(e))

            return 0, 0, 0

    def getHeading(self):
        """
        Calculates and returns the heading (in degrees) based on the drone's yaw angle

        @param self

        @return: The heading in degrees as a float
        """

        # Check if the drone is connected, otherwise return null
        if not self.isConnected():
            return 0

        try:
            attitude_data = self.drone.get_state(AttitudeChanged)
            yaw = attitude_data["yaw"]
            if yaw < 3.1 and yaw >= 0:
                heading = yaw * 58.064
            elif yaw < 0 and yaw > -3.1:
                heading = 360 + (yaw * 58.064)
            else:
                heading = 180
            return heading
        except Exception as e:
            print("[ERROR] Unable to calculate the heading: ({})".format(e))
            return 0

        ### Alternatively
        # flightData = self.drone.get_state(olympe.messages.flight_data)
        # return flightData['heading']

    def getToFDistance(self):
        """
        This is not clear!!!

        Condition:

        @param self

        @return: ????
        """

        raise NotImplementedError("Function Not Implemented!")

    def isConnected(self):
        """
        Returns the connection status of the drone

        Condition:

        @param self

        @return: True if connected, False otherwise
        """

        return self.drone.connection_state()

    def isArmed(self):
        """
        Returns the arm status of the drone

        Condition:

        @param self

        @return: True if armed, False otherwise
        """

        raise NotImplementedError("Function Not Implemented!")

    def cameraStream_ON(self):
        """
        Starting camera stream

        Condition:

        @param self

        @return: Status
        """

        self.drone.streaming.server_addr = f"{self.droneIP}:{self.DRONE_RTSP_PORT}"

        self.drone.streaming.set_callbacks(
            raw_cb=self.yuv_frame_cb,
            h264_cb=self.h264_frame_cb,
            start_cb=self.start_cb,
            end_cb=self.end_cb,
            flush_raw_cb=self.flush_cb,
        )

        self.drone.streaming.start()

        return True

    def yuv_frame_cb(self, yuv_frame):
        """
        This function will be called by Olympe for each decoded YUV frame.

        @param yuv_frame: olympe.VideoFrame
        """
        yuv_frame.ref()
        self.frameQueue.put_nowait(yuv_frame)

    def flush_cb(self, stream):
        """
        This function is called to flush the frames queue

        @param stream: Stream to flush
        """

        if stream["vdef_format"] != olympe.VDEF_I420:
            return True
        while not self.frameQueue.empty():
            self.frameQueue.get_nowait().unref()
        return True

    def start_cb(self):
        pass

    def end_cb(self):
        pass

    def h264_frame_cb(self, h264_frame):
        pass

    def cameraStream_OFF(self):
        """
        Stopping camera stream

        Condition:

        @param self

        @return: Status
        """

        assert self.drone.streaming.stop()

        return True

    def getFrame(self):
        """
        Capturing a frame

        Condition:

        @param self

        @return: Status and Frame
        """

        try:
            yuv_frame = self.frameQueue.get(timeout=0.1)
        except queue.Empty:
            return None

        # convert pdraw YUV flag to OpenCV YUV flag
        cv2_cvt_color_flag = {
            olympe.VDEF_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.VDEF_NV12: cv2.COLOR_YUV2BGR_NV12,
        }[yuv_frame.format()]

        # yuv_frame.as_ndarray() is a 2D numpy array with the proper "shape"
        # i.e (3 * height / 2, width) because it's a YUV I420 or NV12 frame

        # Use OpenCV to convert the yuv frame to RGB
        cv2frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)  # noqa

        yuv_frame.unref()

        return cv2frame

    def gimbalDown(self):
        """
        Tilts the gimbal downwards (Down facing camera)

        Condition:

        @param self

        @return: Status
        """
        # Adjust the gimbal such that the camera is facing downwards
        gimbal_set_target = self.drone(gimbal.set_target(
            gimbal_id=0,
            control_mode="position",
            yaw_frame_of_reference="none",
            yaw=0,
            pitch_frame_of_reference="absolute",
            pitch=-90,
            roll_frame_of_reference="none",
            roll=0,
        )).wait()
        gimbal_set_target.success()

        # print(gimbal_set_target.explain())

    def gimbalUp(self):
        """
        Tilts the gimbal upwards (front facing camera)

        Condition:

        @param self

        @return: Status
        """

        # Adjust the gimbal such that the camera is facing forward
        gimbal_set_target = self.drone(gimbal.set_target(
            gimbal_id=0,
            control_mode="position",
            yaw_frame_of_reference="none",
            yaw=0,
            pitch_frame_of_reference="absolute",
            pitch=0,
            roll_frame_of_reference="none",
            roll=0,
        )).wait()
        gimbal_set_target.success()


def main():
    DRONE_IP = "10.202.0.1"
    # DRONE_IP = "192.168.42.1"
    drone = AiANAFI(DRONE_IP)
    print("[INFO] Drone object is created!")

    drone.connect()
    print("[INFO] Drone is connected")
    time.sleep(2)

    batteryLevel = drone.getBattery()
    print(f'[INFO] Battery Status = {batteryLevel}%')

    '''
    while True:
        print("[INFO] Attempting to get the GPS position of the drone")
        drone.getGPSPosition()
        print("[INFO] DONE!!!")
        time.sleep(1)

        if cv2.waitKey(10) == ord('q'):
            break

    '''

    if drone.arm():
        print("[INFO] Drone is Armed")
    else:
        print("[ERROR] Drone arming failed. Exiting")
        drone.disconnect()
        exit()
    time.sleep(1)

    drone.gimbalDown()
    time.sleep(10)
    print("[INFO] Gimbal adjusted!")

    drone.takeoff()
    print("[INFO] Drone took off")
    time.sleep(3)

    altitude = drone.getAltitude()
    print(f"Current altitude: {altitude} meters")

    drone.cameraStream_ON()

    while True:
        image = drone.getFrame()

        if image is None:
            continue

        print(f'Width x Height = {image.shape[1]} x {image.shape[0]}')
        image = cv2.resize(image, (640, 480), interpolation=cv2.INTER_CUBIC)

        cv2.imshow("Camera Streaming", image)
        drone.rcc(0, 0.05, 0, 0, 100)

        if cv2.waitKey(5) == ord('q'):
            cv2.destroyAllWindows()
            break

    # # Test Roll (+ve go left, -ve go right)
    # drone.rcc(1, 0, 0, 0)
    # time.sleep(1)
    # drone.rcc(-1, 0, 0, 0)
    # time.sleep(1)
    #
    # # Test Pitch (+ve go fwd, -ve go bwd)
    # drone.rcc(0, 1, 0, 0)
    # time.sleep(3)
    # drone.rcc(0, -1, 0, 0)
    # time.sleep(3)
    #
    # # Test Throttle (+ve go up, -ve go down)
    # drone.rcc(0, 0, 2, 0)
    # time.sleep(3)
    # drone.rcc(0, 0, -1, 0)
    # time.sleep(3)
    #
    # # Test Yaw (+ve go CCW, -ve go CW)
    # drone.rcc(0, 0, 0, 1)
    # time.sleep(3)
    # drone.rcc(0, 0, 0, -1)
    # time.sleep(3)

    drone.land()
    print("[INFO] Drone landed")
    time.sleep(1)

    drone.gimbalUp()
    time.sleep(10)

    drone.disarm()
    print("[INFO] Drone is disarmed")
    time.sleep(1)

    drone.cameraStream_OFF()

    drone.disconnect()
    print("[INFO] Drone is disconnected")
    time.sleep(1)


if __name__ == "__main__":
    main()
