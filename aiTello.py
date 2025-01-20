# Import packages
from djitellopy import tello
import time
import numpy as np
import cv2


class AiTello:
    """
    This class provides API to control the DJI Tello Drone
    """

    # Takeoff Altitude (cm)
    takeoff_Alt = 150
    max_Alt = 500
    min_Alt = 100

    droneState = None
    drone = None
    frameReader = None

    def __init__(self):
        """
        Initializes the object

        @param self:
        """

        self.drone = tello.Tello()

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

        # self.drone.end()
        return True

    def arm(self):
        """
        Arm the drone

        Condition: check if drone is connected before arming.

        @param self:

        @return: True on success, False otherwise
        """

        # self.drone.turn_motor_on()
        return True

    def disarm(self):
        """
        Disarm the drone

        Condition: check if drone has landed before disarming.

        @param self:

        @return: True on success, False otherwise
        """

        # self.drone.turn_motor_off()
        return True

    def takeoff(self):
        """
        Take off the drone to an altitude = takeoff_Alt

        Condition: check if drone is armed before taking off.

        @param self:

        @return: True on success, False otherwise
        """

        self.drone.takeoff()
        return True

    def land(self):
        """
        Land the drone

        Condition:

        @param self:

        @return: True on success, False otherwise
        """

        self.drone.land()
        return True

    def rcc(self, LR, FB, UD, Yaw):
        """
        Update the motor status according to the remote control command to provided value

        Condition: Keep the altitude between min_Alt and max_Alt

        @param self
        @param LR: Left (+) and Right (-)
        @param FB: Forward (+) and Backward (-)
        @param UD: Upward (+) and Downward (-)
        @param Yaw: Left (+) and Right (-)

        @return: True on success, False otherwise
        """

        # Limit the input values from -1 to 1
        valList = [LR, FB, UD, Yaw]
        LR, FB, UD, Yaw = np.clip(valList, -1, 1)
        # print(f'[DEBUG] Limited Range = {LR}, {FB}, {UD}, {Yaw}')

        # Scale the values to match 50% of Tello's maximum speed
        LR, FB, UD, Yaw = np.multiply([LR, FB, UD, Yaw], 50)
        # print(f'[DEBUG] Scaled Range = {LR}, {FB}, {UD}, {Yaw}')

        # Convert to integer
        LR, FB, UD, Yaw = int(LR), int(FB), int(UD), int(Yaw)
        # print(f'[DEBUG] Integer Range = {LR}, {FB}, {UD}, {Yaw}')

        self.drone.send_rc_control(-LR, FB, UD, -Yaw)
        return True

    def moveUp(self, distance):
        """
        Increase altitude by requested distance

        Condition: limit the altitude to max_Alt

        @param self
        @param distance: Go up by a distance = distance cm

        @return: True on success, False otherwise
        """

        self.drone.move_up(distance)

    def moveDown(self, distance):
        """
        Decrease altitude by requested distance

        Condition: limit the altitude to min_Alt

        @param self
        @param distance: Go up by a distance = distance cm

        @return: True on success, False otherwise
        """

        self.drone.move_down(distance)

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

        return self.drone.get_battery()

    def getAltitude(self):
        """
        Returns the drone altitude

        Condition:

        @param self

        @return: The drone altitude
        """

        return self.drone.get_height()

    def getGPSPosition(self):
        """
        Returns the droneGPS position

        Condition:

        @param self

        @return: The drone GPS position as a Tuple (Latitude, Longitude, Altitude)
        """

        raise NotImplementedError("Function Not Implemented!")

    def getHeading(self):
        """
        Calculates and returns the compass degree based on the drone's yaw angle

        @param self

        @return: The compass degree as a float
        """

        raise NotImplementedError("Function Not Implemented!")

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

        raise NotImplementedError("Function Not Implemented!")

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

        self.drone.streamoff()
        self.drone.streamon()
        self.frameReader = self.drone.get_frame_read()

    def cameraStream_OFF(self):
        """
        Stopping camera stream

        Condition:

        @param self

        @return: Status
        """

        self.drone.streamoff()
        self.frameReader = None

    def getFrame(self):
        """
        Capturing a frame

        Condition:

        @param self

        @return: Status and Frame
        """

        image = self.frameReader.frame
        image = cv2.resize(image, (640, 480), interpolation=cv2.INTER_CUBIC)
        image = cv2.flip(image, 0)

        return image

    def gimbalDown(self):
        """
        Tilts the gimbal downwards (Down facing camera)

        Condition:

        @param self

        @return: Status
        """
        raise NotImplementedError("Function Not Implemented!")

    def gimbalUp(self):
        """
        Tilts the gimbal upwards (front facing camera)

        Condition:

        @param self

        @return: Status
        """
        raise NotImplementedError("Function Not Implemented!")


def main():
    drone = AiTello()
    print("Drone object is created!")

    drone.connect()
    print("Drone is connected")
    # time.sleep(2)

    if drone.arm():
        print("Drone is Armed")
    else:
        print("Drone arming failed. Exiting")
        drone.disconnect()
        exit()
    time.sleep(5)

    drone.takeoff()
    print("Drone took off")

    time.sleep(3)

    drone.rcc(0, 1, 0, 0.5)
    time.sleep(10)
    drone.rcc(0, -1, 0, -0.5)
    time.sleep(10)

    drone.land()
    print("Drone landed")
    time.sleep(1)

    drone.disarm()
    print("Drone is disarmed")
    time.sleep(1)

    drone.disconnect()
    print("Drone is disconnected")
    time.sleep(1)


if __name__ == "__main__":
    main()
