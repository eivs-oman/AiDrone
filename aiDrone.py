# Import packages
from .aiTello import AiTello
from .aiRospy import AiRospy
from .aiAnafi import AiANAFI
from .aiX500 import AiX500

import time

# import cv2

PLATFORMS = ["RoS", "X500", "Tello", "ANAFI"]


class AiDrone:
    """
    This class provides API to control a drone from the list PLATFORMS
    """

    # The target platform Used to determine which class to instantiate
    platform = None
    drone = None

    droneStatus = {"Connected": False,
                   "armed": False,
                   "altitude": 0.0,
                   "battery": 0.0,
                   "ToF": 0.0
                   }

    def __init__(self, platform=PLATFORMS[0], droneIP=None):
        """
        Initializes the object. The function sets the target platform

        @param self:
        @param platform: The selected Drone platform
        @param droneIP: The IP of the drone (Applicable to some of the platforms)
        """

        self.platform = platform

        if self.platform == PLATFORMS[0]:
            self.drone = AiRospy()
        elif self.platform == PLATFORMS[1]:
            self.drone = AiX500()
        elif self.platform == PLATFORMS[2]:
            self.drone = AiTello()
        elif self.platform == PLATFORMS[3]:
            self.drone = AiANAFI(droneIP)
        else:
            raise NotImplementedError("Class Not Implemented!")

    def connect(self):
        """
        Connects to the drone

        @param self:

        @return: True on success, False otherwise
        """

        return self.drone.connect()

    def disconnect(self):
        """
        Disconnects from the drone

        @param self:

        @return: True on success, False otherwise
        """

        return self.drone.disconnect()

    def arm(self):
        """
        Arm the drone

        Condition: check if drone is connected before arming.

        @param self:

        @return: True on success, False otherwise
        """

        return self.drone.arm()

    def disarm(self):
        """
        Disarm the drone

        Condition: check if drone has landed before disarming.

        @param self:

        @return: True on success, False otherwise
        """

        return self.drone.disarm()

    def takeoff(self):
        """
        Take off the drone to an altitude = takeoff_Alt

        Condition: check if drone is armed before taking off.

        @param self:

        @return: True on success, False otherwise
        """

        return self.drone.takeoff()

    def land(self):
        """
        Land the drone

        Condition:

        @param self:

        @return: True on success, False otherwise
        """

        return self.drone.land()

    def rcc(self, LR, FB, UD, Yaw, speedCap=[100, 100, 100, 100]):
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

        return self.drone.rcc(LR, FB, UD, Yaw, speedCap)

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

        return self.drone.getBattery()

    def getAltitude(self):
        """
        Returns the drone altitude

        Condition:

        @param self

        @return: The drone altitude
        """

        return self.drone.getAltitude()

    def getGPSPosition(self):
        """
        Returns the droneGPS position

        Condition:

        @param self

        @return: The drone GPS position as a Tuple (Latitude, Longitude, Altitude)
        """

        return self.drone.getGPSPosition()

    def getHeading(self):
        """
        Calculates and returns the heading (in degrees) based on the drone's yaw angle

        @param self

        @return: The heading in degrees as a float
        """

        return self.drone.getHeading()

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

        return self.drone.isConnected()

    def isArmed(self):
        """
        Returns the arm status of the drone

        Condition:

        @param self

        @return: True if armed, False otherwise
        """

        return self.drone.isArmed()

    def cameraStream_ON(self):
        """
        Starting camera stream

        Condition:

        @param self

        @return: Status
        """

        return self.drone.cameraStream_ON()

    def cameraStream_OFF(self):
        """
        Stopping camera stream

        Condition:

        @param self

        @return: Status
        """

        return self.drone.cameraStream_OFF()

    def getFrame(self):
        """
        Capturing a frame

        Condition:

        @param self

        @return: Status and Frame
        """

        return self.drone.getFrame()

    def gimbalDown(self):
        """
        Tilts the gimbal downwards (Down facing camera)

        Condition:

        @param self

        @return: Status
        """

        return self.drone.gimbalDown()

    def gimbalUp(self):
        """
        Tilts the gimbal upwards (front facing camera)

        Condition:

        @param self

        @return: Status
        """

        return self.drone.gimbalUp()


def main():
    drone = AiDrone()
    print("Drone object is created!")

    drone.connect()
    print("Drone is connected")
    time.sleep(2)

    if drone.arm():
        print("Drone is Armed")
    else:
        print("Drone arming failed. Exiting")
        drone.disconnect()
        exit()
    time.sleep(5)

    drone.cameraStream_ON()

    drone.takeoff()
    print("Drone took off")

    time.sleep(3)

    drone.rcc(0, 1, 0, 0.5)

    '''while True:
        image = drone.getFrame()
        #cv2.imshow("Drone Camera", image)

       # if cv2.waitKey(10) == ord('q'):
            break'''

    time.sleep(10)
    drone.rcc(0, -1, 0, -0.5)
    time.sleep(10)

    drone.land()
    print("Drone landed")
    time.sleep(1)

    drone.cameraStream_OFF()

    drone.disarm()
    print("Drone is disarmed")
    time.sleep(1)

    drone.disconnect()
    print("Drone is disconnected")
    time.sleep(1)


if __name__ == "__main__":
    main()
