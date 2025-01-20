# Import packages
# import ....

class AiX500:
    """
    This class provides API to control the Holybro drone
    """

    # Takeoff Altitude (cm)
    takeoff_Alt = 150
    max_Alt = 500
    min_Alt = 100

    def __init__(self):
        """
        Initializes the object

        @param self:
        """

        raise NotImplementedError("Function Not Implemented!")

    def connect(self):
        """
        Connects to the drone

        @param self:

        @return: True on success, False otherwise
        """

        raise NotImplementedError("Function Not Implemented!")

    def disconnect(self):
        """
        Disconnects from the drone

        @param self:

        @return: True on success, False otherwise
        """

        raise NotImplementedError("Function Not Implemented!")

    def arm(self):
        """
        Arm the drone

        Condition: check if drone is connected before arming.

        @param self:

        @return: True on success, False otherwise
        """

        raise NotImplementedError("Function Not Implemented!")

    def disarm(self):
        """
        Disarm the drone

        Condition: check if drone has landed before disarming.

        @param self:

        @return: True on success, False otherwise
        """

        raise NotImplementedError("Function Not Implemented!")

    def takeoff(self):
        """
        Take off the drone to an altitude = takeoff_Alt

        Condition: check if drone is armed before taking off.

        @param self:

        @return: True on success, False otherwise
        """

        raise NotImplementedError("Function Not Implemented!")

    def land(self):
        """
        Land the drone

        Condition:

        @param self:

        @return: True on success, False otherwise
        """

        raise NotImplementedError("Function Not Implemented!")

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

        raise NotImplementedError("Function Not Implemented!")

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

        raise NotImplementedError("Function Not Implemented!")

    def getAltitude(self):
        """
        Returns the drone altitude

        Condition:

        @param self

        @return: The drone altitude
        """

        raise NotImplementedError("Function Not Implemented!")

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
