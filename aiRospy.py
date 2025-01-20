# Import packages
import time
import cv2

import rospy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State, Altitude
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class AiRospy:
    """
    This class provides API to control a drone in the RoS/Gazebo environment
    """

    # Takeoff Altitude (cm)
    takeoff_Alt = 150
    max_Alt = 500
    min_Alt = 100

    droneState = None
    dronePosition = None
    droneVelocity = None
    camImage = None
    camBridge = None
    rate = None  # Rate at which state is polled

    def state_callback(self, state_msg):
        self.droneState = state_msg

    def pose_callback(self, pose_msg):
        self.dronePosition = pose_msg

    def vel_callback(self, vel_msg):
        self.droneVelocity = vel_msg

    def img_callback(self, img_msg):
        try:
            img = self.camBridge.imgmsg_to_cv2(img_msg, "rgb8")
            self.camImage = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        except CvBridgeError as error:
            print(error)

    def set_mode(self, mode):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            service(0, mode)  # Set to 0 for custom mode
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def __init__(self):
        """
        Initializes the object

        @param self:
        """

        # Global variable to store the drone's state
        self.image_sub = None
        self.vel_sub = None
        self.local_setpoint_pub = None
        self.setpoint_velocity_pub = None
        self.pose_sub = None
        self.state_sub = None
        self.droneState = State()
        self.dronePosition = PoseStamped()
        self.droneVelocity = TwistStamped()
        self.camBridge = CvBridge()

    def connect(self):
        """
        Connects to the drone

        @param self:

        @return: True on success, False otherwise
        """

        rospy.init_node('drone_control')

        self.rate = rospy.Rate(10.0)  # 10 Hz

        # Create a publisher for the local position setpoint
        self.local_setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local',
                                                  PoseStamped, queue_size=10)
        self.setpoint_velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',
                                                     TwistStamped, queue_size=10)

        # Define a subscriber for the `/mavros/state` topic
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_callback)
        self.pose_sub = rospy.Subscriber("/mavros/local_position/pose",
                                         PoseStamped, self.pose_callback, queue_size=10)
        self.vel_sub = rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped,
                                        self.vel_callback, queue_size=10)

        # Wait for FCU connection
        while not self.droneState.connected and not rospy.is_shutdown():
            self.rate.sleep()

        mode = "GUIDED"
        self.set_mode(mode)

        while not (self.droneState.mode == mode):
            self.rate.sleep()

        return True

    def disconnect(self):
        """
        Disconnects from the drone

        @param self:

        @return: True on success, False otherwise
        """

        self.set_mode("RTL")

        while not (self.droneState.mode == "RTL"):
            self.rate.sleep()

        return True

    def arm(self):
        """
        Arm the drone

        Condition: check if drone is connected before arming.

        @param self:

        @return: True on success, False otherwise
        """

        if not self.isConnected():
            print("ERROR: Drone is not connected")
            return False

        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            service(True)
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False

        while not self.isArmed():
            self.rate.sleep()
        return True

    def disarm(self):
        """
        Disarm the drone

        Condition: check if drone has landed before disarming.

        @param self:

        @return: True on success, False otherwise
        """

        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            service(False)
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False

        while self.isArmed():
            self.rate.sleep()
        return True

    def takeoff(self):
        """
        Take off the drone to an altitude = takeoff_Alt

        Condition: check if drone is armed before taking off.

        @param self:

        @return: True on success, False otherwise
        """

        if not self.isArmed():
            print("ERROR: The drone is NOT armed!")
            return False

        # Send the takeoff command with the specified height
        rospy.wait_for_service('/mavros/cmd/takeoff')
        try:
            service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
            service(altitude=(self.takeoff_Alt // 100))
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

        while self.dronePosition.pose.position.z < (self.takeoff_Alt // 100) - 0.5:
            self.rate.sleep()
        return True

    def land(self):
        """
        Land the drone

        Condition:

        @param self:

        @return: True on success, False otherwise
        """

        if not self.isArmed():
            print("ERROR: The drone is NOT armed!")
            return False

        # Send the takeoff command with the specified height
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
            service(altitude=0)
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

        while self.dronePosition.pose.position.z > 0.1:
            self.rate.sleep()

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

        velocity = TwistStamped()
        velocity.twist.linear = Vector3(x=0, y=0, z=0)
        velocity.twist.angular = Vector3(x=0, y=0, z=0)

        velocity.twist.linear = Vector3(x=FB, y=LR, z=UD)
        velocity.twist.angular.z = Yaw

        self.setpoint_velocity_pub.publish(velocity)

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

        return self.droneState.connected

    def isArmed(self):
        """
        Returns the arm status of the drone

        Condition:

        @param self

        @return: True if armed, False otherwise
        """

        return self.droneState.armed

    def cameraStream_ON(self):
        """
        Starting camera stream

        Condition:

        @param self

        @return: Status
        """

        self.image_sub = rospy.Subscriber("/webcam/image_raw", Image, self.img_callback)

    def cameraStream_OFF(self):
        """
        Stopping camera stream

        Condition:

        @param self

        @return: Status
        """

        self.image_sub = None

    def getFrame(self):
        """
        Capturing a frame

        Condition:

        @param self

        @return: Status and Frame
        """

        return self.camImage


def main():
    drone = AiRospy()
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
    print("{}".format(drone.dronePosition))
    print("{}".format(drone.droneVelocity))

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
