from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
import transforms3d


class CoordinateNavigation(Node):

    def __init__(self):
        super().__init__('coordinate_navigation')
        self.scan_ranges = []
        self.has_scan_received = False

        # distance robot travels before attempting to re-orient towards goal
        self.travel_distance = 0.25

        #Acceptable obstacle distance = 0.3m
        self.stop_distance = 0.3

        qos = QoSProfile(depth=10)

        #destination coordinates
        self.dest_x = 1.5
        self.dest_y = 1.5

        #current robot coordinates
        self.current_x = 0
        self.current_y = 0

        #robot angle in radians
        self.yaw = 0.0

        #variables used to track distance covered
        self.prev_x = None
        self.prev_y = None
        self.distance_travelled = 0.0
        self.reversing = False

        #tracks if robot has been oriented initially
        self.initially_oriented = False


        #scan parameters
        self.scan_angle_min = 0.0
        self.scan_angle_increment = 0.0

        #initializing publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos_profile=qos_profile_sensor_data
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)


        self.timer = self.create_timer(0.1, self.timer_callback)

    def odom_callback(self, msg):
        #updating current coordinates using odom readings
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Extracting yaw from quaternion (quaternion is a metric to measure orientation)
        orientation_q = msg.pose.pose.orientation
        _, _, self.yaw = self.euler_from_quaternion(orientation_q)


        # Setting previous values for tracking distance travelled on first call
        if self.prev_x is None:
            self.prev_x = self.current_x
            self.prev_y = self.current_y
            return
        

        # Calculating distance moved
        dx = self.current_x - self.prev_x
        dy = self.current_y - self.prev_y

        # Adding distance travelled only if not reversing
        if not self.reversing:
            self.distance_travelled += math.hypot(dx, dy)
            #if distance travelled >= 25cm, call face_destination to orient robot towards destination
            if self.distance_travelled >= self.travel_distance:
                self.face_destination()
                self.distance_travelled = 0.0

        self.prev_x = self.current_x
        self.prev_y = self.current_y

    #quaternion is a metric to measure orientation with 4 fields, we use euler form (roll, pitch, yaw)
    #so this function converts it
    def euler_from_quaternion(self, q):
        # Convert quaternion to euler (roll, pitch, yaw)
        orientation_list = [q.w, q.x, q.y, q.z]
        (roll, pitch, yaw) = transforms3d.euler.euler2quat(orientation_list)
        return roll, pitch, yaw



    def timer_callback(self):
        #orients robot in direction of destination on initialization of program
        if not self.initially_oriented:
            self.face_destination()
            self.initially_oriented = True
            return


        #checking if robot is close enough to destination, stopping if it is
        if math.hypot(self.dest_x - self.current_x,
            self.dest_y - self.current_y) < 0.1:
            stop = Twist()
            self.cmd_vel_pub.publish(stop)
            return
        
        #if robot is not reversing (and since this method is running, not checking for obstacles), move forward
        if not self.reversing:
            twist = Twist()
            twist.linear.x = 0.2
            self.cmd_vel_pub.publish(twist)
        



    def scan_callback(self, msg: LaserScan):

        #cancelling scan operations if robot is reversing
        if self.reversing:
            return

        #saving scan parameters
        self.scan_angle_min        = msg.angle_min
        self.scan_angle_increment  = msg.angle_increment
        self.scan_ranges           = msg.ranges

        #ensures sufficient readings are present to perform obstacle checks
        if len(self.scan_ranges) <= 4:
            return
        # finding smallest distance of obstacle in left and right scanning ranges
        left  = min(self.scan_ranges[0: len(self.scan_ranges)//4])
        right = min(self.scan_ranges[3*len(self.scan_ranges)//4:])
        if min(left, right) < self.stop_distance:
            # find which side was closer and save index of reading and distance
            if left < right:
                index = self.scan_ranges.index(left)
                dist = left
            else:
                index = self.scan_ranges.index(right, 3*len(self.scan_ranges)//4)
                dist = right

            # calculate the beam angle of that nearest obstacle
            beam_angle = self.scan_angle_min + index * self.scan_angle_increment

            # reversing robot
            self.reverse()
            # rotating robot right to be parallel with obstacle
            self.rotate_parallel(beam_angle)


    #beam_angle: angle in radians of the obstacle ray relative to robot front.
    #We turn right so that the robot faces parallel to the wall:
    def rotate_parallel(self, beam_angle: float):

        # calculate desired yaw
        desired = self.yaw + beam_angle - math.pi/2
        # normalize into [-π, π]
        desired = math.atan2(math.sin(desired), math.cos(desired))

        # error between current yaw and desired yaw
        err = math.atan2(math.sin(desired - self.yaw),
                         math.cos(desired - self.yaw))
        
        twist = Twist()
        # rotate until robot is within 0.05 rad of desired yaw
        while abs(err) > 0.05:
            twist.angular.z = 0.5 * err
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self)
            err = math.atan2(math.sin(desired - self.yaw),
                             math.cos(desired - self.yaw))

        # stop turning
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)




    def reverse(self):
        #setting reversing flag, so distance travelled not counted
        self.reversing = True

        # starting position
        start_x = self.current_x
        start_y = self.current_y

        twist = Twist()
        #twist to move backwards
        twist.linear.x = -0.1
        self.cmd_vel_pub.publish(twist)

        # reverse until until 20cm travelled
        while math.hypot(self.current_x - start_x,
                         self.current_y - start_y) < 0.2:
            rclpy.spin_once(self)

        # stopping movement
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)

        # resetting previous position so reverse isn't counted
        self.prev_x = self.current_x
        self.prev_y = self.current_y
        self.distance_travelled = 0
        self.reversing = False

    def face_destination(self):
        #differences in x and y between current and goal coordinates
        dx = self.dest_x - self.current_x
        dy = self.dest_y - self.current_y
        target_angle = math.atan2(dy, dx)

        # angle difference
        angle_diff = target_angle - self.yaw
        # normalize angle within range [-pi, pi]
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        twist = Twist()
        #looping while difference between 
        while abs(angle_diff) > 0.05:
            twist.angular.z = 0.5 * angle_diff  # Proportional control
            #publishing rotation command
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self)
            angle_diff = target_angle - self.yaw
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        # stopping rotation
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
            

            
def main(args=None):
    rclpy.init(args=args)
    node = CoordinateNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()






