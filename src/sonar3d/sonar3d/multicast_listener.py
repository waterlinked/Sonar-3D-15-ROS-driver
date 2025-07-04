import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from sonar3d.api.inspect_sonar_data import parse_rip1_packet, decode_protobuf_packet, rangeImageToXYZ
from sonar3d.api.interface_sonar_api import set_acoustics, describe_response
import socket
import struct

# Multicast group and port used by the Sonar 3D-15
MULTICAST_GROUP = '224.0.0.96'
PORT = 4747

# Listen to all IPs by default, or set to a specific IP.
SONAR_IP = ""

# The maximum possible packet size for Sonar 3D-15 data
BUFFER_SIZE = 65535


class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')
        
        # Declare parameters
        self.declare_parameter('IP', "192.168.1.233")#'192.168.194.96') # Change to the sonar ip
        self.declare_parameter('speed', 1491)    # setting this takes ~20s

        self.sonar_ip = self.get_parameter('IP').get_parameter_value().string_value
        self.sonar_speed = self.get_parameter('speed').get_parameter_value().integer_value
        SONAR_IP = self.sonar_ip

        # Create a timer that calls the timer_callback every sample_time seconds 
        sample_time = 0.01          # sample time in seconds
        self.create_timer(sample_time, self.timer_callback)
        self.get_logger().info(f'Timer Node initialized with {1/sample_time} Hz')

        # Create a publisher that publishes the point cloud data
        self.publisher_ = self.create_publisher(PointCloud2, 'sonar_point_cloud', 10)

        # Enable the acoustics on the sonar
        resp = set_acoustics(self.sonar_ip, True)
        self.get_logger().info(f'Enabling acoustics response: {describe_response(self.sonar_ip, resp)}')

        # Set up a UDP socket with multicast membership
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', PORT))

        group = socket.inet_aton(MULTICAST_GROUP)
        mreq = struct.pack('4sL', group, socket.INADDR_ANY)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

        self.get_logger().info(f"Listening for Sonar 3D-15 RIP1 packets on {MULTICAST_GROUP}:{PORT}...")
        if SONAR_IP != "":
            self.get_logger().info(f"Filtering packets from IP: {SONAR_IP}")


    def timer_callback(self):
        data, addr = self.sock.recvfrom(BUFFER_SIZE)

        # If SONAR_IP is configured, and this doesn't match the known Sonar IP, skip it.
        if SONAR_IP != "" and addr[0] != SONAR_IP:
            return

        payload = parse_rip1_packet(data)
        if payload is None:
            return
        # Decode the Protobuf message
        result = decode_protobuf_packet(payload)
        if not result:
            return

        msg_type, msg_obj = result

        if msg_type == 'RangeImage':
            # Convert the RangeImage message to voxel data
            voxels = rangeImageToXYZ(msg_obj)

            # extract the x, y, z coordinates from the voxels
            pts = []
            for i in range(len(voxels)):
                pts.append((voxels[i]['x'], voxels[i]['y'], voxels[i]['z']))

            # Create a PointCloud2 message
            # Create the msg header
            header = Header()
            header.stamp = self.get_clock().now().to_msg()  # Use ROS2 time
            # header.stamp = msg_obj.header.timestamp.ToNanoseconds()  # Use the timestamp from the message
            header.frame_id = 'sonar_frame'

            msg = point_cloud2.create_cloud_xyz32(header, pts)

            # Publish the PointCloud2 message
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published PointCloud2 message with {len(voxels)} points')

        
        
    

def main(args=None):
    rclpy.init(args=args)
    node = TimerNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()