import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from sonar3d.api.inspect_sonar_data import parse_rip1_packet, decode_protobuf_packet, rangeImageToXYZ


class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')

        # Declare parameters
        self.declare_parameter('file', '/home/peder/git/Sonar-3D-15-ROS-driver/sonar-recording-2025-05-15-093611.sonar')
        
        # Get parameter
        self.sonar_file = self.get_parameter('file').get_parameter_value().string_value
        
        # Create a timer that calls the timer_callback with a sample time of 6 Hz
        sample_time = 1/6          # sample time in seconds 
        self.create_timer(sample_time, self.timer_callback)
        self.get_logger().info(f'Timer Node initialized with {1/sample_time} Hz')

        # Create a publisher that publishes the point cloud data
        self.publisher_ = self.create_publisher(PointCloud2, 'sonar_point_cloud', 10)


        # Open the saved .sonar file 
        filename = '/home/peder/git/Sonar-3D-15-ROS-driver/sonar-recording-2025-05-15-093611.sonar'
        with open(filename, 'rb') as f:
            content = f.read()
        packets = content.split(b'RIP1')
        self.checked_packets = self.check_valid_packet(packets)


    def check_valid_packet(self, packets):
        """
        Check if the packets are valid by parsing them and decoding the Protobuf message.
        """
        # loop through the packets and check if they are valid
        valid_packets = []
        for pkt in packets:
            # Check if the packet is valid
            payload = parse_rip1_packet(b'RIP1' + pkt)
            if payload is None:
                continue  # skip invalid packets

            # Decode the Protobuf message
            result = decode_protobuf_packet(payload)
            if not result:
                continue  # skip invalid packets
            valid_packets.append(pkt)
        return valid_packets


    def timer_callback(self):

        # Check if there are packets left to process
        if len(self.checked_packets) > 0:
            # Get the next packet and process it
            pkt = self.checked_packets.pop(0)

            # Check if the packet is valid
            payload = parse_rip1_packet(b'RIP1' + pkt)

            # Decode the Protobuf message
            result = decode_protobuf_packet(payload)

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


            # Add the packet back to the end of the list for continuous processing
            self.checked_packets.append(pkt)

        else:
            self.get_logger().info('No packets left to process')
        
    

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