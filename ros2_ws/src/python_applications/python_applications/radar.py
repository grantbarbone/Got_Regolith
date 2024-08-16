import struct
import ctypes
import signal
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool
from std_msgs.msg import Header
from radariq import RadarIQ, MODE_POINT_CLOUD

class RadarIQPointCloudNode(Node):
    def __init__(self):
        super().__init__('RadarIQPointCloudNode')

        # Declare parameters
        serial_port = self.declare_parameter('serial_port', "/dev/ttyACM0").value
        framerate = self.declare_parameter('framerate', 10).value
        distancefilter_min = self.declare_parameter('distancefilter_min', 0.0).value
        distancefilter_max = self.declare_parameter('distancefilter_max', 10.0).value
        anglefilter_min = self.declare_parameter('anglefilter_min', -45).value
        anglefilter_max = self.declare_parameter('anglefilter_max', 45).value
        pointdensity = self.declare_parameter('pointdensity', 0).value
        topic = self.declare_parameter('topic', "radariq").value
        frame_id = self.declare_parameter('frame_id', "radar").value

        # Publisher
        self.pub = self.create_publisher(PointCloud2, topic, 10)

        # Subscriber
        self.subscription = self.create_subscription(
            Bool,
            "topic_Main_to_Radar",
            self.listener_callback,
            10)

        # RadarIQ Setup
        self.riq = RadarIQ(port=serial_port)
        self.riq.set_mode(MODE_POINT_CLOUD)
        self.riq.set_units('m', 'm/s')
        self.riq.set_frame_rate(framerate)
        self.riq.set_distance_filter(distancefilter_min, distancefilter_max)
        self.riq.set_angle_filter(anglefilter_min, anglefilter_max)
        self.riq.set_point_density(pointdensity)
        self.riq.start()

        # PointCloud2 Fields
        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=16, datatype=PointField.FLOAT32, count=1)
        ]

        self.header = Header()
        self.header.frame_id = frame_id

    def listener_callback(self, msg):
        if msg.data:
            self.start_publishing()
        else:
            self.stop_publishing()

    def start_publishing(self):
        for row in self.riq.get_data():
            if row:
                pc2 = self.create_cloud(self.header, self.fields, row)
                pc2.header.stamp = self.get_clock().now().to_msg()
                self.pub.publish(pc2)
                self.get_logger().info(f'Published PointCloud2 message with {len(row)} points.')

    def stop_publishing(self):
        self.get_logger().info('Stopped publishing.')

    def create_cloud(self, header, fields, points):
        cloud_struct = struct.Struct("<ffff")
        buff = ctypes.create_string_buffer(cloud_struct.size * len(points))
        point_step, pack_into = cloud_struct.size, cloud_struct.pack_into
        offset = 0
        for p in points:
            pack_into(buff, offset, *p[:-1])
            offset += point_step

        return PointCloud2(header=header,
                           height=1,
                           width=len(points),
                           is_dense=False,
                           is_bigendian=False,
                           fields=fields,
                           point_step=cloud_struct.size,
                           row_step=cloud_struct.size * len(points),
                           data=buff.raw)

def main(args=None):
    signal.signal(signal.SIGINT, signal_handler)
    rclpy.init(args=args)
    pointcloud_publisher = RadarIQPointCloudNode()
    rclpy.spin(pointcloud_publisher)
    rclpy.shutdown()

def signal_handler(sig, frame):
    try:
        riq.close()
    except:
        pass

if __name__ == '__main__':
    main()