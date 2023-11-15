#Author Jai McElroy Novemeber 14,2023,The Univeristy of Adelaide
#This file is a base file for setting markers using Rviz2 ROS 2. By using marker_publisher.add_marker(pose) and passing in pose 
#you can add a marker at postion pose on the map. Note Map frame must be published before hand and the frame ID that this file is 
#refrencing is "map". This file also accepts a decription of the object and will allocate that object a colour and and shape based 
#off that description.
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from pose_utils import create_pose
class MarkerPublisher(Node):

    def __init__(self):
        super().__init__('marker_publisher')
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)

        # Create a Timer to periodically publish markers
        self.timer = self.create_timer(1.0, self.publish_markers)

        # Initialize MarkerArray
        self.markers_array = MarkerArray()

    def publish_markers(self):
        # Publish the array of markers
        self.marker_pub.publish(self.markers_array)
        self.get_logger().info('Markers published')

    def add_marker(self,pose,description):
        # Create a new marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "my_namespace"
        marker.id = len(self.markers_array.markers)
    
        marker.pose=pose
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
      

        if description=="Helicopter (Friendly)":#GREEN

            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            

        elif description=="Helicopter (Neutral)":#YELLOW

            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            
        elif description=="Helicopter (Hostile)":#RED

            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            
        elif description=="Criminal":#DARK RED

            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.color.a = 1.0
            marker.color.r = 0.55
            marker.color.g = 0.0
            marker.color.b = 0.0
           
        elif description=="Animal":#DARK ORANGE

            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.55
            marker.color.b = 0.0
        elif description=="Child":#PEACH

            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.85
            marker.color.b = 0.73   
        elif description=="Victim":#CYAN

            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
        elif description=="Bystander":#DEEP SKY BLUE

            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.75
            marker.color.b = 1.0
        elif description=="Robot (Friendly)":#GREEN

            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            
        elif description=="Robot (Neutral)":#Dark Grey

            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.color.a = 1.0
            marker.color.r = 0.66
            marker.color.g = 0.66
            marker.color.b = 0.66
        elif description=="Robot (Hostile)":#Black

            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        else:
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        

        marker.lifetime = Duration()
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        # Append the new marker to the markers_array
        self.markers_array.markers.append(marker)
        self.get_logger().info(f'Added marker at ({pose.position.x}, {pose.position.y},{pose.position.z})')

def main(args=None):
    rclpy.init(args=args)

    marker_publisher = MarkerPublisher()
    pose1 = create_pose(1.0, 2.0,1.0)
    pose2 = create_pose(1.0, 4.0,0.15)
    pose3 = create_pose(2.0, 2.0,1.0)
    pose4 = create_pose(2.0, 4.0,0.15)
    marker_publisher.add_marker(pose1,"Helicopter (Friendly)")
    marker_publisher.add_marker(pose2,"Criminal")
    marker_publisher.add_marker(pose3,"Helicopter (Neutral)")
    marker_publisher.add_marker(pose4,"Robot (Hostile)")

# Create a Pose object with desired position and orientation
    try:
        rclpy.spin(marker_publisher)
    except KeyboardInterrupt:
        pass

    marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
