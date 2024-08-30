import rclpy
import os
import numpy as np
import pandas as pd

from rclpy.node import Node
from rclpy.time import Duration
from lfs_msgs.msg import Map
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

class MapPublisher(Node):
    def __init__(self):
        super().__init__('mapping_node')
        
        # Create publisher and timer
        self.map_publisher_ = self.create_publisher(Map, '/mapping/track_points', 10)
        self.map_viz_ = self.create_publisher(MarkerArray, '/mapping/track_viz', 10)
        self.timer_period : int = 1  # whole seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.num_tracks = 10

        # Cone colors
        blue, yellow, orange = ColorRGBA(), ColorRGBA(), ColorRGBA()
        blue.r, blue.g, blue.b, blue.a = 0., 0., 1., 1.  
        yellow.r, yellow.g, yellow.b, yellow.a = 1., 1., 0., 1.
        orange.r, orange.g, orange.b, orange.a = 1., .5, 0., 1.
        self.color_dict = {
            'b': blue,
            'y': yellow,
            'o': orange
        }
        # Converts color to uint8
        self.color_to_int = {
            'b': 0,
            'y': 1,
            'o': 2
        }

        # Dimensions according to FSG rules
        self.cone_radius = .114
        self.cone_height = .325
        self.xl_cone_radius = .1425
        self.xl_cone_height = .505

        # Find the path to the tracks folder
        current_path = os.path.dirname(__file__)
        path_to_ws = current_path.partition('planning_ws')[0]
        self.track_dir = os.path.join(path_to_ws, 'planning_ws/src/map_publisher/map_publisher/tracks')

    def timer_callback(self):
        # Load a random track
        chosen_track = np.random.randint(0, self.num_tracks)
        data = pd.read_csv(self.track_dir + '/track_{0:03d}.csv'.format(chosen_track))
        num_track_points = data.shape[0]
        cone_x = data['x']
        cone_y = data['y']
        cone_colors = data['color']

        # Publish numerical values
        cone_map = Map()
        cone_map.x = cone_x.to_list()
        cone_map.y = cone_y.to_list()
        cone_map.color = cone_colors.replace(self.color_to_int).to_list()
        self.map_publisher_.publish(cone_map)

        # Publish the chosen track as colored cyliners
        markers = []
        for i in range(num_track_points):
            # Create marker object
            marker = Marker()
            marker.type = Marker.CYLINDER
            marker.color = self.color_dict[cone_colors[i]]
            marker.ns = 'track'
            marker.id = i
            marker.lifetime = Duration(seconds=self.timer_period, nanoseconds=0).to_msg()
            
            # Scale of the marker
            radius = self.xl_cone_radius if cone_colors[i] == 'o' else self.cone_radius
            height = self.xl_cone_height if cone_colors[i] == 'o' else self.cone_height
            marker.scale.x, marker.scale.y, marker.scale.z = radius, radius, height

            # Position and orientation
            marker.pose.position.x = cone_x[i]
            marker.pose.position.y = cone_y[i]
            marker.pose.position.z = height / 2
            marker.pose.orientation.x = 0.
            marker.pose.orientation.y = 0.
            marker.pose.orientation.z = 0.
            marker.pose.orientation.w = 1.

            # Add header and append
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            markers.append(marker)

        msg = MarkerArray()
        msg.markers = markers
        self.map_viz_.publish(msg)
        self.get_logger().info('Published track {0:03d}'.format(chosen_track))


def main(args=None):
    rclpy.init(args=args)

    map_pub = MapPublisher()

    rclpy.spin(map_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    map_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()