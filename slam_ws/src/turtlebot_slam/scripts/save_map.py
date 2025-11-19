#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SaveMap

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        self.cli = self.create_client(SaveMap, 'slam_toolbox/save_map')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for save_map service...')
        req = SaveMap.Request()
        req.name = 'team_map'
        self.future = self.cli.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = MapSaver()
    rclpy.spin_until_future_complete(node, node.future)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
