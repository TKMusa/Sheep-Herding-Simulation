import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from std_msgs.msg import String
from sheep_simulation_interfaces.msg import EntityPoseArray

import os
import math

class TestCollisionNode(Node):
    def __init__(self):
        super().__init__('test_collision_node')

        # Track timestamps
        self.start_time = self.get_clock().now()
        self.get_logger().info(f"Simulation started at: {self.start_time.to_msg()}")

        # Subscribe to sheep positions
        self.sheep_position_subscription = self.create_subscription(
            EntityPoseArray,
            'sheep_simulation/sheep/pose',
            self.sheep_position_callback,
            10
        )

        # Publisher for results
        self.results_publisher = self.create_publisher(String, '/simulation/collision_results', 10)

        # Define pen boundaries
        self.grid = [
            [-25.0, 25.0],  # xmin, xmax
            [-25.0, 25.0],  # ymin, ymax
        ]
        self.pen_size = 10.0

        self.pen_x_min = self.grid[0][1] - self.pen_size
        self.pen_y_min = self.grid[1][1] - self.pen_size
        self.pen_x_max = self.grid[0][1]
        self.pen_y_max = self.grid[1][1]

        # Track sheep positions and collisions
        self.sheep_positions = {}
        self.collision_count = 0
        self.collisions_over_time = [0] * 10  # 5-second intervals up to 50 seconds
        self.all_sheep_safe_logged = False

        # Directory to save results
        self.results_dir = os.path.expanduser("~/ros2_ws/simulation_results")
        os.makedirs(self.results_dir, exist_ok=True)

    def sheep_position_callback(self, msg):
        """
        Track sheep positions, check for collisions, and check if all sheep are safe.
        """
        if self.all_sheep_safe_logged:
            return  # Stop processing once all sheep are safe

        # Update sheep positions
        new_positions = {sheep.name: (sheep.x, sheep.y) for sheep in msg.entity_positions}

        # Check for collisions
        self.detect_collisions(new_positions)

        # Update positions
        self.sheep_positions = new_positions

        # Check if all sheep are safe
        if self.sheep_safe() and not self.all_sheep_safe_logged:
            self.on_all_sheep_safe()

    def detect_collisions(self, positions):
        """
        Detect collisions between sheep based on rounded positions.
        """
        rounded_positions = {}
        for name, (x, y) in positions.items():
            rounded_pos = (round(x, 1), round(y, 1))
            if rounded_pos in rounded_positions:
                self.collision_count += 1

                # Calculate the current time interval
                elapsed_time = (self.get_clock().now().nanoseconds - self.start_time.nanoseconds) / 1e9
                time_index = int(elapsed_time // 5)
                if 0 <= time_index < len(self.collisions_over_time):
                    self.collisions_over_time[time_index] += 1

                self.get_logger().info(f"Collision detected between {rounded_positions[rounded_pos]} and {name} at {rounded_pos}")
            else:
                rounded_positions[rounded_pos] = name

    def sheep_safe(self):
        """
        Check if all sheep are within the pen boundaries.
        """
        return all(
            (self.pen_x_min <= pos[0] <= self.pen_x_max and self.pen_y_min <= pos[1] <= self.pen_y_max)
            for pos in self.sheep_positions.values()
        )

    def on_all_sheep_safe(self):
        """
        Publish collision results, log elapsed time, and save to file when all sheep are safe.
        """
        end_time = self.get_clock().now()
        elapsed_time = (end_time.nanoseconds - self.start_time.nanoseconds) / 1e9
        result_msg = (
            f"All sheep safe in pen. Elapsed time: {elapsed_time:.2f} seconds. "
            f"Total collisions detected: {self.collision_count}."
        )

        # Publish the results
        self.results_publisher.publish(String(data=result_msg))
        self.get_logger().info(result_msg)

        # Save detailed results to a file
        try:
            results_file_path = os.path.join(self.results_dir, "collision_results.txt")
            with open(results_file_path, "w") as file:
                file.write(result_msg + "\n")
                file.write("Collision breakdown by 5-second intervals:\n")
                for i, count in enumerate(self.collisions_over_time):
                    file.write(f"{i * 5}-{(i + 1) * 5}: {count}\n")
            self.get_logger().info(f"Results saved to '{results_file_path}'.")
        except Exception as e:
            self.get_logger().error(f"Failed to write results to file: {e}")

        # Mark as logged
        self.all_sheep_safe_logged = True

def main(args=None):
    rclpy.init(args=args)
    node = TestCollisionNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()


