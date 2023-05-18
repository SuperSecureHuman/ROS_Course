#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial

from my_robot_interfaces.srv import SetLed


class BatteryNode(Node):

    def __init__(self):
        super().__init__("battery_status")
        self.battery_state = "full"
        self.state_change_time_ = self.get_current_time_seconds()
        self.battery_timer_ = self.create_timer(0.5,self.check_battery_status)

    def get_current_time_seconds(self):
        secs, nsecs = self.get_clock().now().seconds_nanoseconds()
        return secs + nsecs/1000000000.0

    def check_battery_status(self):
        charging_time = 6.0
        discharging_time = 4.0
        current_time_ = self.get_current_time_seconds()
        cycle_time = current_time_- self.state_change_time_
        if  cycle_time > discharging_time and self.battery_state == "full":
            self.battery_state = "empty"
            self.get_logger().info(f"The battery is empty! Charging in {charging_time} seconds...")
            self.state_change_time_ = current_time_
            self.call_battery_status(3,1)
        if cycle_time > charging_time and self.battery_state == "empty":
            self.battery_state = "full"
            self.get_logger().info(f"The battery is full! Discharging in {discharging_time} seconds...")
            self.state_change_time_ = current_time_
            self.call_battery_status(3,0)

        
        

    def call_battery_status(self, n, s):
        client_ = self.create_client(SetLed, "set_led")
        while not client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for Server Led Panel ...")

        request = SetLed.Request()
        request.led_number = n
        request.state = s

        future_ = client_.call_async(request)
        future_.add_done_callback(
            partial(self.callback_call_battery_status))

    def callback_call_battery_status(self, future_):
        # try catch statement
        try:
            response = future_.result()
            self.get_logger().info(
                f"The led status changed: {response.success}")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
