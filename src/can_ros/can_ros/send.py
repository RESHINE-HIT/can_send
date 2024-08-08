import can  # python-can  4.3.1
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Float32MultiArray
from sensor_msgs.msg import JointState
import time

import struct
import math
from read_encoder import EncoderDriver
from read_joystick import JoystickDriver

# joint_msg = JointState()
# joint_msg.name = ["x", "y", "button", "encoder"]
# joint_msg.position = []


class CmdSendNode(Node):
    def __init__(self, encoder: EncoderDriver, joystick: JoystickDriver):
        super().__init__("cmd_send_node")
        self.encoder = encoder
        self.joystick = joystick
        self.publisher_x = self.create_publisher(Float32, "x_value", 10)
        self.publisher_y = self.create_publisher(Float32, "y_value", 10)
        self.publisher_button = self.create_publisher(Int32, "button_data", 10)
        self.publisher_encoder = self.create_publisher(Float32, "encoder_value", 10)
        self.timer = self.create_timer(0.01, self._publish_cmd)

    def _publish_cmd(self):
        x, y, b = self.joystick.get_state()
        encoder_pos = self.encoder.get_state()
        self.publisher_button.publish(Int32(data=int(b)))
        self.get_logger().info(f"Button Value: {b}")
        x = -self.map_value(x, 93, True)
        self.publisher_x.publish(Float32(data=x))
        self.get_logger().info(f"X Value: {x}")
        y = self.map_value(y, 93, True)
        self.publisher_y.publish(Float32(data=y))
        self.get_logger().info(f"Y Value: {y}")


        self.publisher_encoder.publish(Float32(data=-encoder_pos))
        self.get_logger().info(f"Encoder Value: {encoder_pos}")



    def map_value(self, value, center, is_x):
        if value < center:
            mapped_value = (value - center) / center
        else:
            mapped_value = (value - center) / (127 - center)

        # 限幅算法：在输出绝对值小于0.1时输出为0
        if abs(mapped_value) < 0.2:
            mapped_value = 0.0

        return mapped_value


def main(args=None):
    rclpy.init(args=args)
    import time

    joystick_driver = JoystickDriver("can0")
    encoder_driver = EncoderDriver(2, "can1")

    #time.sleep(1)

    cmd_send_node = CmdSendNode(encoder_driver, joystick_driver)

    try:
        rclpy.spin(cmd_send_node)
    finally:
        cmd_send_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
