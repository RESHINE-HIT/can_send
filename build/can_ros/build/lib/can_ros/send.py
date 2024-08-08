import can  # python-can  4.3.1
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int32
import time

import struct
import math

class EncoderDriver(Node):
    def __init__(self, motor_id, can_interface):
        super().__init__('encoder_driver')
        self.publisher_x = self.create_publisher(Float32, 'x_value', 10)
        self.publisher_y = self.create_publisher(Float32, 'y_value', 10)
        self.publisher_button = self.create_publisher(Int32, 'button_data', 10)
        self.publisher_encoder = self.create_publisher(Float32, 'encoder', 10)

        self.inited = False
        self.board_id = motor_id
        self.can_interface = can_interface
        self.bus = can.interface.Bus(channel=can_interface, bustype="socketcan")
        self.move_id = 0x588 #摇杆
        self.button_cmd_id = 0x522 #按钮
        #self.motor_id=2 #编码器
        self.notifier = can.Notifier(self.bus, [self._can_callback], timeout=1.0)
        self.inited = True
        self.x_value = None
        self.y_value = None
        self.button_data = None
        self.encoder = None
        self.motor_pos = 0.0
        OPERATE_CMD = 0b0000
        RET_CMD = OPERATE_CMD | 2
        self.ret_cmd_id = self.board_id | (RET_CMD << 7)

       # 消抖相关变量
        self.button_state = 0
        self.last_button_state = 0
        self.last_debounce_time = 0
        self.debounce_delay = 0.05  # 消抖时间，单位：秒

    def __del__(self):
        if self.inited:
            self.notifier.stop()

    def mit_cmd(self, f_p, f_v, f_kp, f_kd, f_t):
        tx_frame = can.Message(
            arbitration_id=self.board_id, dlc=0x01, data=[0x07], is_extended_id=False
        )
        self._can_send(tx_frame)
    def _can_send(self, tx_frame):
        self.bus.send(tx_frame)

    def _can_callback(self, msg: can.Message):
        print("qqq")
        self.mit_cmd(1.0, 1.0, 1.0, 1.0, 1.0)
        if msg.arbitration_id == self.button_cmd_id:  # 按键数据
            data = msg.data[0]
            current_time = time.time()

            # 仅当按键状态变化时处理消抖逻辑
            if data != self.last_button_state:
                self.last_debounce_time = current_time

            if (current_time - self.last_debounce_time) > self.debounce_delay:
                if data != self.button_state:
                    self.button_state = data
                    button_value = self.button_state
                    self.publisher_button.publish(Int32(data=button_value))
                    self.get_logger().info(f'Button Value: {button_value}')
                else     :            
                    self.publisher_button.publish(Int32(data=data))
                    self.get_logger().info(f'Button Value: {data}')
            else:
                self.publisher_button.publish(Int32(data=self.last_button_state))
                self.get_logger().info(f'Button Value: {self.last_button_state}')
            self.last_button_state = data

            # if msg.arbitration_id == self.button_cmd_id:
            #     data = msg.data[0]
            #     self.button_data = data
            #     self.publisher_button.publish(Int32(data=data))
            #     self.get_logger().info(f'Button Value: {data}')

        if msg.arbitration_id == self.move_id:
            data = msg.data[0]
            if data & 0x80:  # Check if the highest bit is 1
                self.x_value = data & 0x7F  # 移除标志位
                self.x_value = -self.map_value(self.x_value, 93, True)
                self.publisher_x.publish(Float32(data=self.x_value))
                self.get_logger().info(f'X Value: {self.x_value}')
            else:  # Highest bit is 0
                self.y_value = (data & 0x7F)   # Map to -1 to 1
                self.y_value = self.map_value(self.y_value, 93, True)
                self.publisher_y.publish(Float32(data=self.y_value))
                self.get_logger().info(f'Y Value: {self.y_value}')
                
        if msg.arbitration_id == 0x102:  # is this motor
            #if msg.data[0] == 0x07:  # angle state
                data = msg.data[0]
                pos_bytes = msg.data[2:6]
                pos_ = struct.unpack("f", bytes(pos_bytes))[0]
                self.motor_pos = pos_ / 360.0 * 2.0 * math.pi
                self.publisher_encoder.publish(Float32(data=self.motor_pos))
                self.get_logger().info(f'Encoder Value: {self.motor_pos}')
                
       
    
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
    encoder_driver = EncoderDriver(2, "can1")
   # encoder_driver = EncoderDriver(2, "can0")
    rclpy.spin(encoder_driver)

    encoder_driver.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()