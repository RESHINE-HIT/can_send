import can  # python-can  4.3.1
import time


class JoystickDriver(object):
    def __init__(self, can_interface):
        self.inited = False
        self.can_interface = can_interface
        self.bus = can.interface.Bus(channel=can_interface, bustype="socketcan")
        self.ret_cmd_id = {"stick": 0x588, "button": 0x522}
        self.notifier = can.Notifier(self.bus, [self._can_callback], timeout=1.0)
        self.inited = True
        self.x_value = None
        self.y_value = None
        self.button_state = None
        # 消抖相关变量
        self.button_state_temp = 0
        self.last_button_state = 0
        self.last_debounce_time = 0
        self.debounce_delay = 0.1  # 消抖时间，单位：秒

    # def __del__(self):
    #     if self.inited:
    #         self.notifier.stop()

    # def pos_cmd(self, pos, spd, ignore_limit):
    #     tx_frame = can.Message(
    #         arbitration_id=self.board_id, data=[0x07], is_extended_id=False
    #     )
    #     self._can_send(tx_frame)

    # def mit_cmd(self, f_p, f_v, f_kp, f_kd, f_t):
    #     tx_frame = can.Message(
    #         arbitration_id=self.board_id, dlc=0x01, data=[0x07], is_extended_id=False
    #     )
    #     self._can_send(tx_frame)

    # def set_zero(self):
    #     can_id = self.board_id | 0x80
    #     tx_frame1 = can.Message(
    #         arbitration_id=can_id, dlc=0x01, data=[0x0B], is_extended_id=False
    #     )
    #     self._can_send(tx_frame1)
    #     tx_frame2 = can.Message(
    #         arbitration_id=can_id, dlc=0x01, data=[0x0C], is_extended_id=False
    #     )
    #     self._can_send(tx_frame2)
    #     return True

    # def get_sSFtate(self):
    #     return self.motor_pos

    # def _can_send(self, tx_frame):
    #     self.bus.send(tx_frame)

    def get_state(self):
        return self.x_value, self.y_value, self.button_state

    def _can_callback(self, msg: can.Message):
        if msg.arbitration_id == self.ret_cmd_id["button"]:
            data = msg.data[0]
            current_time = time.time()
            # self.button_state = data
            # 仅当按键状态变化时处理消抖逻辑
            if data != self.last_button_state:
                self.last_debounce_time = current_time

            if (current_time - self.last_debounce_time) > self.debounce_delay:
                if data != self.button_state_temp:
                    self.button_state_temp = data
                else:
                    self.button_state = data
            self.last_button_state = data

        if msg.arbitration_id == self.ret_cmd_id["stick"]:
            data = msg.data[0]
            if data & 0x80:  # 检查最高位是否为1
                self.x_value = data & 0x7F  # 移除标志位
                # print("X Value :", self.x_value)
            else:  # 最高位为0
                self.y_value = data & 0x7F  # 移除标志位
                # print("Y Value :", self.y_value)

            # mess=self.bus.recv()
            # print(mess)
        # print(msg.data[0])


# Usage example
if __name__ == "__main__":
    encoder_driver = JoystickDriver("can0")
    while True:
        time.sleep(0.5)
        print(encoder_driver.get_state())
