import can  # python-can  4.3.1
import struct
import math


class EncoderDriver(object):
    def __init__(self, motor_id, can_interface):
        self.inited = False
        self.board_id = motor_id
        self.can_interface = can_interface
        self.bus = can.interface.Bus(channel=can_interface, bustype="socketcan")

        OPERATE_CMD = 0b0000
        RET_CMD = OPERATE_CMD | 2
        self.ret_cmd_id = self.board_id | (RET_CMD << 7)
        self.motor_pos = 0.0
        self.ret_id = 0
        self.notifier = can.Notifier(self.bus, [self._can_callback], timeout=1.0)
        self.inited = True

    def __del__(self):
        if self.inited:
            self.notifier.stop()

    def pos_cmd(self, pos, spd, ignore_limit):
        tx_frame = can.Message(
            arbitration_id=self.board_id, data=[0x07], is_extended_id=False
        )
        self._can_send(tx_frame)

    def mit_cmd(self, f_p, f_v, f_kp, f_kd, f_t):
        tx_frame = can.Message(
            arbitration_id=self.board_id, dlc=0x01, data=[0x07], is_extended_id=False
        )
        self._can_send(tx_frame)

    def set_zero(self):
        can_id = self.board_id | 0x80
        tx_frame1 = can.Message(
            arbitration_id=can_id, dlc=0x01, data=[0x0B], is_extended_id=False
        )
        self._can_send(tx_frame1)
        tx_frame2 = can.Message(
            arbitration_id=can_id, dlc=0x01, data=[0x0C], is_extended_id=False
        )
        self._can_send(tx_frame2)
        return True

    def get_state(self):
        return self.motor_pos

    def _can_send(self, tx_frame):
        self.bus.send(tx_frame)

    def _can_callback(self, msg: can.Message):
        if msg.arbitration_id == self.ret_cmd_id:  # is this motor
            if msg.data[0] == 0x07:  # angle state
                pos_bytes = msg.data[2:6]
                pos_ = struct.unpack("f", bytes(pos_bytes))[0]
                self.motor_pos = pos_ / 360.0 * 2.0 * math.pi
                self.ret_id = self.board_id
                
             
            #print(msg)

# Usage example
if __name__ == "__main__":
    import time

    encoder_driver = EncoderDriver(2, "can1")
    # encoder_driver.MotorInit()
    # encoder_driver.MotorPosModeCmd(1.0, 1.0, False)
    while True:
        encoder_driver.mit_cmd(1.0, 1.0, 1.0, 1.0, 1.0)
        time.sleep(0.1)
        print(encoder_driver.motor_pos)
        #print(encoder_driver.ret_cmd_id)
    # encoder_driver.MotorSetZero()
