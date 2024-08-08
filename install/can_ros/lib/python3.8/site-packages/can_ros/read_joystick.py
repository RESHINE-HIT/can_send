import can  # python-can  4.3.1
import struct
import math


class EncoderDriver(object):
    def __init__(self, motor_id, can_interface):
        self.inited = False
        self.board_id = motor_id
        self.can_interface = can_interface
        self.bus = can.interface.Bus(channel=can_interface, bustype="socketcan")
        self.ret_cmd_id = 0x588
        self.motor_pos = 0.0
        self.ret_id = 0
        self.notifier = can.Notifier(self.bus, [self._can_callback], timeout=1.0)
        self.inited = True
        self.x_value=None
        self.y_value=None

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

    def _can_callback(self, msg: can.Message):
        if msg.arbitration_id == 0x522:  # is this motor
            data=msg.data[0]
            print("Buttom value :",data)


        if msg.arbitration_id == 0x588:  # is this motor
            data=msg.data[0]
            
            if data & 0x80:  # 检查最高位是否为1
                    self.x_value = data & 0x7F  # 移除标志位
                    print("X Value :", self.x_value)
           
    
            else :  # 最高位为0
                    self.y_value = data & 0x7F  # 移除标志位
                    print("Y Value :", self.y_value)

            #mess=self.bus.recv()
            #print(mess)
        #print(msg.data[0])                

# Usage example
if __name__ == "__main__":
    import time

    encoder_driver = EncoderDriver(588, "can0")
    encoder_driver = EncoderDriver(522, "can0")
    # encoder_driver.MotorInit()
    # encoder_driver.MotorPosModeCmd(1.0, 1.0, False)
    while True:
        # encoder_driver.mit_cmd(1.0, 1.0, 1.0, 1.0, 1.0)
        time.sleep(0.5)
        print(encoder_driver.x_value)
    # encoder_driver.MotorSetZero()
