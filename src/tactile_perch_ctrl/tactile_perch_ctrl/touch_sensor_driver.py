import time
# Import MPR121 module.
import board
import adafruit_mpr121
import busio
import rclpy
from rclpy.node import Node
from rclpy import qos

from custom_msgs.msg import TouchData

MINIMUM_CAPACITANCE_DIFFERENCE = -5
NUMBER_OF_TOUCH_PADS = 12

class TouchSensorDriver(Node):

    def __init__(self):
        super().__init__('mpr121_publisher')
        self._touch_data_publisher = self.create_publisher(TouchData, '/touch_sensor/data', qos.QoSPresetProfiles.SENSOR_DATA.value)

        timer_period = 1e-3  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Create I2C bus.
        self.i2c = busio.I2C(board.SCL, board.SDA)
        # Create MPR121 object.
        self.mpr121 = adafruit_mpr121.MPR121(self.i2c)
        self.sensor_channels = [adafruit_mpr121.MPR121_Channel(self.mpr121, i) for i in range(NUMBER_OF_TOUCH_PADS)]
        self._local_time = time.monotonic_ns()
        self._remote_time = time.monotonic_ns()
        self._init_state = self.check_for_raw_data()
    
    def timer_callback(self):

        # touch_event = self.check_for_touch_event()
        # touch_msg = StampedInt32MultiArray()
        # touch_msg.data = touch_event
        # touch_msg.timestamp = self._get_timestamp()
        # self._touch_publiser.publish(touch_msg)

        msg = TouchData()
        msg.header.timestamp = rclpy.now().to_msg()
        
        msg.raw_data = self.check_for_raw_data()
        msg.filtered_data = self.check_for_filtered_data()
        msg.baseline_data = self.check_for_baseline_data()
        
        self._touch_data_publisher.publish(msg)

    def check_for_baseline_data(self):
        return [self.mpr121.baseline_data(i) for i in range(NUMBER_OF_TOUCH_PADS)]
    
    def check_for_raw_data(self):
        return [self.sensor_channels[i].raw_value for i in range(NUMBER_OF_TOUCH_PADS)]
    
    def check_for_filtered_data(self):
        return [self.mpr121.filtered_data(i) for i in range(NUMBER_OF_TOUCH_PADS)]


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = TouchSensorDriver()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()