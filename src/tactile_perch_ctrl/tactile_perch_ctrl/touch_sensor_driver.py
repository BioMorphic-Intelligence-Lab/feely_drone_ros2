import time
from typing import List, Tuple

# Import MPR121 module.
import board
import adafruit_mpr121
import busio
import rclpy
from rclpy.node import Node
from rclpy import qos

from custom_msgs.msg import TouchData

MINIMUM_CAPACITANCE_DIFFERENCE = -5
NUMBER_OF_TOUCH_PADS = 9


class TouchSensorDriver(Node):

    def __init__(self):
        super().__init__("mpr121_publisher")
        self._touch_data_publisher = self.create_publisher(
            TouchData,
            "/feely_drone/out/touch_data",
            qos.QoSPresetProfiles.SENSOR_DATA.value,
        )

        # Get parameters
        self.declare_parameter("frequency", 25.0)
        self.frequency = (
            self.get_parameter("frequency").get_parameter_value().double_value
        )

        # Backoff state for sensor reinitialization
        self._reinit_backoff_seconds = 0.1
        self._reinit_backoff_max_seconds = 5.0
        self._next_reinit_time_ns = 0

        # Initialize sensor hardware
        self.i2c = None
        self.mpr121 = None
        self.sensor_channels = None
        self._init_sensor(first_init=True)

        self.timer = self.create_timer(1.0 / self.frequency, self.timer_callback)

        self._local_time = time.monotonic_ns()
        self._remote_time = time.monotonic_ns()

    def _init_sensor(self, first_init: bool = False) -> None:
        """Initialize or reinitialize the I2C bus and MPR121 device."""
        now_ns = time.monotonic_ns()
        if not first_init and now_ns < self._next_reinit_time_ns:
            # Still in backoff window
            return

        try:
            if first_init:
                self.get_logger().info("Initializing MPR121 touch sensor...")
            else:
                self.get_logger().warn("Reinitializing MPR121 touch sensor...")

            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.mpr121 = adafruit_mpr121.MPR121(self.i2c)
            self.sensor_channels = [
                adafruit_mpr121.MPR121_Channel(self.mpr121, i)
                for i in range(NUMBER_OF_TOUCH_PADS)
            ]

            self.get_logger().info("MPR121 initialized successfully.")
            # Reset backoff on success
            self._reinit_backoff_seconds = 0.1
            self._next_reinit_time_ns = 0
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MPR121: {e}")
            self.i2c = None
            self.mpr121 = None
            self.sensor_channels = None

            # Exponential backoff
            self._next_reinit_time_ns = now_ns + int(
                self._reinit_backoff_seconds * 1e9
            )
            self._reinit_backoff_seconds = min(
                self._reinit_backoff_seconds * 2.0, self._reinit_backoff_max_seconds
            )

    def _read_channel_values(self) -> Tuple[List[int], List[int], List[int], List[bool]]:
        """Read all channels with per-channel error handling.

        Returns:
            raw_data, filtered_data, baseline_data, valid
        """
        raw_data: List[int] = []
        filtered_data: List[int] = []
        baseline_data: List[int] = []
        valid: List[bool] = []

        if self.mpr121 is None or self.sensor_channels is None:
            # No sensor available; mark everything invalid and schedule reinit
            self._init_sensor()
            for _ in range(NUMBER_OF_TOUCH_PADS):
                raw_data.append(0)
                filtered_data.append(0)
                baseline_data.append(0)
                valid.append(False)
            return raw_data, filtered_data, baseline_data, valid

        for i in range(NUMBER_OF_TOUCH_PADS):
            try:
                raw_val = self.sensor_channels[i].raw_value
                filt_val = self.mpr121.filtered_data(i)
                base_val = self.mpr121.baseline_data(i)

                raw_data.append(int(raw_val))
                filtered_data.append(int(filt_val))
                baseline_data.append(int(base_val))
                valid.append(True)
            except (OSError, ValueError) as e:
                # Typical I2C / device errors for this channel
                self.get_logger().warn(
                    f"I2C/MPR121 read error on channel {i}: {e}. Marking channel invalid."
                )
                raw_data.append(0)
                filtered_data.append(0)
                baseline_data.append(0)
                valid.append(False)
            except Exception as e:
                # Catch-all to avoid crashing the node
                self.get_logger().error(
                    f"Unexpected error reading MPR121 channel {i}: {e}"
                )
                raw_data.append(0)
                filtered_data.append(0)
                baseline_data.append(0)
                valid.append(False)

        # If all channels failed, likely a bus-level issue; schedule a reinit attempt
        if not any(valid):
            self._init_sensor()

        return raw_data, filtered_data, baseline_data, valid

    def timer_callback(self):
        msg = TouchData()
        msg.header.stamp = self.get_clock().now().to_msg()

        raw_data, filtered_data, baseline_data, valid = self._read_channel_values()

        msg.raw_data = raw_data
        msg.filtered_data = filtered_data
        msg.baseline_data = baseline_data
        msg.valid = valid

        self._touch_data_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    touch_sensor_driver = TouchSensorDriver()

    rclpy.spin(touch_sensor_driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    touch_sensor_driver.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()