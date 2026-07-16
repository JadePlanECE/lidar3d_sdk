import time
import Jetson.GPIO as GPIO

# Physical pins
INPUT_LIDAR = 11
INPUT_VIZ = 13
TIME_CHECK = 0.2


class Pins:
    """
    Verify pins (gpio) of a NVIDIA carrier board
    Using a Jetson Orin Nano Developer Kit Super (Yahboom)
    """
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)

        GPIO.setup(INPUT_LIDAR, GPIO.IN)
        GPIO.setup(INPUT_VIZ, GPIO.IN)

    def tracking(self, lidar_state, viz_state):
        """Checking if the buttons are pressed"""
        try:
            # checking lidar button first (don't need vizualisation yet)
            print("[GPIO] Entering LiDAR loop")
            pre_value_lidar = GPIO.input(INPUT_LIDAR)
            while lidar_state.value:
                curr_value_lidar = GPIO.input(INPUT_LIDAR)
                if curr_value_lidar != pre_value_lidar:
                    print(f"[GPIO] Stoping LiDAR")
                    lidar_state.value = False
                time.sleep(TIME_CHECK)
            # checking vizualisation button (don't need lidar button anymore)
            print("[GPIO] Entering vizualisation loop")
            pre_value_viz = GPIO.input(INPUT_VIZ)
            while viz_state.value:
                curr_value_viz = GPIO.input(INPUT_VIZ)
                if curr_value_viz != pre_value_viz:
                    print(f"[GPIO] Stoping vizualisation")
                    viz_state.value = False
                time.sleep(TIME_CHECK)
        except KeyboardInterrupt:
            print("[GPIO] Keyboard interrupt")
        except Exception as e:
            print(f"[GPIO error] {e}")
        finally:
            GPIO.cleanup()
            print(f"[GPIO] Cleanup done")
