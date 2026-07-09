import time
import Jetson.GPIO as GPIO

# Physical pins
INPUT_LIDAR = 7
OUTPUT_LIDAR = 8
INPUT_VIZ = 11
OUTPUT_VIZ = 12

class pins:
    """
    Verify pins (gpio) of a NVIDIA carrier board
    Using a Jetson Orin Nano Developer Kit Super (Yahboom)
    """
    def __init__(self):
        GPIO.setmode(GPIO.BCM)

        # The value may be floats, so we'll probably have to add external pull-down resistor (10k ohm) or configure the pinmux as one
        # pull_up_down=GPIO.PUD_DOWN is ignore on Jetson
        GPIO.setup(INPUT_LIDAR, GPIO.IN)
        GPIO.setup(INPUT_VIZ, GPIO.IN)

        GPIO.setup(OUTPUT_LIDAR, GPIO.OUT)
        GPIO.setup(OUTPUT_VIZ, GPIO.OUT)

        GPIO.output(OUTPUT_LIDAR, GPIO.HIGH)
        GPIO.output(OUTPUT_VIZ, GPIO.HIGH)

    def tracking(self, lidar_state, viz_state):
        while True:
            lidar_state.value = self.verify_pin(INPUT_LIDAR)
            viz_state.value = self.verify_pin(INPUT_VIZ)
            time.sleep(0.2)

    def verify_pin(self, pin):
        # Verify if there is a jumper on pin and pin+1
        return GPIO.input(pin)

    def cleanup(self):
        GPIO.cleanup()
