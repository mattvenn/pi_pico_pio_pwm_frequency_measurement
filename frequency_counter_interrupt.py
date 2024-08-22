from machine import Pin, PWM, Timer
import utime
import gc

PWM_OUTPUT_PIN = 0
PULSE_INPUT_PIN = 2


# Class to handle pulse counting using interrupts
class PulseCounterInterrupt:
    def __init__(self, pin):
        self.pin = pin
        self.last_time = 0
        self.current_time = utime.ticks_us()
        self.time_diff = 0

        self.counter = 0
        self.pin.irq(trigger=Pin.IRQ_FALLING, handler=self.callback)

    def callback(self, pin):
        # we only record time every 10 pulses to reduce the overhead
        self.counter += 1
        if self.counter == 20:
            self.last_time = self.current_time
            self.current_time = utime.ticks_us()
            self.counter = 0

    def get_frequency(self):
        self.time_diff = utime.ticks_diff(self.current_time, self.last_time)
        if self.time_diff > 0:
            return 2e7 / self.time_diff  # Convert microseconds to Hz
        else:
            return 0.0

    def deinit(self):
        self.pin.irq(handler=None)


# Initialize the pulse counter
pulse_counter_interrupt = PulseCounterInterrupt(Pin(PULSE_INPUT_PIN, Pin.IN))


def main():
    global pulse_counter_interrupt
    try:
        # Generate testing PWM signal
        pwm_testing = PWM(Pin(PWM_OUTPUT_PIN, Pin.OUT))
        pwm_testing.freq(10000)  # Set the desired frequency in Hz
        pwm_testing.duty_u16(32768)  # Set duty cycle to 50%

        while True:
            utime.sleep(1)
            print(
                f"Generated PWM Frequency: {pwm_testing.freq()} Hz, Measured Frequency: {pulse_counter_interrupt.get_frequency()} Hz"
            )
    except KeyboardInterrupt:
        pulse_counter_interrupt.deinit()
        pwm_testing.deinit()
        print("Exiting...")
        gc.collect()


if __name__ == "__main__":
    main()
