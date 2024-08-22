from machine import Pin, Timer, PWM, freq, mem32
from rp2 import asm_pio, StateMachine
import time

PWM_OUTPUT_PIN = 0
PULSE_INPUT_PIN = 2


# PIO program to count pulses
@asm_pio()
def pulse_counter_pio():
    set(x, 0)  # Reset counter
    wrap_target()
    label("count")
    # we will count the falling edge of the pulse
    wait(1, pin, 0)  # Wait for high pulse
    wait(0, pin, 0)  # Wait for low pulse
    jmp(x_dec, "count")  # Decrement counter
    wrap()


# Class to handle the pulse counting
class PulseCounter:
    def __init__(self, sm_id, pin, program):
        self.sm_id = sm_id
        self.pin = pin
        # Set the frequency to match the system clock
        self.sm = StateMachine(
            sm_id, program, freq=freq(), in_base=Pin(pin, Pin.IN, Pin.PULL_UP)
        )
        self.sm.active(1)
        self.counter = 0

    def read(self):
        self.sm.exec("mov(isr, x)")
        self.sm.exec("push()")
        self.counter = self.sm.get()
        return (0x100000000 - self.counter) & 0xFFFFFFFF

    def reset(self):
        self.sm.exec("set(x, 0)")  # Reset the counter

    def __del__(self):
        self.sm.active(0)


# Initialize the pulse counter for the timer method
pulse_counter_pio = PulseCounter(0, PULSE_INPUT_PIN, pulse_counter_pio)
pulse_frequency_pio = 0

time_us = time.ticks_us()


# Timer callback to read and reset the counter
def timer_callback_timer(timer):
    global pulse_counter_pio, pulse_frequency_pio, time_us
    # adjust the frequency based on the elapsed time
    pulse_frequency_pio = pulse_counter_pio.read() / (time.ticks_us() - time_us) * 1e6
    pulse_counter_pio.reset()
    # record the time of the last read
    time_us = time.ticks_us()


def main():
    try:
        # Generate testing PWM signal
        pwm_testing = PWM(Pin(PWM_OUTPUT_PIN, Pin.OUT))
        pwm_testing.freq(20)  # Set the desired frequency in Hz
        pwm_testing.duty_u16(32768)  # Set duty cycle to 50%

        # Set up the timer to periodically read the counter
        timer = Timer(period=1000, mode=Timer.PERIODIC, callback=timer_callback_timer)

        while True:
            time.sleep(1)
            print(
                f"Generated PWM Frequency: {pwm_testing.freq()} Hz, Measured frequency: {pulse_frequency_pio} Hz"
            )
    except KeyboardInterrupt:
        global pulse_counter_pio
        timer.deinit()
        pwm_testing.deinit()
        del pulse_counter_pio


if __name__ == "__main__":
    main()
