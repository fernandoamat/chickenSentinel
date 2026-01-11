import curses
from rpi_hardware_pwm import HardwarePWM
from gpiozero import DigitalOutputDevice
from time import sleep

"""
Python script that I can execute in the Raspberry PI where I control the servos 
with the arrow keys (I will be SSHing into the Raspberry Pi to run the Python script) 
and activate the water pump (i.e. "shoot" for 3 seconds) when pressing 
the letter s in my laptop keyboard.
"""

# --- HARDWARE CONFIGURATION ---
# PWM Channel 0 is usually GPIO 12 or 18
# PWM Channel 1 is usually GPIO 13 or 19
PUMP_PIN = 17
pan_servo = HardwarePWM(pwm_channel=0, hz=50) # GPIO 12
tilt_servo = HardwarePWM(pwm_channel=1, hz=50) # GPIO 13
pump = DigitalOutputDevice(PUMP_PIN)

# Start PWM
pan_servo.start(0)
tilt_servo.start(0)

# --- MAP ANGLES TO DUTY CYCLE ---
# Most servos: 2.5% = -90deg, 7.5% = 0deg, 12.5% = +90deg
def set_angle(pwm_device, angle):
    # Clamp angle between -90 and 90
    angle = max(min(90, angle), -90)
    
    # Convert angle to duty cycle (Equation: 2.5 + (angle + 90) / 180 * 10)
    duty_cycle = 2.5 + (angle + 90) / 18.0
    pwm_device.change_duty_cycle(duty_cycle)

pan_angle = 0
tilt_angle = 0
STEP = 5

def main(stdscr):
    global pan_angle, tilt_angle
    
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.clear()
    
    stdscr.addstr(0, 0, "--- CHICKEN SENTINEL (HARDWARE PWM) ---")
    
    # Initialize position
    set_angle(pan_servo, pan_angle)
    set_angle(tilt_servo, tilt_angle)

    while True:
        stdscr.addstr(5, 0, f"Pan: {pan_angle:>3}   Tilt: {tilt_angle:>3}")
        stdscr.refresh()

        try:
            key = stdscr.getch()
        except:
            key = -1

        if key != -1:
            if key == curses.KEY_RIGHT:
                pan_angle -= STEP
            elif key == curses.KEY_LEFT:
                pan_angle += STEP
            elif key == curses.KEY_UP:
                tilt_angle += STEP
            elif key == curses.KEY_DOWN:
                tilt_angle -= STEP
            elif key == ord('s'):
                pump.on()
                sleep(3)
                pump.off()
            elif key == ord('q'):
                break

            set_angle(pan_servo, pan_angle)
            set_angle(tilt_servo, tilt_angle)
            
        sleep(0.05)

    # Cleanup
    pan_servo.stop()
    tilt_servo.stop()

curses.wrapper(main)