program_name = "hand" # "control", "receptor", "mac"
import machine
import time

def led_blink(times=0, delay=0.3):
    led = machine.Pin(2, machine.Pin.OUT)
    if times == 0:
        while True:
            led.value(not led.value())
            time.sleep(delay)
    else:
        for _ in range(times*2):
            led.value(not led.value())
            time.sleep(delay)

try:
    print("Starting program:", program_name)
    if program_name == "mac":
        import mac
        led_blink(1)
        mac
    elif program_name == "hand":
        import hand
        led_blink(2)
        hand.main_loop()

except Exception as e:
    print("An error occurred:\n", str(e))
    led_blink(100, 0.1)