from machine import Pin, PWM
import time

# Configuración del servo en el pin 16
MEASURE_PIN = 21

servo = PWM(Pin(25))
servo.freq(50)  # Frecuencia de 50Hz para servos
servo2 = PWM(Pin(26))
servo2.freq(50)

last_time = 0
pulse_width = 0
measuring = False

def medir_pulse(pin):
    global last_time, pulse_width, measuring
    current_time = time.ticks_us()
    if pin.value() == 1 and not measuring:
        last_time = current_time
        measuring = True
    elif pin.value() == 0 and measuring:
        pulse_width = time.ticks_diff(current_time, last_time)
        measuring = False
        print(f"Pulso medido: {pulse_width} ns")

def setup_measure_pin():
    pin = Pin(MEASURE_PIN, Pin.IN)
    pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=lambda p: medir_pulse(p))
    print(f"Pin {MEASURE_PIN} configurado para medir PWM.")

def main_loop():
    setup_measure_pin()
    index = 20
    while True:
        print (index)
        servo.duty(index)
        servo2.duty(index)
        time.sleep(0.1)
        index += 1
        if index > 130:
            index = 20


