from machine import Pin, PWM
import network
import socket
import time

# Conectarse a la red Wi-Fi
SSID = 'RED-BERGUECIO-2.4G'
PASSWORD = 'Colchagua81'

# Pines GPIO conectados a los servos
SERVO_PINS = [16, 17, 18]
MOTOR_PIN_FORWARD = 19

# Frecuencia PWM
PWM_FREQ = 50  # 50 Hz (período de 20 ms)

STOP = 76  #73-79      
FORWARD_MAX = [128,96,91,128]
REVERSE_MAX = [25,50,60,25]

# Página HTML con botones para controlar los servos
HTML_PAGE = """<!DOCTYPE html>
<html>
<head>
    <title>Control de Servos</title>
    <style>
        body {{ font-family: Arial, sans-serif; text-align: center; }}
        .servo {{ margin: 20px; }}
        button {{ padding: 10px 20px; margin: 5px; }}
    </style>
</head>
<body>
    {response}
    <h1>Control de Servos</h1>
    {buttons}
</body>
</html>
"""


MOTOR_PWM_FORWARD = PWM(Pin(MOTOR_PIN_FORWARD))
MOTOR_PWM_FORWARD.freq(500)

def set_motor_speed(speed, s_time=0):
    """Controla la velocidad del motor mediante un puente H usando PWM."""
    speed = min(max(speed, -100), 100)

    if speed > 0:
        # Control de avance con PWM
        duty_cycle = int((speed / 100) * 1023)  # Convierte a valor de duty cycle (0-1023)
        MOTOR_PWM_FORWARD.duty(duty_cycle)
    else:
        # Detener el motor
        MOTOR_PWM_FORWARD.duty(0)

    if s_time > 0:
        time.sleep_ms(s_time)
        # Detener el motor después del tiempo especificado
        MOTOR_PWM_FORWARD.duty(0)
        print(f"Motor ajustado a velocidad {speed} durante {s_time} ms")
    else:
        print(f"Motor ajustado a velocidad {speed}")

def generar_botones_html():
    botones = ""
    botones += f"""
        <div class="servo">
            <h2>Servo 0</h2>
            <button onclick="location.href='/?servo=0&speed=50&s_time=500'">Avanzar 0,5</button>
            <button onclick="location.href='/?servo=0&speed=50&s_time=200'">Avanzar 0,2s</button>
            <button onclick="location.href='/?servo=0&speed=0'">Detener</button>
            <button onclick="location.href='/?servo=0&speed=-50&s_time=200'">Retroceder 0,2s</button>
            <button onclick="location.href='/?servo=0&speed=-50&s_time=500'">Retroceder 0,5</button>
        </div>
        """
    for i in range(1,3):
        botones += f"""
        <div class="servo">
            <h2>Servo {i}</h2>
        """
        for j in range(-100,120,20):
            botones += f"""<button onclick="location.href='/?servo={i}&speed={j}'">Avanzar {j}</button>"""
        botones += f"""
        </div>
        """
    botones += f"""
    <div class="motor">
        <h2>Motor</h2>
        <button onclick="location.href='/?servo=3&speed=100'">Bombear</button>
        <button onclick="location.href='/?servo=3&speed=50'">Bombeo leve</button>
        <button onclick="location.href='/?servo=3&speed=50&s_time=1000'">Bombeo leve 1s</button>
        <button onclick="location.href='/?servo=3&speed=0'">Detener</button>
    </div>
    """
    return botones

def inicializar_servos():
    servos = []
    for pin_num in SERVO_PINS:
        pwm = PWM(Pin(pin_num))
        pwm.freq(PWM_FREQ)
        pwm.duty(STOP)  # Posición inicial (detenido)
        servos.append(pwm)
    print("Servos inicializados.")
    return servos

def conectar_wifi():
    station = network.WLAN(network.STA_IF)
    station.active(True)
    station.connect(SSID, PASSWORD)
    
    print('Conectando a la red Wi-Fi...')
    while not station.isconnected():
        time.sleep(0.5)
    print('Conexión exitosa')
    print('Configuración de red:', station.ifconfig())

def configurar_servidor():
    addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
    s = socket.socket()
    s.bind(addr)
    s.listen(1)
    print('Servidor escuchando en', addr)
    return s

def set_speed(servo_index, speed):
    if not (-100 <= speed <= 100):
        speed = min(max(speed, -100), 100)
    if speed > 0:
        servo_speed = STOP + (FORWARD_MAX[servo_index] - STOP) * speed / 100
    elif speed < 0:
        servo_speed = STOP + (STOP - REVERSE_MAX[servo_index]) * speed / 100
    else:
        servo_speed = STOP  # Detener el servo
    return servo_speed

def inverse_speed(servo_index, duty):
    if duty > STOP: 
        speed = 100 * (duty - STOP) / (FORWARD_MAX[servo_index] - STOP)
    elif duty < STOP:
        speed = 100 * (duty - STOP) / (STOP - REVERSE_MAX[servo_index])
    else:
        speed = 0
    return speed

def set_servo_speed(servos, servo_index, speed, s_time=0):
    if not (-100 <= speed <= 100):
        speed = min(max(speed, -100), 100)
    servo_speed = set_speed(servo_index, speed)

    if servo_index == 1 and speed < -60:
        min_position = -2 * speed - 210
        position_s2 = inverse_speed(2, servos[2].duty())
        print(f"Posicion s2: {position_s2}")
        position_s2 = max(position_s2, min_position)
        servo_position_s2 = set_speed(2, position_s2)
        servos[2].duty(int(servo_position_s2))
        print(f"Servo 2 ajustado a pocicion {position_s2}")

    elif servo_index == 2 and speed < -20:
        min_position = -speed / 2 - 100
        position_s1 = inverse_speed(1, servos[1].duty())
        print(f"Posicion s1: {position_s1}")
        position_s1 = max(position_s1, min_position)
        servo_position_s1 = set_speed(1, position_s1)
        servos[1].duty(int(servo_position_s1))
        print(f"Servo 1 ajustado a pocicion {position_s1}")
    
    servo = servos[servo_index]
    servo.duty(int(servo_speed))

    if s_time > 0:
        time.sleep_ms(s_time)
        servo.duty(STOP)  # Detener el servo después del tiempo especificado
        print(f"Servo {servo} ajustado a velocidad {servo_speed} por: {s_time} ms)")
    else:
        print(f"Servo {servo} ajustado a velocidad {servo_speed}")

def manejar_solicitud(cl):
    print('Manejando solicitud...')
    servo_index = -1
    speed = 0
    s_time = 0
    try:
        request = cl.recv(1024)
        request_str = request.decode()
        print('Solicitud:', request_str)
        if '?' in request_str:
            _, params = request_str.split('?', 1)
            params = params.split(' ')[0]
            param_dict = {}
            for param in params.split('&'):
                key, value = param.split('=')
                param_dict[key] = value
            servo_index = int(param_dict.get('servo', -1))
            speed = int(param_dict.get('speed', 0))
            s_time = int(param_dict.get('s_time', 0))
            response = f'Servo: {servo_index}, Velocidad: {speed}, Tiempo: {s_time}\n'
        else:
            response = 'Formato de solicitud inválido\n'
    except Exception as e:
        response = f'Error al procesar la solicitud: {e}\n'
    print(response)
    return response, servo_index, speed, s_time

def respond_web(response, cl):
    cl.send('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
    cl.send(response)
    cl.close()

def main_loop():
    servos = inicializar_servos()
    # Conectarse a Wi-Fi
    conectar_wifi()
    # Configurar el servidor
    servidor = configurar_servidor()
    # Generar la página HTML con botones
    
    # Bucle principal para aceptar conexiones
    while True:
        try:
            print('Esperando conexión...')
            cl, addr = servidor.accept()
            print('Cliente conectado desde', addr)
            response, servo_index, speed, s_time = manejar_solicitud(cl)
            if 0 <= servo_index < len(servos):
                set_servo_speed(servos, servo_index, speed, s_time)
                #respond_web(response, cl)
                respond_web(HTML_PAGE.format(buttons=generar_botones_html(), response=response), cl)
            elif servo_index == 3:
                set_motor_speed(speed, s_time)
                respond_web(HTML_PAGE.format(buttons=generar_botones_html(), response=response), cl)
            else:
                respond_web(f"Índice de servo inválido {servo_index}", cl)
        except Exception as e:
            print('Error en el servidor:', e)
            time.sleep(1)  # Evita bucles rápidos en caso de error