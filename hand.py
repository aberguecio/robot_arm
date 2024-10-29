from machine import Pin, PWM
import network
import socket
import time

# Conectarse a la red Wi-Fi
SSID = 'RED-BERGUECIO-2.4G'
PASSWORD = 'Colchagua81'

# Pines GPIO conectados a los servos
SERVO_PINS = [16, 17, 18, 19]

# Frecuencia PWM
PWM_FREQ = 50  # 50 Hz (período de 20 ms)


STOP = 76  #73-79      
FORWARD_MAX = 132  
REVERSE_MAX = 21

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

def set_servo_speed(servo, speed, s_time = 0):
    if not(-100 <= speed <= 100):
        speed = min(max(speed, -100), 100)
    if speed > 0:
        servo_speed = STOP + (FORWARD_MAX - STOP) * speed / 100
    elif speed < 0:
        servo_speed = STOP + (STOP - REVERSE_MAX) * speed / 100
    else:
        servo_speed = STOP  # Detener el servo
    servo.duty(int(servo_speed))
    if s_time >= 50:
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
    print (response)
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
    # Bucle principal para aceptar conexiones
    while True:
        try:
            print('Esperando conexión...')
            cl, addr = servidor.accept()
            print('Cliente conectado desde', addr)
            response, servo_index, speed, s_time = manejar_solicitud(cl)
            if 0 <= servo_index < len(servos):
                set_servo_speed(servos[servo_index], speed, s_time)
                respond_web(response, cl)
            else:
                respond_web(f"Índice de servo inválido {servo_index}", cl)

        except Exception as e:
            print('Error en el servidor:', e)
            time.sleep(1)  # Evita bucles rápidos en caso de error