import camera
import network
import socket
import time

def init_camera():
    # Inicializar la cámara
    try:
        camera.deinit()
    except:
        pass
    camera.init(0, format=camera.JPEG)
    camera.framesize(camera.FRAME_QQVGA)  # 160x120

def connect_wifi(ssid, password):
    # Conectar a la red WiFi
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, password)
    print('Conectando a la red WiFi...')
    while not wlan.isconnected():
        time.sleep(1)
    print('Conexión exitosa!')
    print('Configuración de red:', wlan.ifconfig())
    return wlan.ifconfig()[0]

def start_server():
    # Iniciar el servidor web
    addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
    s = socket.socket()
    s.bind(addr)
    s.listen(1)
    print('Servidor escuchando en el puerto 80')
    return s

def main_loop():
    ssid = 'RED-BERGUECIO-2.4G'          # Reemplaza con tu SSID
    password = 'Colchagua81'  # Reemplaza con tu contraseña
    ip = connect_wifi(ssid, password)
    init_camera()
    s = start_server()
    print('Abre tu navegador y visita http://{}'.format(ip))

    while True:
        cl, addr = s.accept()
        print('Cliente conectado desde', addr)
        request = cl.recv(1024)
        request = str(request)
        print('Solicitud:', request)

        if 'GET / ' in request:
            buf = camera.capture()
            cl.send('HTTP/1.0 200 OK\r\nContent-Type: image/jpeg\r\n\r\n')
            cl.send(buf)
        else:
            cl.send('HTTP/1.0 404 NOT FOUND\r\n\r\n')
        cl.close()
