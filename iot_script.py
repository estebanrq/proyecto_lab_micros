import serial, time, json
import paho.mqtt.client as mqtt

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        client.connected = True
        print("Conexión exitosa")
    else: 
        print("No conectado, código: ", rc)
        client.loop_stop()
def on_disconnect(client, userdata, rc):
    if(rc == 0):
        print("Desconexión exitosa")
    else:
        print("Sistema desconectado mediante el código: ", rc)
def on_publish(client, userdata, mid):
    print("Mensaje: ", mid, " ha abandonado el cliente")


datos = serial.Serial("/dev/ttyACM0",115200,timeout=1) 
print("Conectado al puerto serial /dev/ttyACM1")
client = mqtt.Client("B97333")
client.connected = False
client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.on_publish = on_publish

broker ="iot.eie.ucr.ac.cr"
port = 1883
topic = "v1/devices/me/telemetry"
device = "7ZUie6OGc2UJvSje3UWS"
client.username_pw_set(device)
client.connect(broker, port)
dict = dict()
while client.connected != True:
    client.loop()
    time.sleep(2)

while (1):
    data = datos.readline().decode('utf-8').replace('\r', "").replace('\n', "")
    data = data.split(',')
    #giroscopio
    dict["Eje GX"] = data[0]
    dict["Eje GY"] = data[1]
    dict["Eje GZ"] = data[2]

    # aceleracion
    dict["Eje AX"] = data[3]
    dict["Eje AY"] = data[4]
    dict["Eje AZ"] = data[5]

    # campo magnetico
    dict["Campo Magnetico X"] = data[6]
    dict["Campo Magnetico Y"] = data[7]
    dict["Campo Magnetico Z"] = data[8]

    # temperatura
    dict["Temperatura"] = data[9]

    # humedad
    dict["Humedad"] = data[10]
    
    # varometro
    dict["Presion Atmosferica"] = data[11]

    # proximidad
    dict["Proximidad Izq"] = data[12]
    dict["Proximidad Cen"] = data[13]
    dict["Proximidad Der"] = data[14]

    # mov de ruedas
    dict["Velocidad Rueda Izq"] = data[15]
    dict["Velocidad Rueda Der"] = data[16]

    # posicion
    dict["Posicion X"] = data[17]
    dict["Posicion Y"] = data[18]

    
    output = json.dumps(dict)
    print(output)
    client.publish(topic, output)
    time.sleep(5)