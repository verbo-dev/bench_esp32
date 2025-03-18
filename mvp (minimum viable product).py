from google.cloud import pubsub_v1
import paho.mqtt.client as mqtt
from datetime import datetime
import pytz
from time import sleep
import os
import json

os.getenv('GOOGLE_APPLICATION_CREDENTIALS')

client = "la_caldera/+"
topic_buffer = ""
msg_buffer = ""

def on_message(client, userdata, msg):
    global topic_buffer
    global msg_buffer
    topic_buffer = msg.topic
    msg_buffer = msg.payload.decode()

class mqtt_google:
    def __init__(self, client_):
        # Google Cloud initial parameters
        self.publisher = pubsub_v1.PublisherClient()
        self.project_id = "bench-e00e6"
        # MQTT initial parameters
        self.client = mqtt.Client(client_id="local_python")
        self.client.on_message = on_message
        self.client.connect("localhost", 1883)
        self.client.subscribe(client_)
        self.client.subscribe("clientes")
        self.client.subscribe("dispositivos")

    def wait_msg(self):
        self.client.loop_start()

    def publish_msg(self, topic, msg):
        topic_id = topic.replace('/', '-')
        topic_path = self.publisher.topic_path(self.project_id, topic_id)
        try:
            self.publisher.publish(topic_path, msg, timeout=10)
        except Exception as e:
            print(f"An error occurred: {e}")

    def send_logger_data(self, msg):
        msg_array = msg.split('|')
        # Check if the length of msg_array is as expected
        if len(msg_array) != 9:
            print(f"Error: Expected 9 elements in msg_array, got {len(msg_array)}")
            return None
        # Remove "-" characters from folio
        msg_array[5] = msg_array[5].replace("-", "")
        datetimemexico = pytz.timezone("America/Mexico_City")
        msg_dictionary = {
            'voltaje_panel': float(msg_array[0]),
            'voltaje_bateria': float(msg_array[1]),
            'clave_disp': int(msg_array[2]),
            'datetime': datetime.now(datetimemexico),
            'folio': int(msg_array[5]),
            'usuario_cargando': msg_array[6].lower() in ['true', '1', 'yes'],
            'id_sesion': msg_array[7],
            'current' : msg_array[8]
        }
        return json.dumps(msg_dictionary).encode('utf-8')

    
    def send_disp_data(self, msg):
        msg_array = msg.split('|')
        if len(msg_array) != 6:
            print(f"Error: Expected 6 elements in msg_array, got {len(msg_array)}")
            return None
        msg_dictionary = {
            'numero_serie': int(msg_array[0]),
            'clave': int(msg_array[1]),
            'fecha_fabricacion':msg_array[2],
            'clave_cliente':int(msg_array[3]),
            'latitud':float(msg_array[4]),
            'longitud':float(msg_array[5]),
        }
        return json.dumps(msg_dictionary).encode('utf-8')
    
    def send_clientes_data(self, msg):
        msg_array = msg.split('|')
        if len(msg_array) != 3:
            print(f"Error: Expected 3 elements in msg_array, got {len(msg_array)}")
            return None
        msg_dictionary = {
            'clave_cliente': int(msg_array[0]),
            'nombre': msg_array[1],
            'fecha_registro':msg_array[2],
        }
        return json.dumps(msg_dictionary).encode('utf-8')


if __name__ == "__main__":

    last_msg_buffer = ""
    #class instance creation
    la_caldera_class = mqtt_google(client)

    #then there is an infinite loop for all the logger data in order to monitor the bench status
    while True:
        while msg_buffer == "":
            la_caldera_class.wait_msg()
        msg_send = la_caldera_class.send_logger_data(msg_buffer)
        msg_buffer = ""
        if msg_send:
            la_caldera_class.publish_msg(topic_buffer, msg_send)
