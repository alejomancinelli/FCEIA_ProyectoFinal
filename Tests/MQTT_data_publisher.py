import random
import time
from datetime import datetime
import paho.mqtt.client as mqtt

# Configuración del broker MQTT
broker = "192.168.1.22"  # Cambia a tu broker si tienes uno propio
port = 1883  # Puerto MQTT por defecto

# Configuración de los temas
topics = {
    "voltage": "BL0940/voltageRMS",
    "current": "BL0940/currentRMS",
    "power": "BL0940/activePower"
}

# Función para simular los valores de los sensores dentro de rangos normales
def generar_valores_normales():
    voltage = random.uniform(220.0, 240.0)  # Voltaje en rango doméstico
    current = random.uniform(10.0, 20.0)     # Corriente en rango doméstico
    power = voltage * current               # Potencia activa (aproximada)
    return voltage, current, power

# Función para simular valores anómalos fuera de los rangos
def generar_valores_anonimos():
    voltage = random.uniform(100.0, 300.0)  # Valores fuera de rango
    current = random.uniform(0.0, 50.0)     # Corriente fuera de rango
    power = voltage * current
    return voltage, current, power

# Función de conexión
def on_connect(client, userdata, flags, rc):
    print(f"Conectado con código de resultado {rc}")

# Inicializa el cliente MQTT
client = mqtt.Client()
client.on_connect = on_connect

# Conectar al broker
client.connect(broker, port, 60)

# Bucle principal de publicación de datos
def publicar_datos():
    contador = 0
    while True:
        # Simula valores anómalos cada 20 publicaciones
        if contador % 20 == 0 and contador != 0:
            voltage, current, power = generar_valores_anonimos()
            print("¡ALERTA! Valores anómalos generados")
        else:
            voltage, current, power = generar_valores_normales()

        energy = power
        power_factor = random.uniform(0.97, 0.99)       
       
        # Get current datetime in the format 'YYYY-MM-DD HH:MM:SS'
        current_datetime = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
       
        msg = f"{current_datetime}|{voltage:.2f}|{current:.2f}|{power:.2f}|{power:.2f}|{power_factor:.3f}"
        client.publish("energyMeter", msg)

        # Imprimir valores en consola para seguimiento
        print(f"Publicado -> Voltaje: {voltage:.2f} V, Corriente: {current:.2f} A, Potencia: {power:.2f} W")

        # Esperar 2 segundos antes de la próxima publicación
        time.sleep(2)
        contador += 1

if __name__ == "__main__":
    try:
        publicar_datos()
    except KeyboardInterrupt:
        print("Terminando la simulación.")
