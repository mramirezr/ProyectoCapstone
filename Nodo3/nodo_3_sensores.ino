// Este programa lee temperatura y humedad utilizando el sensor DHT11, también detecta la presencia
// o ausencia de gas LP utilizando el sensor MQ6 y activa o desactiva dos seromotores micro (SG90).
// Los servomotores permiten abrir o cerrar las válvulas de gas y agua, respectivamente.

// Los datos de humedad (h), temperatura (t), nivel de gas (ng), nivel de humedad (nh) y los estados
// de los dos servomotores de gas (esg) y agua (esa) se publican en un broker público con el tema
// "capstone/humedad_temperatura_niveldegas_niveldehumedad_estadoservogas_estadoservoagua".

// La terminal de salida (S) del sensor DHT11 se conecta al GPIO 14 de la ESP32CAM, la terminal de
// salida digital (DO) del sensor MQ6 se conecta al GPIO 13. La terminal de control del servomotor
// de gas (agua) se conecta al GPIO 12 (GPIO 15). NOTA: SÓLO LOS PINES GPIO 12 Y GPIO 15 ESTÁN 
// DEFINIDOS PARA USAR LA SEÑAL PWM DE LA ESP32CAM.

// Los sensores DHT11, MQ6, la ESP32CAM y los servomotores se alimentan con DOS fuentes de 5 volts.

// IMPORTANTE: Al cargar el programa a la ES32CAM, la terminal de salida (SIGNAL)
// del sensor DHT11 no debe estar conectado al ESP32CAM (DHTPIN). No olvide conectarlo
// después de cargar el programa a la ESP32CAM.

// La válvula de gas se cierra si se detecta presencia de gas LP o si la temperatura es mayor a un
// cierto umbral (por ejemplo, 50°C). La válvula de agua se cierra si se detecta un nivel de humedad
// mayor a un cierto umbral (por ejemplo, 50%). Las vávulas de gas y agua también se pueden controlar
// de forma remota. Para esto, el sistema se subscribe a un agente (broker) público en los temas
// "capstone/actuador_gas" y "capstone/actuador_agua". Si se recibe el mensaje "true" la válvula se
// abre, si se recibe el mensaje "false", la válvula se cierra.

// Incluimos librerías
#include <DHT.h>
#include <EspMQTTClient.h>
#include <ESP32Servo.h>

#define DHTPIN 14         // Definimos el pin digital donde se conecta el sensor DHT11 (SIGNAL)
#define DHTTYPE DHT11     // Tipo de sensor de temperatura (DHT11 o DHT22)
#define MQ6PIN 13         // Este pin lee el valor digital que entrega el sensor de gasLP MQ-6
#define LED_FLASH 4       // Se enciende en presencia de gas
#define SERVO_GAS_PIN 12  // Pin para controlar al servomotor que abre y cierra la válvula de GAS
#define SERVO_AGUA_PIN 15 // Pin para controlar al servomotor que abre y cierra la válvula de AGUA

int periodo = 10000;
unsigned long tiempoactual = 0;
int edo_servo_gas;         // 1 abierto, 0 cerrado
int edo_servo_agua;        // 1 abierto, 0 cerrado
int nivel_sensor_gas;      // 1 sin fuga de gas, 0 existe fuga de gas
int nivel_sensor_humedad;  // 1 humedad baja, 0 humedad alta

Servo servo_gas;
Servo servo_agua;

EspMQTTClient client(
  //"TP-Link_3714",   // WifiSSID nombre de la red
  "P_Capstone",       // Nombre de la red
  //"92574632",       // WifiPassword contraseña
  "PCapstone1",       // Contraseña
  //"52.29.114.186",  // Dirección del broker
  "192.172.0.102",    // Dirección del broker
  "",               // MQTTUsername Can be omitted if not needed
  "",               // MQTTPassword Can be omitted if not needed
  "ESP32GHV",       // Client name that uniquely identify your device
  1883              // The MQTT port, default to 1883. this line can be omitted
);

DHT dht(DHTPIN, DHTTYPE);// Inicializamos el sensor DHT11
String salida;

void onConnectionEstablished() {
  client.subscribe("capstone/actuador_gas", [] (const String & payload)
  {
    Serial.println(payload);
    if ((payload.equals("true")) && (edo_servo_gas == 0) && (nivel_sensor_gas == 1))
    {
      Serial.println("TRUE: Abre la válvula de gas");
      edo_servo_gas = 1;
      servo_gas.write(0);
    }
    if ((payload.equals("false")) && (edo_servo_gas == 1))
    {
      Serial.println("FALSE: Cierra la válvula de gas");
      edo_servo_gas = 0;
      servo_gas.write(180);
    }
    if ((edo_servo_gas == 1) && (nivel_sensor_gas == 0))
    {
      Serial.println("GAS: Cierra la válvula de gas");
      edo_servo_gas = 0;
      servo_gas.write(180);
    }
  });

  client.subscribe("capstone/actuador_agua", [] (const String & payload)
  {
    Serial.println(payload);
    if ((payload.equals("true")) && (edo_servo_agua == 0)) //&&(nivel_sensor_humedad==1))
    {
      Serial.println("TRUE: Abre la válvula de agua");
      edo_servo_agua = 1;
      servo_agua.write(0);
    }
    if ((payload.equals("false")) && (edo_servo_agua == 1))
    {
      Serial.println("FALSE: Cierra la válvula de agua");
      edo_servo_agua = 0;
      servo_agua.write(180);
    }
    if ((edo_servo_agua == 1) && (nivel_sensor_humedad == 0))
    {
      Serial.println("HUMEDAD: Cierra la válvula de agua");
      edo_servo_agua = 0;
      servo_agua.write(180);
    }
  });

  client.publish("capstone/humedad_temperatura_niveldegas_niveldehumedad_estadoservogas_estadoservoagua", "Conectado al MQTT");
}

void onMessageReceived(const String& topic, const String& message) {
  Serial.println(topic + ": " + message);
}


// Interrupción para cambiar la velocidad con la que se leen los sensores
// y para CERRAR la válvula de gas en caso de que exita fuga de gas.
void IRAM_ATTR ISR() {
  if (digitalRead(MQ6PIN)) {
    periodo = 10000; // Los sensores se leen cad 10 segundos
    digitalWrite(LED_FLASH, LOW);
  }
  else {
    periodo = 3000; // Los sensores se leen cada tres segundos
    digitalWrite(LED_FLASH, HIGH);
    if (edo_servo_gas == 1)
    {
      Serial.println("INTERRUPCIÓN PARA CERRAR la válvula de gas");
      edo_servo_gas = 0;
      servo_gas.write(180);
      //  delay(1500);
    }
  }
}

void setup() {

  pinMode(MQ6PIN, INPUT); //Se configura como pin de entrada
  pinMode(LED_FLASH, OUTPUT);
  pinMode(SERVO_GAS_PIN, OUTPUT);
  pinMode(SERVO_AGUA_PIN, OUTPUT);
  Serial.begin(115200);// Se inicializa la comunicación serial

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo_gas.setPeriodHertz(50);    // Estandard 50 Hz
  servo_agua.setPeriodHertz(50);   // Estandard 50 Hz
  servo_gas.attach(SERVO_GAS_PIN, 1000, 2000);
  servo_agua.attach(SERVO_AGUA_PIN, 1000, 2000);

  servo_gas.write(180);   edo_servo_gas = 0;
  servo_agua.write(180); edo_servo_agua = 0;
  Serial. println("Servomotores cerrados");
  delay(3000);

  servo_gas.write(0);   edo_servo_gas = 1;
  servo_agua.write(0); edo_servo_agua = 1;
  Serial. println("Servomotores abiertos");
  delay(3000);
  nivel_sensor_gas = 1;

  attachInterrupt(MQ6PIN, ISR, CHANGE);
  client.enableDebuggingMessages(); // Permite visualizar los mensajes que envía MQTT durante su ejecución
  Serial.println(F("Prueba de conexión del sensor DHT11"));
  dht.begin();// Se inicializa el sensor DHT11
}

void loop() {
  // Esperamos "periodo" segundos entre mediciones
  if (millis() > tiempoactual + periodo)
  {
    tiempoactual = millis();
    nivel_sensor_gas = digitalRead(MQ6PIN); //lee el nivel digital de gasLP
    Serial.print ("Nivel digital de gas LP: ");
    Serial.println(nivel_sensor_gas);
    client.loop();
    float h = dht.readHumidity();   // Se lee la humedad relativa (en porcentaje)
    float t = dht.readTemperature();// Se lee la temperatura en grados centígrados (por defaut)
    if (isnan(h) || isnan(t)) {     // En este "if" comprobamos si exite error en la lectura del sensor DHT11
      Serial.println("Error al realizar la lectura del sensor DHT11");
      return;
    }

    if ((t > 50.0) && (edo_servo_gas == 1))
    {
      Serial.println("TEMPERATURA ELEVADA: Cierra la válvula de gas");
      servo_gas.write(180);
      edo_servo_gas = 0;
    }

    if ((h >= 50.0) && (edo_servo_agua == 1)) // En el futuro se utilizará un sensor que detecte la presencia de agua
    {
      Serial.println("HUMEDAD ALTA: Cierra la válvula de AGUA");
      servo_agua.write(180);
      edo_servo_agua = 0;
      nivel_sensor_humedad = 0;
    }
    if ((h < 50.0)) nivel_sensor_humedad = 1;

    salida = String("{\"HUMEDAD\": \" " + String(h) + "\", \"TEMPERATURA\": \"" + String(t) + "\",\"NIVELDEGAS\": \"" + String(nivel_sensor_gas) + "\",\"NIVELDEHUMEDAD\": \"" + String(nivel_sensor_humedad) + "\",\"ESTADOSERVOGAS\": \"" + String(edo_servo_gas) + "\",\"ESTADOSERVOAGUA\": \"" + String(edo_servo_agua) + "\"}");
    Serial.println(salida);
    client.publish("capstone/humedad_temperatura_niveldegas_niveldehumedad_estadoservogas_estadoservoagua", salida);
  }
  else client.loop();
}
