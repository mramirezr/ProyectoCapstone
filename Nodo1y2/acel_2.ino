#include <I2Cdev.h>
#include <MPU6050.h>
#include <Wire.h>
#include <EspMQTTClient.h>

// inicialización de puertos
#define SCL 15
#define SDA 14
#define SIGNAL_PIN 12
#define led_FLASH 4

EspMQTTClient client(
  "P_Capstone", // nombre de la red
  "PCapstone1", // contraseña
  "192.168.0.102",  // dirección del broker
  "",   // omitir
  "",   // omitir
  "nodo_MPU6050" // nombre del cliente
);

//Inicialización de variables
const int mpuAddress = 0x68;  //Puede ser 0x68 o 0x69
MPU6050 mpu(mpuAddress);

int16_t ax, ay, az;
int16_t gx, gy, gz;
int vibra=0;
long tiempo_prev, dt;
float girosc_ang_x, girosc_ang_y;
float girosc_ang_x_prev, girosc_ang_y_prev;
float accel_ang_x, accel_ang_y;

float acc_x, acc_y, acc_z;
float vel_x, vel_y, vel_z;
float rot_x, rot_y;
float inc_x, inc_y;

char dataString[8]; 
String salida;
int periodo = 60000;
unsigned long TiempoAhora = 0;

const float accScale = 2.0 * 9.81 / 32768.0;
const float gyroScale = 250.0 / 32768.0;

void printTab()
{
   Serial.print(F("\t"));
}

void act_info()
{

   // Se calculan métricas
   dt = millis() - tiempo_prev;
   tiempo_prev = millis();
   
   girosc_ang_x = (gx / 131)*dt / 1000.0 + girosc_ang_x_prev;
   girosc_ang_y = (gy / 131)*dt / 1000.0 + girosc_ang_y_prev;
   girosc_ang_x_prev = girosc_ang_x;
   girosc_ang_y_prev = girosc_ang_y;
   accel_ang_x = atan(ax / sqrt(pow(ay, 2) + pow(az, 2)))*(180.0 / 3.14);
   accel_ang_y = atan(ay / sqrt(pow(ax, 2) + pow(az, 2)))*(180.0 / 3.14);
   
   rot_x = girosc_ang_x;
   rot_y = girosc_ang_y;
   inc_x = accel_ang_x;
   inc_y = accel_ang_y;
   acc_x = ax * accScale;
   acc_y = ay * accScale;
   acc_z = az * accScale;
   vel_x = gx * gyroScale;
   vel_y = gy * gyroScale;
   vel_z = gz * gyroScale;

   if(!digitalRead(SIGNAL_PIN))
      vibra=1;
   else
      vibra=0;
      
   salida = String("{\"vibra\": \""+String(vibra)+
                    "\", \"ax\": \""+String(acc_x)+"\", \"ay\": \""+String(acc_y)+"\", \"az\": \""+String(acc_z)+
                    "\", \"gx\": \""+String(vel_x)+"\", \"gy\": \""+String(vel_y)+"\", \"gz\": \""+String(vel_z)+
                    "\", \"rx\": \""+String(rot_x)+"\", \"ry\": \""+String(rot_y)+
                    "\", \"ix\": \""+String(inc_x)+"\", \"iy\": \""+String(inc_y)+"\"}");     
   Serial.println(salida);
   client.publish("capstone/nodo_mpu6050",salida);
}

void onConnectionEstablished() {

  client.subscribe("capstone/nodo_actuadores", [] (const String &payload)  {
    Serial.println(payload);
  });

  client.publish("capstone/nodo_mpu6050", "Conectado");
}

void onMessageReceived(const String& topic, const String& message) {
  Serial.println(topic + ": " + message);
}

//diseño de interrupción para cambio de velocidad de muestreo
void IRAM_ATTR ISR() {
    if(!digitalRead(SIGNAL_PIN))
    {
      periodo=60000;
      digitalWrite(led_FLASH, LOW); 
    }
    else
    {
      periodo=6000;
      digitalWrite(led_FLASH, HIGH);
    } 
}

void setup()
{
   pinMode(led_FLASH, OUTPUT);
   attachInterrupt(SIGNAL_PIN, ISR, CHANGE);
   Serial.begin(115200);
   Serial.println("init serial");
   delay(5000);
   Wire.begin(SDA,SCL);
   Serial.println("init wire");
   delay(5000);
   mpu.initialize();
   Serial.println(mpu.testConnection() ? F("IMU iniciado correctamente") : F("Error al iniciar IMU"));
   delay(5000);
   
}
void loop()
{
   client.loop();
   if(millis() > TiempoAhora + periodo)
   {
        TiempoAhora = millis();
        mpu.getAcceleration(&ax, &ay, &az);
        mpu.getRotation(&gx, &gy, &gz);
        act_info();
    }
}
   
   
