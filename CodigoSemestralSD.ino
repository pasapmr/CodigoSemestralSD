#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <BluetoothSerial.h>
#include <Adafruit_PWMServoDriver.h>
#include <ESP32Servo.h>

// Configuración de la pantalla LCD con dirección I2C 0x27
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Configuración del sensor ultrasónico
#define TRIG_PIN 16
#define ECHO_PIN 5

// Configuración del sensor DHT11
#define DHTPIN 19
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Configuración del LED y Buzzer
int ledPin = 15;     // Pin del LED para indicar proximidad
int buzzerPin = 4;   // Pin del buzzer para indicar proximidad

// Configuración del PCA9685 con dirección I2C 0x40 (predeterminada)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // Valor mínimo del pulso para el servo (aprox. 0.5 ms)
#define SERVOMAX  600 // Valor máximo del pulso para el servo (aprox. 2.5 ms)
int servoUbi;        // Variable para la ubicación del servo
float angulo;        // Variable para el ángulo del servo

// Configuración del Bluetooth
BluetoothSerial SerialBT;
Servo servo0;        // Objeto servo para controlar un servo motor

void setup() {
  // Inicialización de la comunicación serial
  Serial.begin(115200);
  SerialBT.begin("ESP32_BT");  // Inicialización del Bluetooth con nombre "ESP32_BT"

  // Inicialización de la pantalla LCD
  lcd.init();
  lcd.backlight();

  // Configuración de los pines del sensor ultrasónico
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Inicialización del servo en el pin 12
  servo0.attach(12);

  // Inicialización del sensor DHT11
  dht.begin();

  // Configuración de los pines del LED y buzzer
  pinMode(ledPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  // Inicialización del PCA9685 (controlador PWM)
  Serial.println("Inicializando PCA9685");
  pwm.begin();
  pwm.setPWMFreq(60);  // Frecuencia de PWM de 60 Hz para los servos

  delay(10);
}

void ServoAngconfig(uint8_t servoUbi, uint16_t angulo) {
  // Mapea el ángulo deseado al rango de pulsos del servo
  uint16_t pulseLength = map(angulo, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servoUbi, 0, pulseLength);  // Configura el servo en la posición deseada
}

void loop() {
  // Lectura del sensor ultrasónico
  long duracion;
  int distancia;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duracion = pulseIn(ECHO_PIN, HIGH);
  distancia = (duracion / 2.0) / 29.1;  // Calcula la distancia en centímetros

  // Lectura del sensor DHT11
  float h = dht.readHumidity();       // Lee la humedad
  float t = dht.readTemperature();    // Lee la temperatura

  // Verificar si la lectura del DHT11 falló
  if (isnan(h) || isnan(t)) {
    Serial.println("Error al leer el sensor DHT11");
  }

  // Envío de datos por Bluetooth: temperatura y humedad
  SerialBT.print(t);
  SerialBT.print(";");
  SerialBT.print(h);
  SerialBT.println(";");

  // Convertir los valores de temperatura y humedad a enteros
  int h_int = (int)h;
  int t_int = (int)t;

  // Mostrar los valores en la pantalla LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(" Dist: ");
  lcd.print(distancia);
  lcd.print(" cm");

  lcd.setCursor(0, 1);
  lcd.print("Tem:");
  lcd.print(t_int);
  lcd.print("C Hum:");
  lcd.print(h_int);
  lcd.print("%");

  // Activación del LED y buzzer si la distancia es menor a 10 cm
  if (distancia < 10) {
    digitalWrite(ledPin, HIGH);    // Enciende el LED
    digitalWrite(buzzerPin, HIGH); // Enciende el buzzer
  } else {
    digitalWrite(ledPin, LOW);     // Apaga el LED
    digitalWrite(buzzerPin, LOW);  // Apaga el buzzer
  }

  // Verificar si hay datos disponibles en el Bluetooth
  if (SerialBT.available()) {
    String bt = SerialBT.readString();
    Serial.println("Comando recibido: " + bt);
    delay(500);
      
    // Verificar el tipo de comando recibido por Bluetooth
    if (bt.startsWith("t0")) {
      String anguloStr = bt.substring(2);  // Extrae el ángulo del comando
      int angulo = anguloStr.toInt();      // Convierte el ángulo a entero

      // Mover el servo0 según el ángulo recibido
      if (angulo == 0) {
        servo0.write(0);  // Mueve el servo a 0 grados (sentido horario)
        Serial.println("Servo movido a 0 grados (sentido horario)");
      } else if (angulo == 180) {
        servo0.write(180);  // Mueve el servo a 180 grados (sentido antihorario)
        Serial.println("Servo movido a 180 grados (sentido antihorario)");
      } else if (angulo == 90) {
        servo0.write(90);  // Mueve el servo a 90 grados (parada)
        Serial.println("Servo movido a 90 grados (parada)");
      } else {
        Serial.println("Ángulo inválido, solo se acepta 0, 90 o 180 grados.");
      }
    }

    // Verificar comandos para controlar otros servos
    else if (bt.startsWith("s")) {
      // Extrae el número de servo y el ángulo del comando
      int servoUbi = bt.substring(1, 2).toInt();
      int angulo = bt.substring(2).toInt();

      // Verifica que el número de servo esté en el rango válido de ángulo
      if (servoUbi >= 0 && servoUbi < 16 && angulo >= 0 && angulo <= 180) {
        ServoAngconfig(servoUbi, angulo);  // Configura el ángulo del servo deseado
        if (servoUbi == 6) {
          ServoAngconfig(7, angulo);  // Configura también el servo 7 para cierta condición
          Serial.println(angulo);
        } else {
          Serial.println("Ángulo configurado");
        }
      } else {
        Serial.println("Comando de servo inválido");
      }
    } else {
      Serial.println("Formato de comando Bluetooth incorrecto");
    }
  }

  // Retardo antes de la siguiente lectura
  delay(500);
}
