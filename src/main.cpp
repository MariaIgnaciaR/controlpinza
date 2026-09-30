#include <Arduino.h>
#include <Servo.h>

Servo pinza;
const int PIN_SERVO = 9;

// Ángulos calibrados
const int ANGULO_ABIERTO = 100;
const int ANGULO_CERRADO = 170;

void setup() {
  Serial.begin(9600);
  
  // En servos estándar, attach() mapea pulsos típicos de 544 a 2400 us.
  // Especificamos el rango extendido para alcanzar 195 grados con seguridad:
  pinza.attach(PIN_SERVO, 500, 2500);
  
  // Posición inicial: pinza abierta
  pinza.write(ANGULO_ABIERTO);
  Serial.println("PINZA_LISTA");
}

void loop() {
  if (Serial.available() > 0) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();

    if (comando == "OPEN") {
      pinza.write(ANGULO_ABIERTO);
      Serial.println("OK_OPEN");
    } 
    else if (comando == "CLOSE") {
      pinza.write(ANGULO_CERRADO);
      Serial.println("OK_CLOSE");
    }
  }
}