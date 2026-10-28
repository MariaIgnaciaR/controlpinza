#include <Arduino.h>
#include <Servo.h>   // Viene incluida con el framework Arduino: no hace falta lib_deps

// Pinza controlada por comandos seriales desde Python.
// Comando: "G<angulo>\n"  (ej: G60 abre, G120 cierra). Angulo 0-180.

Servo pinza;
const int PIN_PINZA = 9;          // Pin de señal (PWM) del servo
const int ANGULO_INICIAL = 60;    // Posición al encender (pinza abierta)

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);          // por defecto espera 1000 ms y trababa el loop
  pinza.attach(PIN_PINZA);
  pinza.write(ANGULO_INICIAL);
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 1 && cmd.charAt(0) == 'G') {
      int angulo = constrain(cmd.substring(1).toInt(), 0, 180);
      pinza.write(angulo);
      Serial.print("OK ");
      Serial.println(angulo);
    }
  }
}
