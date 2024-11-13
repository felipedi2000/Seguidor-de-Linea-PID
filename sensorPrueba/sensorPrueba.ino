#include <QTRSensors.h>

// Configuración del emisor y número de sensores
#define EMITTER_PIN 10
#define NUM_SENSORS 8

// Inicializar el sensor QTR-8A
QTRSensors qtr;
uint16_t sensorValues[NUM_SENSORS];

void setup() {
  // Iniciar el puerto serie
  Serial.begin(9600);

  // Configurar los pines de los sensores y el emisor
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, NUM_SENSORS);
  qtr.setEmitterPin(EMITTER_PIN);

  // Calibración de los sensores esta es automatica
  Serial.println("Calibrando sensores...");
  for (int i = 0; i < 400; i++) {
    qtr.calibrate();
    delay(20);
  }
  Serial.println("Calibración completa.");
}

void loop() {
  // Leer los valores de los sensores
  uint16_t position = qtr.readLineBlack(sensorValues);

  // Mostrar los valores de cada sensor en el monitor serie
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("s");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(sensorValues[i]);
    Serial.print("\t");
  }

  // Imprimir una nueva línea
  Serial.println();

  delay(500);  // Espera 500ms entre lecturas
}

