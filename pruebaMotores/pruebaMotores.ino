// Define los pines del motor
const int motorAInput1 = 7;
const int motorAInput2 = 8;
const int motorAPWM = 9;

const int motorBInput1 = 5;
const int motorBInput2 = 4;
const int motorBPWM = 3;

const int STBY = 6;  // Pin de standby

// Velocidades del motor creamos un vector con 4 posibles velocidades
const int motorSpeeds[4] = {50, 100, 150, 200};  // Ejemplos de velocidades
int motorSpeedA, motorSpeedB;

void setup() {
  pinMode(motorAInput1, OUTPUT);
  pinMode(motorAInput2, OUTPUT);
  pinMode(motorAPWM, OUTPUT);

  pinMode(motorBInput1, OUTPUT);
  pinMode(motorBInput2, OUTPUT);
  pinMode(motorBPWM, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);  // Habilita el driver de motor

  // Inicializa los motores para avanzar
  digitalWrite(motorAInput1, LOW);
  digitalWrite(motorAInput2, HIGH);
  digitalWrite(motorBInput1, LOW);
  digitalWrite(motorBInput2, HIGH);
}

void loop() {
  // Aumentar la velocidad en 4 pasos
  for (int i = 0; i < 4; i++) {
    motorSpeedA = motorSpeeds[i];
    motorSpeedB = motorSpeeds[i];
    analogWrite(motorAPWM, motorSpeedA);
    analogWrite(motorBPWM, motorSpeedB);
    delay(3000); // Esperar 3 segundos en cada paso
  }

  // Disminuir la velocidad en 4 pasos
  for (int i = 3; i >= 0; i--) {
    motorSpeedA = motorSpeeds[i];
    motorSpeedB = motorSpeeds[i];
    analogWrite(motorAPWM, motorSpeedA);
    analogWrite(motorBPWM, motorSpeedB);
    delay(3000); // Esperar 3 segundos en cada paso
  }
}

