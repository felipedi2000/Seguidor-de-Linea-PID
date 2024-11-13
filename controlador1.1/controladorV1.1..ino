/*
  Pgromada encargado de controlar la posicion de un robot seguidor de linea
  mediante un control de posicion PID.

  **Requirimentos de hardaware**
    --Placa de desarrollor arduino NANO
    --Sensor QTR8A
    --Driver TB6612FNG

  **Reuirimientos de software**
  --IDE para arduino, windows.

  **Intalcion**
  --Concete la placa ordenador y abra el ide
  --Cargue el codigo y ejecute pruebas

  **Funcionamiento**
  --Encienda el robot mediante el boton deslisante y oprima e
  l pulasdor de calibracion(realice movimiento del sensor por toda la superficie incluyendo al linea negra),
    se encendara el led rojo del arduino cuando se apague dicho led, la calibracion habra terminado.
  --Arranque el carro pulsado el boton de arranque.

  **Version del firmware 1.1**
*/

#include <QTRSensors.h>                       // libreria del sensor

// Definir pines para el driver de motores TB6612FNG
#define motorAInput1 7
#define motorAInput2 8
#define motorBInput1 5
#define motorBInput2 4
#define motorAPWM 9
#define motorBPWM 3
#define STBY      6                           // Pin STBY del TB6612FNG para habilitarlo


// Definir pines para el sensor QTR-8A
#define NUM_SENSORS 8
#define NUM_SAMPLES_PER_SENSOR 4
#define EMITTER_PIN 10

// Constantes del control PID
#define KP 4.6                                // Constante proporcional
#define KD 19.4                               // Constante derivativa 
#define KI 0.009                              // Constante integral 
#define motorSpeed 28                         //Ajuste este valor para controlar la velocidad de avance 
#define maxMotorSpeed 35                      // maxima velocidad de los motores  el ranfo es de  a 255

// Pines para los pulsadores para iniciar la calibraicon y para arrancar el carro
#define buttoncalibrate 2  
#define beginProgram 12

// variablees boleanas para controlar el estado de los pulsadoses
boolean buttonPressed = false;               // Variable para saber si el programa debe ejecutarse
int lastButtonState = LOW;                   // Estado anterior del botón
int buttonState = LOW; 


// Inicialzacion del sensor
QTRSensors qtr;

unsigned int sensorValues[NUM_SENSORS];

// Vatiables usadas para el control PID
int lastError = 0;
int integral = 0;

void setup() {
// Establecer pines de salida para el driver TB6612FNG
  pinMode(motorAInput1, OUTPUT);
  pinMode(motorAInput2, OUTPUT);
  pinMode(motorBInput1, OUTPUT);
  pinMode(motorBInput2, OUTPUT);
  pinMode(motorAPWM, OUTPUT);
  pinMode(motorBPWM, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);

  // Inicializa el sensor QTR-8A 
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, NUM_SENSORS);
  qtr.setEmitterPin(EMITTER_PIN);
  
  Serial.begin(9600);// inicio de comunicacion serial

  // Calibracion del sensro QTR8A hasta que no se calibra el programa no continua
    boolean Ok = false;
    while (Ok == false) { 
    if(digitalRead(buttoncalibrate) == HIGH) {
      digitalWrite(LED_BUILTIN, HIGH); // enciende LED de Arduino para indicar el inicio de la calibracion
      Serial.println("calibrando");
      calibration();                //llama al programa de calibracion del sensor
      Ok = true;
      digitalWrite(LED_BUILTIN, LOW); //apaga el led Arduino para indicar termino de calibracion
    }
  }
}


//==================programa que calibra el sensor===================
void calibration() {
  for (uint16_t i = 0; i < 400; i++){
    qtr.calibrate();
  }
  Serial.println("calibrado.....");
}
//===================================================================================


//================================Pgrama encargado de controlar la posicion del robot
void computate(){
  
  uint16_t position = qtr.readLineBlack(sensorValues);                // lee el sensor 

  int error = (NUM_SENSORS - 1) * 1000 / 2 - position;                // calcula el error del sensor con respecto a la posicion del centro

  error = constrain(error, -500, 500);                                // limitamos el error en un rango para evitar perdidas del robot en la pista

  int proportional = KP * error;                                      // Calculo componente P del controlador 
  int derivative = KD * (error - lastError);                          // Calculo componente D del controlador 
  integral += error;                                                  // se acumula el error
  int integralComponent = KI * integral;                              //Calculo componente I del controlador 

  int controlSignal = proportional + derivative + integralComponent;  // Calcular señal de control que es la suma de las 3 componentes

  controlSignal = constrain(controlSignal, -100, 100);                // limitar la señal de control en un rango de -100 a 100 para evitar respuestas lentas

  int motorSpeedA = motorSpeed - controlSignal;                       // corregir la velocidad de los motores, usando el concepto de diferencial
  int motorSpeedB = motorSpeed + controlSignal;                       // motorSpeed es la velocidad base a la que va el robor cuando esta en la linea
                                                                      // cuando este se sale usa la valocidad maxima o maxMotorSpeed

  motorSpeedA = constrain(motorSpeedA, 0, maxMotorSpeed);             // se limitan las velocidades de los motores
  motorSpeedB = constrain(motorSpeedB, 0, maxMotorSpeed);

  digitalWrite(motorBInput1, LOW);                                    // finalmente avanza el carro a las velocidades calculadas
  digitalWrite(motorBInput2, HIGH);
  digitalWrite(motorAInput1, LOW);//LOW para avanzar hacia adelante
  digitalWrite(motorAInput2, HIGH);
  analogWrite(motorAPWM, motorSpeedA);
  analogWrite(motorBPWM, motorSpeedB);

  lastError = error;                                                  //Se  guarda el error anterior

  // aqui se muestran las medidas de los sensores en el monitor serie
  Serial.print("s1: ");
  Serial.print(sensorValues[0]);
  Serial.print("\ts2: ");
  Serial.print(sensorValues[1]);
  Serial.print("\ts3: ");
  Serial.print(sensorValues[2]);
  Serial.print("\ts4: ");
  Serial.print(sensorValues[3]);
  Serial.print("\ts5: ");
  Serial.print(sensorValues[4]);
  Serial.print("\ts6: ");
  Serial.print(sensorValues[5]);
  Serial.print("\ts7: ");
  Serial.print(sensorValues[6]);
  Serial.print("\ts8: ");
  Serial.println(sensorValues[7]);
  // se muestra el erro, errores negativos seignifica que el robot esta a la izquierda del centro de la lina negra, errorers positivos signinifca que esta a la derecha
  Serial.print("Error: ");
  Serial.println(error);
  delay(10);              // pequeño retardo para que el root reaccione
}
//=============================================================================================================================


void loop() {
  //=============================== PROGRAMA QUE CONTROLALA LOGICA DEL PULSADOR DE INICIO======
  buttonState = digitalRead(beginProgram);
  if (buttonState == LOW && lastButtonState == HIGH) {
    buttonPressed = !buttonPressed; // Cambia el estado para iniciar o detener el programa
    delay(50); // Pequeño delay para evitar rebotes (debouncing)
  }

  lastButtonState = buttonState; // Actualizar el estado anterior del botón

  // Ejecutar el código cuando se presiona el botón
  if (buttonPressed) {
    computate(); // Si el pulsador de incio fue oprimido llama al programa que controla al robot
  }
  //===========================================================================================
}
