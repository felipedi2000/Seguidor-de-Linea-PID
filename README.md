# Seguidor de linea PID

## Requerimientos de Hardware

- Placa de desarrollo Arduino Nano
- Sensor QTR-8A (8 sensores infrarrojos)
- Driver de motor TB6612FNG
- Placa de circuito para conexiones
- Motor y ruedas para el robot
- Pulsadores para inicio y calibración
- LED para indicar el estado de calibración

## Requerimientos de Software

- IDE de Arduino (para cargar el código en el Arduino)
- Controlador PID básico para ajuste automático de la dirección

## Instalación

1. **Conexión de hardware**: Conecta el sensor QTR-8A a los pines analógicos del Arduino. También conecta el driver de motor TB6612FNG a los pines digitales del Arduino para controlar los motores. Los pulsadores se conectan a los pines correspondientes de la placa Arduino.
   
2. **Instalación del software**: Abre el IDE de Arduino en tu computadora y carga el código proporcionado en el Arduino. Asegúrate de tener los controladores adecuados instalados para tu modelo de placa.

3. **Configuración**: Una vez cargado el código, conecta el robot a la fuente de energía (por ejemplo, una batería) y asegúrate de que todos los componentes estén correctamente conectados.

## Funcionamiento

1. **Calibración**: 
   - Antes de empezar a utilizar el robot, es necesario calibrar el sensor QTR-8A. Para ello, presiona el pulsador de calibración. El LED del Arduino se encenderá durante la calibración y se apagará cuando termine.
   - El sensor recorre la superficie del robot (con la línea negra) para determinar los valores de calibración adecuados.

2. **Inicio del Robot**:
   - Una vez calibrado, presiona el pulsador de inicio para poner en marcha el robot. El robot comenzará a seguir la línea negra utilizando los sensores y el control PID para ajustar su movimiento.

3. **Control PID**: 
   - El robot utiliza un algoritmo PID (Proporcional, Integral, Derivativo) para corregir su trayectoria. Cuando el robot se desvía de la línea, el algoritmo ajusta la velocidad de los motores para que el robot regrese a la trayectoria correcta.
