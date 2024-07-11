// Definición de pines para sensores y motores
const int sensorLeft = A0;  // Sensor infrarrojo izquierdo
const int sensorCenter = A1; // Sensor infrarrojo central
const int sensorRight = A2;  // Sensor infrarrojo derecho

const int motorLeftForward = 12;  // Pin del motor izquierdo hacia adelante
const int motorLeftBackward = 13; // Pin del motor izquierdo hacia atrás
const int motorLeftEnable = 11;   // Pin de habilitación del motor izquierdo (PWM)

const int motorRightForward = 8;  // Pin del motor derecho hacia adelante
const int motorRightBackward = 9; // Pin del motor derecho hacia atrás
const int motorRightEnable = 6;   // Pin de habilitación del motor derecho (PWM)

// Variables para el controlador PID
double kp = 2.0;  // Ganancia proporcional
double ki = 0.0;  // Ganancia integral
double kd = 1.0;  // Ganancia derivativa

double lastError = 0.0;
double integral = 0.0;

void setup() {
  // Configurar los pines de los motores como salidas
  pinMode(motorLeftForward, OUTPUT);
  pinMode(motorLeftBackward, OUTPUT);
  pinMode(motorLeftEnable, OUTPUT);

  pinMode(motorRightForward, OUTPUT);
  pinMode(motorRightBackward, OUTPUT);
  pinMode(motorRightEnable, OUTPUT);

  // Configurar los pines de los sensores como entradas
  pinMode(sensorLeft, INPUT);
  pinMode(sensorCenter, INPUT);
  pinMode(sensorRight, INPUT);
}

void loop() {
  // Leer valores de los sensores
  int leftValue = analogRead(sensorLeft);
  int centerValue = analogRead(sensorCenter);
  int rightValue = analogRead(sensorRight);

  // Calcular el error
  double error = (leftValue - rightValue);

  // Controlador PID
  integral += error;
  double derivative = error - lastError;
  double control = kp * error + ki * integral + kd * derivative;

  // Actualizar el último error
  lastError = error;

  // Calcular velocidades de los motores
  int baseSpeed = 75;  // Velocidad base de los motores
  int leftSpeed = baseSpeed + control;
  int rightSpeed = baseSpeed - control;

  // Limitar las velocidades a un rango de 0 a 255
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  //Sigue linea negra
  if(centerValue == 0){
      digitalWrite(motorLeftForward, HIGH);
      digitalWrite(motorLeftBackward, LOW);
      digitalWrite(motorRightForward, HIGH);
      digitalWrite(motorRightBackward, LOW);
      analogWrite(motorLeftEnable, baseSpeed);
      analogWrite(motorRightEnable, baseSpeed);
  }

  /*
  // Controlar el motor izquierdo
  if (leftSpeed > 0) {
      digitalWrite(motorLeftForward, HIGH);
      digitalWrite(motorLeftBackward, LOW);
      analogWrite(motorLeftEnable, leftSpeed);
  } else {
      digitalWrite(motorLeftForward, LOW);
      digitalWrite(motorLeftBackward, HIGH);
      analogWrite(motorLeftEnable, -leftSpeed);
  }

    // Controlar el motor derecho
  if (rightSpeed > 0) {
      digitalWrite(motorRightForward, HIGH);
      digitalWrite(motorRightBackward, LOW);
      analogWrite(motorRightEnable, rightSpeed);
  } else {
      digitalWrite(motorRightForward, LOW);
      digitalWrite(motorRightBackward, HIGH);
      analogWrite(motorRightEnable, -rightSpeed);
  }
  */

  // Pequeño retraso para permitir el tiempo de respuesta del sistema
  delay(10);
}
