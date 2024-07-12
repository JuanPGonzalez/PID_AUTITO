#define VELOCIDAD_MAXIMA 255
#define VELOCIDAD_BASE 100

// Motor A --> IZQUIERDO
const int ENA = 11;
const int IN1 = 5;
const int IN2 = 7;
// Motor B --> DERECHO
const int ENB = 6;
const int IN3 = 8;
const int IN4 = 9;
// Pines de los 3 sensores
const int sensorDerecha = A2;
const int sensorCentro = A1;
const int sensorIzquierda = A0;

// Variables de ajuste del PID
const float Kp = 20;
const float Ki = 0;
const float Kd = 0;

// Variables del PID
float P = 0, I= 0, D = 0, PID = 0;
float error = 0, errorAnterior = 0;

//Variable Bandera
int bandera = 0;

void setup()
{
  //Serial.begin(9600) ;
  // Pines del driver L298N
  pinMode (ENA, OUTPUT);
  pinMode (ENB, OUTPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);
  // Pines de los sensores
  pinMode(sensorCentro, INPUT);
  pinMode(sensorDerecha, INPUT);
  pinMode(sensorIzquierda, INPUT);
} 

void loop()
{
  int valorDerecha;
  int valorCentro;
  int valorIzquierda;
  
  valorDerecha = digitalRead(sensorDerecha);
  valorCentro = digitalRead(sensorCentro);
  valorIzquierda = digitalRead(sensorIzquierda);

  //HIGH = Negro 
  //LOW = No Negro

  if (valorIzquierda == HIGH && valorCentro == HIGH && valorDerecha == LOW) error = 2, bandera = 1;       // Robot necesita ir a la izda
  else if (valorIzquierda == HIGH && valorCentro == LOW && valorDerecha == LOW) error = 1, bandera = 1;   // Robot necesita ir a la izda
  else if (valorIzquierda == HIGH && valorCentro == LOW && valorDerecha == HIGH) error = 0, bandera = 2;  // Robot centrado
  else if (valorIzquierda == LOW && valorCentro == LOW && valorDerecha == HIGH) error = -1, bandera = 3;  // Robot necesita ir a la dcha
  else if (valorIzquierda == LOW && valorCentro == HIGH && valorDerecha == HIGH) error = -2, bandera = 3; // Robot necesita ir a la dcha
  //else if(valorIzquierda == HIGH && valorCentro == HIGH && valorDerecha == HIGH) Parar();   

  // Calculo del PID
  P = error;
  I += error;
  D = error - errorAnterior;
  PID = (Kp * P) + (Ki * I) + (Kd * D);

  // Se alamacena el error para el siguiente loop
  errorAnterior = error;

  // Se calcula la nueva velocidad
  int velocidadIzquierda = VELOCIDAD_BASE + PID;
  int velocidadDerecha = VELOCIDAD_BASE - PID;

 velocidadIzquierda = constrain(velocidadIzquierda, 0, VELOCIDAD_MAXIMA);
 velocidadDerecha = constrain(velocidadDerecha, 0, VELOCIDAD_MAXIMA);

  switch (bandera){
      case 1: GirarMotoresIzquierda(velocidadIzquierda, velocidadDerecha); break;
      case 2: GirarMotoresAdelante(velocidadIzquierda, velocidadDerecha); break;
      case 3: GirarMotoresDerecha(velocidadIzquierda, velocidadDerecha); break;
  }

} 

void GirarMotoresDerecha(int velIzda, int velDcha){
   // Direccion motor A (Izquierdo)
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  analogWrite (ENA, velIzda); // Velocidad motor A
  // Direccion motor B (Derecho)
  digitalWrite (IN3, HIGH);
  digitalWrite (IN4, LOW);
  analogWrite (ENB, velDcha); // Velocidad motor B
}

void GirarMotoresIzquierda(int velIzda, int velDcha){
   // Direccion motor A (Izquierdo)
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, HIGH);
  analogWrite (ENA, velIzda); // Velocidad motor A
  // Direccion motor B (Derecho)
  digitalWrite (IN3, LOW);
  digitalWrite (IN4, HIGH);
  analogWrite (ENB, velDcha); // Velocidad motor B
}

void GirarMotoresAdelante(int velIzda, int velDcha)
{
  // Direccion motor A (Izquierdo)
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  analogWrite (ENA, velIzda); // Velocidad motor A
  // Direccion motor B (Derecho)
  digitalWrite (IN3, LOW);
  digitalWrite (IN4, HIGH);
  analogWrite (ENB, velDcha); // Velocidad motor B
} 

void Parar()
{
  // Direccion motor A
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, LOW);
  analogWrite (ENA, 0); // Velocidad motor A
  // Direccion motor B
  digitalWrite (IN3, LOW);
  digitalWrite (IN4, LOW);
  analogWrite (ENB, 0); // Velocidad motor A
} 