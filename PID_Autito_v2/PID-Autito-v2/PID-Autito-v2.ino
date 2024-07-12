#define VELOCIDAD_MAXIMA 255
#define VELOCIDAD_BASE 150

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
const float Kp = 15;
const float Ki = 0;
const float Kd = 7;

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

  if (valorIzquierda == LOW && valorCentro == LOW && valorDerecha == LOW) bandera = 4;
  else if (valorIzquierda == LOW && valorCentro == LOW && valorDerecha == HIGH) error = 2, bandera = 3;
  else if (valorIzquierda == LOW && valorCentro == HIGH && valorDerecha == LOW) error = 0, bandera = 2;
  else if (valorIzquierda == LOW && valorCentro == HIGH && valorDerecha == HIGH) error = 1, bandera = 3; 
  else if (valorIzquierda == HIGH && valorCentro == LOW && valorDerecha == LOW) error = -2, bandera = 1; 
  else if (valorIzquierda == HIGH && valorCentro == LOW && valorDerecha == HIGH) bandera = 4; 
  else if (valorIzquierda == HIGH && valorCentro == HIGH && valorDerecha == LOW) error = -1, bandera = 1; 
  else if (valorIzquierda == HIGH && valorCentro == HIGH && valorDerecha == HIGH) bandera = 2; 


    /*
    000 - Parado 
    001 - Me refui a la der
    010 - Va para adelante
    011 - Me estoy yendo a la der
    100 - Me re fui a la iz
    101 - Dos lineas que raro
    110 - Me estoy yendo a la iz
    111 - Todo negro
    */

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
      case 4: Parar(); break;
      //else if (valorIzquierda == HIGH && valorCentro == HIGH && valorDerecha == HIGH) bandera = 5;
  }

} 

/*void GirarMotoresAdelanteRetardado(int velIzda, int velDcha)
{
  delay(1000);
  // Direccion motor A (Izquierdo)
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  analogWrite (ENA, velIzda); // Velocidad motor A
  // Direccion motor B (Derecho)
  digitalWrite (IN3, LOW);
  digitalWrite (IN4, HIGH);
  analogWrite (ENB, velDcha); // Velocidad motor B
}*/

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

