// Definicion de cada uno de los pines a los que se conectan los sensores infrarrojos
const int sensorDerecha = A2;
const int sensorCentro = A0;
const int sensorIzquierda = A1;

void setup()
{
  pinMode(sensorCentro, INPUT);
  pinMode(sensorDerecha, INPUT);
  pinMode(sensorIzquierda, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  int value = 0; // Variable para alamacenar el valor de la lectura
  value = digitalRead(sensorIzquierda);  //Lectura digital de pin

  if (value == HIGH)
  {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
}
