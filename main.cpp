/*
Universidad del Valle de Guatemala
Proyecto 1, Digital 2 
Nicolás Moklebust, 22037
El proyecto tiene como función principal recibir una señal analógica de un sensor de temperatura. La señal se traduce en temperatura y se la muestra
en un display de 7 segmentos. Además, dependiendo de dónde se encuentra la temperatura se encienden leds y se mueve un potenciómetro para indicar si
hay fiebre o la temperatura es normal. Finalmente, la temperatura leída se envía a Adafruit.
Micro: ESP32 DEV Kit 1.0
*/

//Se incluyen las librerías necesarias para el funcionamiento del proyecto.
#include <Arduino.h>
#include "config.h"
#include <stdint.h>
#include <driver/adc.h>
#include "driver/ledc.h"
#include "Adafruit_MQTT.h"

//definimos los pines del botón y del sensor de temperatura
#define boton 15
#define sensor 34
//definimos los pines del display de 7 segmentos
#define disp1 13
#define pinA 12
#define pinF 14
#define disp2 27
#define disp3 26
#define pinB 25
#define pinE 33
#define pinD 32
#define pinP 4
#define pinC 5
#define pinG 18

//definimos los pines de los leds y del servo (de acá saldrán los PWM)
#define ledR 23
#define ledN 22
#define ledA 21
#define servo 19

//definimos los pines en los que colocaremos los PWM
#define PWM_R 7
#define PWM_N 8
#define PWM_A 9
#define PWM_S 10

//definimos frecuencia para el PWM y la resolución que lo necesita
#define freqPWM 50
#define resPWM 10

//definimos las variables que usaremos para obtener y convertir la temperatura. 
int envio = 0;
int val_sensor;
float mapeo;
int dig[3] = {0, 0, 0};
float temp = 0.0;

//Mandamos a llamar la interrupción del botón
void IRAM_ATTR ada_ISR(void);

//Mandamos a llamar las funciones de obtención, conversión, envío y display de temperatura. 
void temperatura(float mapeo);
void display(int valorSensor);
void obtener_temp(void);
void enviar(void);
void num_display(void);

//mandamos a llamar las funciones de los PWM
void initPWM_servo(void);
void initPWM_led_N(void);
void initPWM_led_R(void);
void initPWM_led_A(void);
void PWM(float temp);

//Definimos el mux para la interrupción y el tiempo de debounce
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile unsigned long debounce1 = 0;

//Asignamos el feed de Ada al que mandaremos los datos
AdafruitIO_Feed *Temperatura = io.feed("Temperatura");

void setup() {
  //Iniciamos el serial y colocamos la lógica de conexión de ada, haciendo uso de un ejemplo de publish
  //mandamos un mensaje de conexión para indicar al usuario lo que se está haciendo
  Serial.begin(115200);
  while(! Serial);
  Serial.print("Connecting to Adafruit IO");
  io.connect();
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.println(io.statusText());

  //Indicamos el modo de funcionamiento de los pines (OUTPUT e INPUT)
  pinMode(boton, INPUT_PULLDOWN);
  pinMode(sensor, INPUT);
  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  pinMode(pinC, OUTPUT);
  pinMode(pinD, OUTPUT);
  pinMode(pinE, OUTPUT);
  pinMode(pinF, OUTPUT);
  pinMode(pinG, OUTPUT);
  pinMode(pinP, OUTPUT);
  pinMode(disp1, OUTPUT);
  pinMode(disp2, OUTPUT);
  pinMode(disp3, OUTPUT);
  
//Anexamos la interrupción al botón e indicamos que debe de suceder cuando este se presione
  attachInterrupt(digitalPinToInterrupt(boton), ada_ISR, RISING);
//Inicialos las funciones de PWM
  initPWM_servo();
  initPWM_led_R();
  initPWM_led_A();
  initPWM_led_N();
}

void loop() {
  //iniciamos la conexión del io
  io.run();
  //este loop espera a que la interrupción cambie el valor de la variable envío para obtener el valor de temperatura
  //y enviarlo a Adafruit haciendo uso de dos funciones definidas posteriormente en el código
  if(envio == 1){
    obtener_temp();
    enviar();
  }
  //Estas funciones (definidas abajo) muestran la temperatura en el display de 7 segmentos
  //y activan los leds y el movimiento del servo
  num_display();
  PWM(temp);

}
//esta función simplemente envia el dato de temperatura al feed de adafruit
//esta aparecerá en nuestro dashboard
void enviar(void){
  Temperatura->save(temp);
}
//la función lee el el valor que el sensor tiene y mediante una ecuación obtenida en https://fjrg76.wordpress.com/2021/02/28/lm35-y-su-formula/
//lo transforma para poder usarlo en la función temperatura (abajo está definida). 
void obtener_temp(void){
  val_sensor = analogRead(sensor);
  mapeo = ((float)val_sensor*330)/4095;
  envio = 0;
  temperatura(mapeo);
}
//esta función toma el valor de la variable mapeo y obtiene separado los dígitos para mostrarlos en el display. 
//Los dígitos obtenidos los mete en un array. 
void temperatura(float mapeo){
  temp = mapeo;
  mapeo = mapeo*10;
  dig[0] = mapeo/100;
  dig[1] = ((int)mapeo%100)/10;
  dig[2] = ((int)mapeo%10);
}

//Se define la interrupción la cual tiene un debounce colocado y cambia el valor de la variable envio para activar el ciclo if del "void loop"
void IRAM_ATTR ada_ISR(void){
  unsigned long currentTime = millis();
  if ((currentTime - debounce1) > 100) {
    portENTER_CRITICAL_ISR(&mux);
    envio = 1;
    portEXIT_CRITICAL_ISR(&mux);
    debounce1 = currentTime;
  }
}

//esta función usa un switch case para encender los diferenes segmentos de un display (de cuatro bits) leyendo los dígitos obtenidos mediante la función
//temperatura
void display(int valorSensor){
  switch (valorSensor)
  {
  case 0:
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, HIGH);
    digitalWrite(pinC, HIGH);
    digitalWrite(pinD, HIGH);
    digitalWrite(pinE, HIGH);
    digitalWrite(pinF, HIGH);
    digitalWrite(pinG, LOW);
    break;
  case 1:
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
    digitalWrite(pinC, HIGH);
    digitalWrite(pinD, LOW);
    digitalWrite(pinE, LOW);
    digitalWrite(pinF, LOW);
    digitalWrite(pinG, LOW);
    break;
  case 2:
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, HIGH);
    digitalWrite(pinC, LOW);
    digitalWrite(pinD, HIGH);
    digitalWrite(pinE, HIGH);
    digitalWrite(pinF, LOW);
    digitalWrite(pinG, HIGH);
    break;
  case 3:
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, HIGH);
    digitalWrite(pinC, HIGH);
    digitalWrite(pinD, HIGH);
    digitalWrite(pinE, LOW);
    digitalWrite(pinF, LOW);
    digitalWrite(pinG, HIGH);
    break;
  case 4:
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
    digitalWrite(pinC, HIGH);
    digitalWrite(pinD, LOW);
    digitalWrite(pinE, LOW);
    digitalWrite(pinF, HIGH);
    digitalWrite(pinG, HIGH);
    break;
  case 5:
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
    digitalWrite(pinC, HIGH);
    digitalWrite(pinD, HIGH);
    digitalWrite(pinE, LOW);
    digitalWrite(pinF, HIGH);
    digitalWrite(pinG, HIGH);
    break;
  case 6:
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
    digitalWrite(pinC, HIGH);
    digitalWrite(pinD, HIGH);
    digitalWrite(pinE, HIGH);
    digitalWrite(pinF, HIGH);
    digitalWrite(pinG, HIGH);
    break;
  case 7:
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, HIGH);
    digitalWrite(pinC, HIGH);
    digitalWrite(pinD, LOW);
    digitalWrite(pinE, LOW);
    digitalWrite(pinF, LOW);
    digitalWrite(pinG, LOW);
    break;
  case 8:
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, HIGH);
    digitalWrite(pinC, HIGH);
    digitalWrite(pinD, HIGH);
    digitalWrite(pinE, HIGH);
    digitalWrite(pinF, HIGH);
    digitalWrite(pinG, HIGH);
    break;
  case 9:
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, HIGH);
    digitalWrite(pinC, HIGH);
    digitalWrite(pinD, HIGH);
    digitalWrite(pinE, LOW);
    digitalWrite(pinF, HIGH);
    digitalWrite(pinG, HIGH);
    break;
  }
}
//esta función se encarga de encender los dígitos correctos del display para mostrar la temperatura obtenida por el sensor
void num_display(void){
  digitalWrite(disp1, HIGH);
  digitalWrite(disp2, LOW);
  digitalWrite(disp3, LOW);
  digitalWrite(pinP, LOW);
  display(dig[0]);
  delay(5);

  digitalWrite(disp1, LOW);
  digitalWrite(disp2, HIGH);
  digitalWrite(disp3, LOW);
  digitalWrite(pinP, HIGH);
  display(dig[1]);
  delay(5);

  digitalWrite(disp1, LOW);
  digitalWrite(disp2, LOW);
  digitalWrite(disp3, HIGH);
  digitalWrite(pinP, LOW);
  display(dig[2]);
  delay(5);
}
//esta función usa el valor de la variable temp (temperatura medida) y mediante if's anidados se encarga de encender o apagar diferentes leds
//y mueve un servo para indicar en que rango de temperatura se encuentra
void PWM(float temp){
  if(temp <= 37 & temp != 0){
    ledcWrite(PWM_S, 95);
    ledcWrite(PWM_A, 128);
    ledcWrite(PWM_R, 0);
    ledcWrite(PWM_N, 0);
  }
  else if(temp > 37 & temp <= 37.5){
    ledcWrite(PWM_S, 70);
    ledcWrite(PWM_N, 128);
    ledcWrite(PWM_R, 0);
    ledcWrite(PWM_A, 0);
  }
  else if(temp > 37.5){
    ledcWrite(PWM_S, 51);
    ledcWrite(PWM_R, 128);
    ledcWrite(PWM_A, 0);
    ledcWrite(PWM_N, 0);
  }
}

//las siguientes cuatro funciones son la definición de los PWM y la asignación de los pines que se usaran para los leds y servos.
//hacen uso de la resolución y frecuencia definidas previamente.
void initPWM_servo(void) {
  ledcSetup(PWM_S, freqPWM, resPWM);
  ledcAttachPin(servo, PWM_S);
  ledcWrite(PWM_S, 123);
}
void initPWM_led_R(void) {
  ledcSetup(PWM_R, freqPWM, resPWM);
  ledcAttachPin(ledR, PWM_R);
  ledcWrite(PWM_R, 0);
}
void initPWM_led_N(void) {
  ledcSetup(PWM_N, freqPWM, resPWM);
  ledcAttachPin(ledN, PWM_N);
  ledcWrite(PWM_N, 0);
}
void initPWM_led_A(void) {
  ledcSetup(PWM_A, freqPWM, resPWM);
  ledcAttachPin(ledA, PWM_A);
  ledcWrite(PWM_A, 0);
}
