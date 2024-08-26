#include <Arduino.h>
#include "config.h"
#include <stdint.h>
#include <driver/adc.h>
#include "driver/ledc.h"
#include "Adafruit_MQTT.h"

#define boton 15
#define sensor 34
//arriba
#define disp1 13
#define pinA 12
#define pinF 14
#define disp2 27
#define disp3 26
#define pinB 25
//arriba-abajo
#define pinE 33
#define pinD 32
//abajo
#define pinP 4
#define pinC 5
#define pinG 18

#define ledR 23
#define ledN 22
#define ledA 21
#define servo 19

#define PWM_R 7
#define PWM_N 8
#define PWM_A 9
#define PWM_S 10

#define freqPWM 50
#define resPWM 10

int envio = 0;
int mapeo, val_sensor;
int dig[3] = {0, 0, 0};
float temp = 0.0;

void IRAM_ATTR ada_ISR(void);
void temperatura(int mapeo);
void display(int valorSensor);

void initPWM_servo(void);
void initPWM_led_N(void);
void initPWM_led_R(void);
void initPWM_led_A(void);
void num_display(void);
void PWM(float temp);
void obtener_temp(void);
void enviar(void);

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile unsigned long debounce1 = 0;


AdafruitIO_Feed *Temperatura = io.feed("Temperatura");

void setup() {
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

  attachInterrupt(digitalPinToInterrupt(boton), ada_ISR, RISING);

  initPWM_servo();
  initPWM_led_R();
  initPWM_led_A();
  initPWM_led_N();

}

void loop() {
  io.run();
  if(envio == 1){
    delay(10);
    obtener_temp();
    enviar();
  }
  num_display();
  PWM(temp);

}
void enviar(void){
  Temperatura->save(temp);
  delay(100);
  Serial.print("Hallelujah"); 
}
void obtener_temp(void){
  val_sensor = analogRead(sensor);
  mapeo = map(val_sensor, 0, 4095, 320, 420);
  envio = 0;
  temperatura(mapeo);
  Serial.print(temp);
  delay(100);
}
void temperatura(int mapeo){
  temp = (float)mapeo/10;
  dig[0] = mapeo/100;
  dig[1] = (mapeo%100)/10;
  dig[2] = (mapeo%10);
}
void IRAM_ATTR ada_ISR(void){
  unsigned long currentTime = millis();
  if ((currentTime - debounce1) > 100) {
    portENTER_CRITICAL_ISR(&mux);
    envio = 1;
    portEXIT_CRITICAL_ISR(&mux);
    debounce1 = currentTime;
  }
}

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
