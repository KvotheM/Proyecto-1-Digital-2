//Código recibe la señal del botón y con una interrupción envía los datos a adafruit. 
#include <Arduino.h>
#include <stdint.h>
#include <driver/adc.h>

#define boton 15
#define sensor 34
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

int envio = 0;
int mapeo, val_sensor;
int dig[3] = {0, 0, 0};
float temp = 0.0;

void IRAM_ATTR ada_ISR(void);
void temperatura(int mapeo);
void display(int valorSensor);

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile unsigned long debounce1 = 0;


void setup() {
    Serial.begin(115200);

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
}

void loop() {
  if(envio == 1){
    //lógica de envio de adafruit
    delay(10);
    val_sensor = analogRead(sensor);
    mapeo = map(val_sensor, 0, 4095, 320, 420);
    envio = 0;
    temperatura(mapeo);
    Serial.print(mapeo);
    delay(100);
    Serial.print("Hallelujah");
  }
  
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
