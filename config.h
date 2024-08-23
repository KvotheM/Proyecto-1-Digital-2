//Esta librería es parte de la conexión de Adafruit, se utilizó el código base proporcionado.

#include "Adafruit_MQTT.h"


#define IO_USERNAME  "nmoklebust"
#define IO_KEY       "/* colocar aquí la clave de adafruit*/"


#define WIFI_SSID "/*colocar red a conectar*/"
#define WIFI_PASS "/*colocar contraseña*/"


#include "AdafruitIO_WiFi.h"


AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
