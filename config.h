//Esta librería es parte de la conexión de Adafruit, se utilizó el código base proporcionado.
//en este caso se quitó la llave de adafruit y la información de la red wifi

//se incluye la librería de adafruit para su funcionamiento
#include "Adafruit_MQTT.h"

//se indica el usuario y la llave de adafruit para establecer la conexión
#define IO_USERNAME  "nmoklebust"
#define IO_KEY       "/* colocar aquí la clave de adafruit*/"

//se da la información del wifi para que sea usado por el esp32
#define WIFI_SSID "/*colocar red a conectar*/"
#define WIFI_PASS "/*colocar contraseña*/"

//inclusión de la librería para la conexión wifi
#include "AdafruitIO_WiFi.h"

//se usa la función para establecer la conexión
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
