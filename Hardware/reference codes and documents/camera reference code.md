

### Note :i got this reference code from GitHub repository where I had used during the early development and testing phase

&nbsp;



link:[link](https://github.com/RuiSantosdotme/arduino-esp32-CameraWebServer/blob/master/CameraWebServer/CameraWebServer.ino)



\#include "esp\_camera.h"

\#include <WiFi.h>



//

// WARNING!!! Make sure that you have either selected ESP32 Wrover Module,

//            or another board which has PSRAM enabled

//



// Select camera model

//#define CAMERA\_MODEL\_WROVER\_KIT

//#define CAMERA\_MODEL\_M5STACK\_PSRAM

\#define CAMERA\_MODEL\_AI\_THINKER



const char\* ssid = "REPLACE\_WITH\_YOUR\_SSID";

const char\* password = "REPLACE\_WITH\_YOUR\_PASSWORD";





\#if defined(CAMERA\_MODEL\_WROVER\_KIT)

\#define PWDN\_GPIO\_NUM    -1

\#define RESET\_GPIO\_NUM   -1

\#define XCLK\_GPIO\_NUM    21

\#define SIOD\_GPIO\_NUM    26

\#define SIOC\_GPIO\_NUM    27



\#define Y9\_GPIO\_NUM      35

\#define Y8\_GPIO\_NUM      34

\#define Y7\_GPIO\_NUM      39

\#define Y6\_GPIO\_NUM      36

\#define Y5\_GPIO\_NUM      19

\#define Y4\_GPIO\_NUM      18

\#define Y3\_GPIO\_NUM       5

\#define Y2\_GPIO\_NUM       4

\#define VSYNC\_GPIO\_NUM   25

\#define HREF\_GPIO\_NUM    23

\#define PCLK\_GPIO\_NUM    22



\#elif defined(CAMERA\_MODEL\_M5STACK\_PSRAM)

\#define PWDN\_GPIO\_NUM     -1

\#define RESET\_GPIO\_NUM    15

\#define XCLK\_GPIO\_NUM     27

\#define SIOD\_GPIO\_NUM     25

\#define SIOC\_GPIO\_NUM     23



\#define Y9\_GPIO\_NUM       19

\#define Y8\_GPIO\_NUM       36

\#define Y7\_GPIO\_NUM       18

\#define Y6\_GPIO\_NUM       39

\#define Y5\_GPIO\_NUM        5

\#define Y4\_GPIO\_NUM       34

\#define Y3\_GPIO\_NUM       35

\#define Y2\_GPIO\_NUM       32

\#define VSYNC\_GPIO\_NUM    22

\#define HREF\_GPIO\_NUM     26

\#define PCLK\_GPIO\_NUM     21



\#elif defined(CAMERA\_MODEL\_AI\_THINKER)

\#define PWDN\_GPIO\_NUM     32

\#define RESET\_GPIO\_NUM    -1

\#define XCLK\_GPIO\_NUM      0

\#define SIOD\_GPIO\_NUM     26

\#define SIOC\_GPIO\_NUM     27



\#define Y9\_GPIO\_NUM       35

\#define Y8\_GPIO\_NUM       34

\#define Y7\_GPIO\_NUM       39

\#define Y6\_GPIO\_NUM       36

\#define Y5\_GPIO\_NUM       21

\#define Y4\_GPIO\_NUM       19

\#define Y3\_GPIO\_NUM       18

\#define Y2\_GPIO\_NUM        5

\#define VSYNC\_GPIO\_NUM    25

\#define HREF\_GPIO\_NUM     23

\#define PCLK\_GPIO\_NUM     22



\#else

\#error "Camera model not selected"

\#endif



void startCameraServer();



void setup() {

&nbsp; Serial.begin(115200);

&nbsp; Serial.setDebugOutput(true);

&nbsp; Serial.println();



&nbsp; camera\_config\_t config;

&nbsp; config.ledc\_channel = LEDC\_CHANNEL\_0;

&nbsp; config.ledc\_timer = LEDC\_TIMER\_0;

&nbsp; config.pin\_d0 = Y2\_GPIO\_NUM;

&nbsp; config.pin\_d1 = Y3\_GPIO\_NUM;

&nbsp; config.pin\_d2 = Y4\_GPIO\_NUM;

&nbsp; config.pin\_d3 = Y5\_GPIO\_NUM;

&nbsp; config.pin\_d4 = Y6\_GPIO\_NUM;

&nbsp; config.pin\_d5 = Y7\_GPIO\_NUM;

&nbsp; config.pin\_d6 = Y8\_GPIO\_NUM;

&nbsp; config.pin\_d7 = Y9\_GPIO\_NUM;

&nbsp; config.pin\_xclk = XCLK\_GPIO\_NUM;

&nbsp; config.pin\_pclk = PCLK\_GPIO\_NUM;

&nbsp; config.pin\_vsync = VSYNC\_GPIO\_NUM;

&nbsp; config.pin\_href = HREF\_GPIO\_NUM;

&nbsp; config.pin\_sccb\_sda = SIOD\_GPIO\_NUM;

&nbsp; config.pin\_sccb\_scl = SIOC\_GPIO\_NUM;

&nbsp; config.pin\_pwdn = PWDN\_GPIO\_NUM;

&nbsp; config.pin\_reset = RESET\_GPIO\_NUM;

&nbsp; config.xclk\_freq\_hz = 20000000;

&nbsp; config.pixel\_format = PIXFORMAT\_JPEG;

&nbsp; //init with high specs to pre-allocate larger buffers

&nbsp; if(psramFound()){

&nbsp;   config.frame\_size = FRAMESIZE\_UXGA;

&nbsp;   config.jpeg\_quality = 10;

&nbsp;   config.fb\_count = 2;

&nbsp; } else {

&nbsp;   config.frame\_size = FRAMESIZE\_SVGA;

&nbsp;   config.jpeg\_quality = 12;

&nbsp;   config.fb\_count = 1;

&nbsp; }



&nbsp; // camera init

&nbsp; esp\_err\_t err = esp\_camera\_init(\&config);

&nbsp; if (err != ESP\_OK) {

&nbsp;   Serial.printf("Camera init failed with error 0x%x", err);

&nbsp;   return;

&nbsp; }



&nbsp; //drop down frame size for higher initial frame rate

&nbsp; sensor\_t \* s = esp\_camera\_sensor\_get();

&nbsp; s->set\_framesize(s, FRAMESIZE\_QVGA);



&nbsp; WiFi.begin(ssid, password);



&nbsp; while (WiFi.status() != WL\_CONNECTED) {

&nbsp;   delay(500);

&nbsp;   Serial.print(".");

&nbsp; }

&nbsp; Serial.println("");

&nbsp; Serial.println("WiFi connected");

&nbsp; 

&nbsp; startCameraServer();

&nbsp; 

&nbsp; Serial.print("Camera Ready! Use 'http://");

&nbsp; Serial.print(WiFi.localIP());

&nbsp; Serial.println("' to connect");

}



void loop() {

&nbsp; // put your main code here, to run repeatedly:

&nbsp; delay(10000);

}

