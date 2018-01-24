//remote control

/*Headers*/
#include <WiFiClient.h>
#include <LWiFi.h>
#include <task.h>
#include <math.h>
#include "FreeRTOS.h"

/*Pin*/
static int L_F = 2;
static int L_B = 3;
static int R_F = 4;
static int R_B = 5;
/*Wifi Settings*/
#define SSID "My ASUS"
#define PASSWD "kkkkkkkk"

#define TCP_IP_PHONE "192.168.43.1"

#define TCP_PORT 5000
WiFiClient wifiClientPh;

/*Set players*/
static char client_ID[] ="sakuramiku",Team[] = "LIH";
static char recv_buf[64],bufPh[2];


void setup() {
  // put your setup code here, to run once:
  hold();
  Serial.begin(115200);
  Serial.println("SSSSSTARTTTTT");

  //set WiFi
  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
    // Connect failed.
    // the board is retrying.
    delay(500);
    WiFi.begin(SSID, PASSWD);
    status =  WiFi.begin(SSID, PASSWD);
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(SSID);
  }
  
  Serial.print(SSID);
  Serial.println(" Connected!");
  
  while (!wifiClientPh.connect(TCP_IP_PHONE, TCP_PORT)){
    delay(300);
    Serial.print("Attempting to connect to PHONE SERVER: ");
    Serial.println(TCP_IP_PHONE);
  }
  Serial.println("Connected to phone");
  /*Clear Buffer*/
  for(int i=0;i<64;++i){
    recv_buf[i] = 0;
    bufPh[i]=0;
  }

}

void handleCommand()
{
    // Stop moving
    if (bufPh[1] == 'E') {
        hold();
        return;
    }

    switch (bufPh[0]) {
    case 'F':   // Forward
        forward();
        break;
    case 'B':   // Backward
        backward();
        break;
    case 'L':   // Turn left
        left();
        break;
    case 'R':   // Turn right
        right();
        break;
    case 'Z':   // Report ultrasonic distance and color
    //    reportUltrasonic();
    //reportColorSensor();
        break;
    }
}

void loop() {

  int phmessageLen=0;
  if ((phmessageLen = wifiClientPh.available()) > 0) {
        
        bufPh[0] = wifiClientPh.read();
        bufPh[1] = wifiClientPh.read();
        
       // Serial.println(buf);
       handleCommand();
   }

}

void forward(){
  analogWrite(L_F,250);
  analogWrite(L_B,0);
  analogWrite(R_F,255);
  analogWrite(R_B,0);
}


void backward(){
  analogWrite(L_F,0);
  analogWrite(L_B,250);
  analogWrite(R_F,0);
  analogWrite(R_B,255);
}


void left(){
  analogWrite(L_F,0);
  analogWrite(L_B,0);
  analogWrite(R_F,255);
  analogWrite(R_B,0);
}

void right(){
  analogWrite(L_F,255);
  analogWrite(L_B,0);
  analogWrite(R_F,0);
  analogWrite(R_B,0);
}

void hold(){
  analogWrite(L_F,0);
  analogWrite(L_B,0);
  analogWrite(R_F,0);
  analogWrite(R_B,0);
}
