#include <WiFiClient.h>
#include <LWiFi.h>
#include <task.h>
#include <math.h>
#include <stdio.h>
#define TCP_PORT 5000

#define SSID "2scream"
#define PASSWD "2017scream"
#define TCP_IP "192.168.0.100"
#define TCP_IP_PHONE "192.168.0.131"

xTaskHandle xHandle;

//WiFiClient wifiClientPh;
static char buf[100], bufPh[100], buf_send[100], buf_phsend[100];
static char client_ID[] = "sakuramiku", Team[] = "B";
static int messageLen, phmessageLen;
static int MyPosX, MyPosY, DstPosX = -1, DstPosY = -1;
static int treasure1x, treasure1y, treasure2x, treasure2y, treasure3x, treasure3y, treasure4x, treasure4y;
static int mytreasurex = -1, mytreasurey = -1;
String teammate;
static int pastx = -1, pasty = -1;
static double base, now;
static int start, getgoal = 0, stepfinish = 0;
const double pi = 3.14159265359;

WiFiClient wifiClient;
static char *recv_ID, *recv_buf;

enum MotorPinID {
  L_F = 0,
  L_B,
  R_F,
  R_B,
  NUM_OF_MOTOR_PIN
};
enum UltrasonicPinID {
  U_F = 0,
  U_L,
  U_R,
  NUM_OF_ULTRASONIC_PIN
};
static const uint8_t usTrigPins[NUM_OF_ULTRASONIC_PIN] = {2, 4, 9 };  // F, L, R
static const uint8_t usEchoPins[NUM_OF_ULTRASONIC_PIN] = {3, 5, 10 };  // F, L, R
static const uint8_t motorPins[NUM_OF_MOTOR_PIN] = {14, 15, 16, 17};  //  L_F, L_B,R_F, R_B
bool letgo = false;

long ultrasonicGetDistance(uint8_t trig, uint8_t echo)
{
  long duration;
  vTaskSuspend( xHandle );
  pinMode(trig, OUTPUT);
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(5);
  digitalWrite(trig, LOW);
  pinMode(echo, INPUT);
  duration = pulseIn(echo, HIGH, 5000000L);
  vTaskResume( xHandle );
  return duration / 29 / 2;
}
void forward(int t)
{
  analogWrite(motorPins[L_F], 152);
  analogWrite(motorPins[L_B], 0);
  analogWrite(motorPins[R_F], 150);
  analogWrite(motorPins[R_B], 0);
}
void forward2(int t)
{
  analogWrite(motorPins[L_F], 165);
  analogWrite(motorPins[L_B], 0);
  analogWrite(motorPins[R_F], 150);
  analogWrite(motorPins[R_B], 0);
}
void backward(int t)
{
  analogWrite(motorPins[L_F], 0);
  analogWrite(motorPins[L_B], 100);
  analogWrite(motorPins[R_F], 0);
  analogWrite(motorPins[R_B], 100);
}
void left(int t)
{

  analogWrite(motorPins[L_F], 0);
  analogWrite(motorPins[L_B], 150);
  analogWrite(motorPins[R_F], 150);
  analogWrite(motorPins[R_B], 0);
}
void right(int t)
{
  analogWrite(motorPins[L_F], 150);
  analogWrite(motorPins[L_B], 0);
  analogWrite(motorPins[R_F], 0);
  analogWrite(motorPins[R_B], 150);
}
void nomove(int t) {
  analogWrite(motorPins[L_F], 0);
  analogWrite(motorPins[L_B], 0);
  analogWrite(motorPins[R_F], 0);
  analogWrite(motorPins[R_B], 0);
}

IPAddress ip;


void setup()
{
  int motorpins = 0;
  while (motorpins < NUM_OF_MOTOR_PIN) {
    pinMode(motorPins[motorpins], OUTPUT);
    motorpins++;
  }
  int status = WL_IDLE_STATUS;
  Serial.begin(115200);
  while (!Serial)
    ;



  // set WiFi
  // WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWD);
  while (status != WL_CONNECTED) {
    // Connect failed, blink 0.5 second to indicate
    // the board is retrying.
    delay(500);
    WiFi.begin(SSID, PASSWD);
    status =  WiFi.begin(SSID, PASSWD);
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(SSID);
    Serial.println(status);
  }

  // Conenct to AP successfully
  // wifiClient.connect(TCP_IP, TCP_PORT);
  while (!wifiClient.connect(TCP_IP, TCP_PORT)) {
    delay(300);
    Serial.print("Attempting to connect to SERVER: ");
    Serial.println(TCP_IP);
  }
  /*
    while (!wifiClientPh.connect(TCP_IP_PHONE, TCP_PORT)) {
    delay(300);
    Serial.print("Attempting to connect to PHONE SERVER: ");
    Serial.println(TCP_IP_PHONE);
    }
  */
  reg_ID();

  delay(1000);
  xTaskCreate(
    askPos,          /* Task function. */
    "askPos",        /* String with name of task. */
    10000,            /* Stack size in words. */
    NULL,             /* Parameter passed as input of the task */
    1,                /* Priority of the task. */
    &xHandle);            /* Task handle. */
}



void reg_ID()
{
  strcpy(buf, "Register|");
  strcat(buf, client_ID);
  wifiClient.write(buf, strlen(buf));
  wifiClient.flush();
}

void send_mes(char ID[], char mes[])
{
  sprintf(buf, "%s|%s", ID, mes);
  wifiClient.write(buf, strlen(buf));
  wifiClient.flush();
}

void movewall() {
  long df, dl, dr;
  df = ultrasonicGetDistance(usTrigPins[U_F], usEchoPins[U_F]);
  dl = ultrasonicGetDistance(usTrigPins[U_L], usEchoPins[U_L]);
  dr = ultrasonicGetDistance(usTrigPins[U_R], usEchoPins[U_R]);
  backward(100);
  delay(400);
  if (dl < 10) right(150);
  else  left(150);
  delay(150);
  pastx = -1;
  nomove(0);
  delay(100);
}

void moving() {
  long df, dl, dr;
  df = ultrasonicGetDistance(usTrigPins[U_F], usEchoPins[U_F]);
  dl = ultrasonicGetDistance(usTrigPins[U_L], usEchoPins[U_L]);
  dr = ultrasonicGetDistance(usTrigPins[U_R], usEchoPins[U_R]);
  if (pastx == -1) {
    pastx = MyPosX;
    pasty = MyPosY;
    forward(300);
    delay(400);
    nomove(100);
  }
  else {
    double goal = atan2(DstPosY - MyPosY, DstPosX - MyPosX) * 180 / pi;
    double mine = atan2(MyPosY - pasty, MyPosX - pastx) * 180 / pi;
    double judgement = goal - mine;
    if (judgement > 180)  judgement = judgement - 360;
    if (judgement < -180)  judgement = judgement + 360;
    if (judgement > 0) {
      right(100);
      delay(75);
    }
    if (judgement < 0) {
      left(100);
      delay(75);
    }
    pastx = MyPosX;
    pasty = MyPosY;
    forward(300);
    delay(300);
    nomove(0);
  }
  if (df <= 10) movewall();
  if (dr <= 7) {
    left(150);
    delay(150);
    nomove(100);
  }
  if (dl <= 7) {
    right(150);
    delay(150);
    nomove(100);
  }
}

void askPos( void * parameter )
{
  while (1)
  {
    if ((messageLen = wifiClient.available()) > 0) {
      int i = 0;
      do
      {
        buf[i++] = wifiClient.read();
      } while (i < 100 && buf[i - 1] != '\r');

      buf[i - 1] = '\0';
      recv_ID = strtok(buf, "|\0");
      Serial.print(recv_ID);
      Serial.print(":");
      recv_buf = strtok(NULL, "|\0");
      Serial.println(recv_buf);
      sscanf(recv_buf, "POS:(%d, %d)", &MyPosX, &MyPosY);
      char array1[32];
      char array2[32];
      char array3[32];
      if (sscanf(recv_buf, "Treasure:(%d, %d)(%d, %d)(%d, %d)(%d, %d)", &treasure1x, &treasure1y, &treasure2x, &treasure2y, &treasure3x, &treasure3y, &treasure4x, &treasure4y)) {
        DstPosX = treasure4x;
        DstPosY = treasure4y;
        if(DstPosX < 10)  DstPosX = treasure3x;
        if(DstPosY < 10)  DstPosY = treasure3y;
        if(DstPosX < 10)  DstPosX = treasure2x;
        if(DstPosY < 10)  DstPosY = treasure2y;
        if(DstPosX < 10)  DstPosX = treasure1x;
        if(DstPosY < 10)  DstPosY = treasure1y;
        Serial.print("DstPosX:");
        Serial.println(DstPosX);
        Serial.print("DstPosY:");
        Serial.println(DstPosY);
      }
      else if (sscanf(recv_buf, "Treasure:(%d, %d)(%d, %d)(%d, %d)", &treasure1x, &treasure1y, &treasure2x, &treasure2y, &treasure3x, &treasure3y)) {
        DstPosX = treasure3x;
        DstPosY = treasure3y;
        if(DstPosX < 10)  DstPosX = treasure2x;
        if(DstPosY < 10)  DstPosY = treasure2y;
        if(DstPosX < 10)  DstPosX = treasure1x;
        if(DstPosY < 10)  DstPosY = treasure1y;
        Serial.print("DstPosX:");
        Serial.println(DstPosX);
        Serial.print("DstPosY:");
        Serial.println(DstPosY);
      }
      else if (sscanf(recv_buf, "Treasure:(%d, %d)(%d, %d)", &treasure1x, &treasure1y, &treasure2x, &treasure2y)) {
        DstPosX = treasure1x;
        DstPosY = treasure1y;
        if(DstPosX < 10)  DstPosX = treasure1x;
        if(DstPosY < 10)  DstPosY = treasure1y;
        Serial.print("DstPosX:");
        Serial.println(DstPosX);
        Serial.print("DstPosY:");
        Serial.println(DstPosY);
      }
      else if (sscanf(recv_buf, "Treasure:(%d, %d)", &treasure1x, &treasure1y)) {
        DstPosX = treasure1x;
        DstPosY = treasure1y;
        Serial.print("DstPosX:");
        Serial.println(DstPosX);
        Serial.print("DstPosY:");
        Serial.println(DstPosY);
      }
      else if (sscanf(recv_buf, "False:%s", array1)) {
        Serial.print("Name:");
        //Serial.println(teammate);
        //strcpy(array1,teammate.c_str());
        sprintf(array2, "(%d, %d)", MyPosX, MyPosY);
        Serial.print(array1);
        Serial.println(array2);
        send_mes(array1, array2);
        if (stepfinish == 0) {
          if (MyPosX - DstPosX < 60 && MyPosX - DstPosX > -60)
            if (MyPosY - DstPosY < 60 && MyPosY - DstPosY > -60) {
              stepfinish = 1;
              nomove(100);
            }
        }
      }
      else if (sscanf(recv_buf, "(%d, %d)", &mytreasurex, &mytreasurey)) {
        Serial.print("mytreasurex:");
        Serial.println(mytreasurex);
        Serial.print("mytreasurey:");
        Serial.println(mytreasurey);
      }
      Serial.print("MyPosX:");
      Serial.println(MyPosX);
      Serial.print("MyPosY:");
      Serial.println(MyPosY);
      if (strcmp(recv_buf, "Start") == 0) {
        letgo = true;
        send_mes("Position", "");
      }
      else if (strcmp(recv_buf, "Done") == 0) {
        letgo = false;
      }
      else if (letgo == false && stepfinish != 0) {
        nomove(0);
        DstPosX = -1;
        DstPosY = -1;
        mytreasurex = -1;
        mytreasurey = -1;
        pastx = -1;
        pasty = -1;
        getgoal = 0;
        stepfinish = 0;
      }
      //send_phone(MyPosX, MyPosY);
      else if (getgoal == 0 && letgo == true) {
        send_mes("Treasure", "");
        getgoal = -1;
      }
      else    {
        /*
          sprintf(array3, "t(%d, %d)", MyPosX, MyPosY);
          send_mes(" 509", array3);
          send_mes(" zanzan", array3);
          send_mes(" hihilife", array3);
        */
        send_mes("Position", "");
      }
    }
    delay(100);
  }
  Serial.println("Ending task 1");
  vTaskDelete( NULL );
}

void steponedecide1(int x, int y) {
  DstPosX = x;
  DstPosY = y;
}
void steponedecide2(int x1, int y1, int x2, int y2) {
  int a = (x1 - MyPosX) * (x1 - MyPosX) + (y1 - MyPosY) * (y1 - MyPosY);
  int b = (x2 - MyPosX) * (x2 - MyPosX) + (y2 - MyPosY) * (y2 - MyPosY);
  if (a > b) {
    DstPosX = x2;
    DstPosY = y2;
  }
  else {
    DstPosX = x1;
    DstPosY = y1;
  }
}
void steponedecide3(int x1, int y1, int x2, int y2, int x3, int y3) {

  int a = (x1 - MyPosX) * (x1 - MyPosX) + (y1 - MyPosY) * (y1 - MyPosY);
  int b = (x2 - MyPosX) * (x2 - MyPosX) + (y2 - MyPosY) * (y2 - MyPosY);
  int c = (x3 - MyPosX) * (x3 - MyPosX) + (y3 - MyPosY) * (y3 - MyPosY);
  int thelowest = a;
  if (thelowest > b) thelowest = b;
  if (thelowest > c) thelowest = c;
  if (thelowest == a) {
    DstPosX = x1;
    DstPosY = y1;
  }
  else if (thelowest == b) {
    DstPosX = x2;
    DstPosY = y2;
  }
  else if (thelowest == c) {
    DstPosX = x3;
    DstPosY = y3;
  }
}
void steponedecide4(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4) {

  int a = (x1 - MyPosX) * (x1 - MyPosX) + (y1 - MyPosY) * (y1 - MyPosY);
  int b = (x2 - MyPosX) * (x2 - MyPosX) + (y2 - MyPosY) * (y2 - MyPosY);
  int c = (x3 - MyPosX) * (x3 - MyPosX) + (y3 - MyPosY) * (y3 - MyPosY);
  int d = (x4 - MyPosX) * (x4 - MyPosX) + (y4 - MyPosY) * (y4 - MyPosY);
  int thelowest = a;
  if (thelowest > b) thelowest = b;
  if (thelowest > c) thelowest = c;
  if (thelowest > d) thelowest = d;
  if (thelowest == a) {
    DstPosX = x1;
    DstPosY = y1;
  }
  else if (thelowest == b) {
    DstPosX = x2;
    DstPosY = y2;
  }
  else if (thelowest == c) {
    DstPosX = x3;
    DstPosY = y3;
  }
  else if (thelowest == d) {
    DstPosX = x4;
    DstPosY = y4;
  }
}
/*
  void send_phone(int x, int y)
  {
  sprintf(buf_phsend, "(%d,%d)", x, y);
  wifiClientPh.write(buf_phsend, strlen(buf_phsend));
  wifiClientPh.flush();
  }

  void handleCommand()
  {
  if (letgo == true) {
    // Stop moving
    if (bufPh[1] == 'E') {
      nomove(100);
    }
    else {
      switch (bufPh[0]) {
        case 'F':   // Forward
          forward(100);
          break;
        case 'B':   // Backward
          backward(100);
          break;
        case 'L':   // Turn left
          left(100);
          break;
        case 'R':   // Turn right
          right(100);
          break;
        case 'Z':   // Report ultrasonic distance and color
          //    reportUltrasonic();
          //reportColorSensor();
          break;
      }
    }
  }
  }
*/
void loop()
{
  //if ((phmessageLen = wifiClientPh.available()) > 0) {
  //bufPh[0] = wifiClientPh.read();
  //bufPh[1] = wifiClientPh.read();
  //bufPh[phmessageLen] = '\0';
  // Serial.println(buf);
  //handleCommand();
  if (letgo == true) {
    if (stepfinish == 1 && mytreasurex != -1) {
      DstPosX = mytreasurex;
      DstPosY = mytreasurey;
      stepfinish = 2;
    }
    else if (stepfinish == 1) {
      nomove(100);
    }
    else moving();
  }
  else if (stepfinish == 0 && DstPosX != -1) {
    moving();
  }
  delay(100);
}
