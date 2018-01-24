#include <WiFiClient.h>
#include <LWiFi.h>
#include <task.h>
#include <math.h>
#include <stdio.h>
#define TCP_PORT 5000

#define SSID "1scream2.4G"
#define PASSWD "2017scream"
#define TCP_IP "192.168.0.50"
#define TCP_IP_PHONE "192.168.0.131"

xTaskHandle xHandle;

//WiFiClient wifiClientPh;
static char buf[128], bufPh[128], buf_send[128], buf_phsend[128];
static char client_ID[] = "sakuramiku", Team[] = "B";
static int messageLen, phmessageLen;
static int MyPosX, MyPosY, DstPosX = -1, DstPosY = -1;
static int pastx = -1, pasty = -1;
const double pi = 3.14159265359;

static int light1x, light1y, light2x, light2y, light3x, light3y;
static int plight1x = -1, plight1y = -1, plight2x = -1, plight2y = -1, plight3x = -1, plight3y = -1;
static int teammatemode = 0;
static int teammate1x = 99999999, teammate1y = 99999999, teammate2x = 99999999, teammate2y = 99999999, teammate3x = 99999999, teammate3y = 99999999;
static char baseA = 'C', baseB = 'C';
static int hp;

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
  analogWrite(motorPins[L_F], 255);
  analogWrite(motorPins[L_B], 0);
  analogWrite(motorPins[R_F], 255);
  analogWrite(motorPins[R_B], 0);
}
void forward2(int t)
{
  analogWrite(motorPins[L_F], 255);
  analogWrite(motorPins[L_B], 0);
  analogWrite(motorPins[R_F], 255);
  analogWrite(motorPins[R_B], 0);
}
void backward(int t)
{
  analogWrite(motorPins[L_F], 0);
  analogWrite(motorPins[L_B], 255);
  analogWrite(motorPins[R_F], 0);
  analogWrite(motorPins[R_B], 255);
}
void left(int t)
{

  analogWrite(motorPins[L_F], 0);
  analogWrite(motorPins[L_B], 255);
  analogWrite(motorPins[R_F], 255);
  analogWrite(motorPins[R_B], 0);
}
void right(int t)
{
  analogWrite(motorPins[L_F], 255);
  analogWrite(motorPins[L_B], 0);
  analogWrite(motorPins[R_F], 0);
  analogWrite(motorPins[R_B], 255);
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
  strcpy(buf, "Register");
  strcat(buf, Team);
  strcat(buf, "|");
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
      if (judgement > 90)  right(300);
      else  right(100);
      delay(75);
    }
    if (judgement < 0) {
      if (judgement < -90) left(300);
      else  left(100);
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
      if (sscanf(recv_buf, "POS:(%d, %d)BaseA:%cBaseB:%cTowers:(%d, %d)(%d, %d)(%d, %d)Blood:%d", &MyPosX, &MyPosY, &baseA, &baseB, &light1x, &light1y, &light2x, &light2y, &light3x, &light3y, &hp)) {
        if (light1x != plight1x || light1y != plight1y || light2x != plight2x || light2y != plight2y || light3x != plight3x || light3y != plight3y) {
          plight1x = light1x;
          plight1y = light1y;
          plight2x = light2x;
          plight2y = light2y;
          plight3x = light3x;
          plight3y = light3y;
          teammatemode = 1;
        }
      }
      Serial.println(recv_buf);
      if (sscanf(recv_buf, "zanzan:(&d, &d)", &teammate1x, &teammate1y)) {
      }
      else if (sscanf(recv_buf, "509:(&d, &d)", &teammate2x, &teammate2y)) {
      }
      else if (sscanf(recv_buf, "hihilife:(&d, &d)", &teammate3x, &teammate3y)) {
      }
      if (strcmp(recv_buf, "Start") == 0) {
        letgo = true;
        send_mes("Position", "");
        delay(100);
      }
      else if (strcmp(recv_buf, "Done") == 0) {
        letgo = false;
        nomove(0);
        delay(100);
      }
      //send_phone(MyPosX, MyPosY);
      else    {
        send_mes("Position", "");
        delay(100);
      }
      if (teammatemode == 1) {
        send_mes("AskPos", "zanzan");
        teammatemode = 2;
      }
      else if (teammatemode == 2) {
        send_mes("AskPos", "509");
        teammatemode = 3;
      }
      else if (teammatemode == 3) {
        send_mes("AskPos", "hihilife");
        teammatemode = 0;
        DstPosX = -1;
        DstPosY = -1;
      }
    }
  }
  Serial.println("Ending task 1");
  vTaskDelete( NULL );
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

void decide_dst() {
  int judge[12];
  int min1 = 99999, min2 = 99999, min3 = 99999;
  int min1road, min2road, min3road;
  for (int i = 0; i < 12; ++i)  judge[i] = 999999;
  judge[0] = (light1x - teammate1x) * (light1x - teammate1x) + (light1y - teammate1y) * (light1y - teammate1y);
  judge[1] = (light2x - teammate1x) * (light2x - teammate1x) + (light2y - teammate1y) * (light2y - teammate1y);
  judge[2] = (light3x - teammate1x) * (light3x - teammate1x) + (light3y - teammate1y) * (light3y - teammate1y);
  judge[3] = (light1x - teammate2x) * (light1x - teammate2x) + (light1y - teammate2y) * (light1y - teammate2y);
  judge[4] = (light2x - teammate2x) * (light2x - teammate2x) + (light2y - teammate2y) * (light2y - teammate2y);
  judge[5] = (light3x - teammate2x) * (light3x - teammate2x) + (light3y - teammate2y) * (light3y - teammate2y);
  judge[6] = (light1x - teammate3x) * (light1x - teammate3x) + (light1y - teammate3y) * (light1y - teammate3y);
  judge[7] = (light2x - teammate3x) * (light2x - teammate3x) + (light2y - teammate3y) * (light2y - teammate3y);
  judge[8] = (light3x - teammate3x) * (light3x - teammate3x) + (light3y - teammate3y) * (light3y - teammate3y);
  judge[9] = (light1x - MyPosX) * (light1x - MyPosX) + (light1y - MyPosY) * (light1y - MyPosY);
  judge[10] = (light2x - MyPosX) * (light2x - MyPosX) + (light2y - MyPosY) * (light2y - MyPosY);
  judge[11] = (light3x - MyPosX) * (light3x - MyPosX) + (light3y - MyPosY) * (light3y - MyPosY);
  for (int i = 0; i < 12 ; ++i) {
    if (min1 > judge[i]) {
      min1 = judge[i];
      min1road = i;
    }
  }
  if (min1road == 9) {
    DstPosX = light1x;
    DstPosY = light1y;
  }
  else if (min1road == 10) {
    DstPosX = light2x;
    DstPosY = light2y;
  }
  else if (min1road == 11) {
    DstPosX = light3x;
    DstPosY = light3y;
  }
  else {
    if (min1road % 3 == 0) {
      judge[0] = 999999;
      judge[3] = 999999;
      judge[6] = 999999;
      judge[9] = 999999;
    }
    else if (min1road % 3 == 1) {
      judge[1] = 999999;
      judge[4] = 999999;
      judge[7] = 999999;
      judge[10] = 999999;
    }
    else if (min1road % 3 == 2) {
      judge[2] = 999999;
      judge[5] = 999999;
      judge[8] = 999999;
      judge[11] = 999999;
    }
    for (int i = 0; i < 12 ; ++i) {
      if (min2 > judge[i]) {
        min2 = judge[i];
        min2road = i;
      }
    }
    if (min2road == 9) {
      DstPosX = light1x;
      DstPosY = light1y;
    }
    else if (min2road == 10) {
      DstPosX = light2x;
      DstPosY = light2y;
    }
    else if (min2road == 11) {
      DstPosX = light3x;
      DstPosY = light3y;
    }
    else {
      if (min2road % 3 == 0) {
        judge[0] = 999999;
        judge[3] = 999999;
        judge[6] = 999999;
        judge[9] = 999999;
      }
      else if (min2road % 3 == 1) {
        judge[1] = 999999;
        judge[4] = 999999;
        judge[7] = 999999;
        judge[10] = 999999;
      }
      else if (min2road % 3 == 2) {
        judge[2] = 999999;
        judge[5] = 999999;
        judge[8] = 999999;
        judge[11] = 999999;
      }
      for (int i = 0; i < 12 ; ++i) {
        if (min3 > judge[i]) {
          min3 = judge[i];
          min3road = i;
        }
      }
      if (min3road == 9) {
        DstPosX = light1x;
        DstPosY = light1y;
      }
      else if (min3road == 10) {
        DstPosX = light2x;
        DstPosY = light2y;
      }
      else if (min3road == 11) {
        DstPosX = light3x;
        DstPosY = light3y;
      }
      else {
        DstPosX = 288;
        DstPosY = 288;
      }
    }
  }
}

void decide_base(int basex, int basey) {
  int judge[4];
  int min1 = 99999, min2 = 99999;
  int min1road, min2road;
  for (int i = 0; i < 4; ++i) judge[4] = 999999;
  judge[0] = (basex - teammate1x) * (basex - teammate1x) + (basey - teammate1y) * (basey - teammate1y);
  judge[1] = (basex - teammate2x) * (basex - teammate2x) + (basey - teammate2y) * (basey - teammate2y);
  judge[2] = (basex - teammate3x) * (basex - teammate3x) + (basey - teammate3y) * (basey - teammate3y);
  judge[3] = (basex - MyPosX) * (basex - MyPosX) + (basey - MyPosY) * (basey - MyPosY);
  for (int i = 0; i < 4; ++i) {
    if (min1 > judge[i]) {
      min1 = judge[i];
      min1road = i;
    }
  }
  if (min1road == 3) {
    DstPosX = basex;
    DstPosY = basey;
  }
  else {
    judge[min1road] = 999999;
    for (int i = 0; i < 4; ++i) {
      if (min2 > judge[i]) {
        min2 = judge[i];
        min2road = i;
      }
    }
    if (min2road == 3) {
      DstPosX = basex;
      DstPosY = basey;
    }
    else {
      DstPosX = 300;
      DstPosY = 300;
    }
  }
}

void loop()
{
  //if ((phmessageLen = wifiClientPh.available()) > 0) {
  //bufPh[0] = wifiClientPh.read();
  //bufPh[1] = wifiClientPh.read();
  //bufPh[phmessageLen] = '\0';
  // Serial.println(buf);
  //handleCommand();
  if (letgo == true && hp > 0) {
    if (Team[0] == 'A') {
      if (baseA == 'O') {
        DstPosX = 90;
        DstPosY = 90;
      }
      else if (baseB == 'O') {
        if (DstPosX == -1) {
          decide_base(485, 485);
        }
      }
    }
    else if (Team[0] == 'B') {
      if (baseA == 'O') {
        if (DstPosX == -1) {
          decide_base(90, 90);
        }
      }
      else if (baseB == 'O') {
        DstPosX = 470;
        DstPosY = 470;
      }
    }
    if (DstPosX == -1) {
      decide_dst();
    }
    else {
      moving();
    }
  }
  else {
    nomove(0);
  }


  delay(100);
}
