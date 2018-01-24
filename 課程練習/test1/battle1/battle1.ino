#include <WiFiClient.h>
#include <LWiFi.h>
#include <task.h>
#include <math.h>
#include <stdio.h>
#define TCP_PORT 5000

#define SSID "2scream"
#define PASSWD "2017scream"
#define TCP_IP "192.168.0.101"
#define TCP_IP_PHONE "192.168.0.131"

xTaskHandle xHandle;

//WiFiClient wifiClientPh;
static char buf[48], bufPh[48], buf_send[32], buf_phsend[32];
static char client_ID[] = " sakuramiku", Team[] = "B";
static int messageLen, phmessageLen;
static int MyPosX, MyPosY, DstPosX, DstPosY;
static int treasure1x,treasure1y,treasure2x,treasure2y,treasure3x,treasure3y,treasure4x,treasure4y  ;
static int pastx = -1, pasty = -1;
static double base, now;
static int mod = 0, start;
const double pi = 3.14159265359;

WiFiClient wifiClient;
LTimer timer0(LTIMER_0);
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

LTimerStatus start(
    uint32_t timeoutMS,
    LTimerMode timerMode,
    ltimer_callback_t callbackFunc,
    void *userData
);

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
  // initialization
  timer0.begin();
  
  // start the execution
  timer0.start(1000, LTIMER_REPEAT_MODE, _callback0, NULL);
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
/*
  void movewall() {
  long df, dl, dr;
  df = ultrasonicGetDistance(usTrigPins[U_F], usEchoPins[U_F]);
  dl = ultrasonicGetDistance(usTrigPins[U_L], usEchoPins[U_L]);
  dr = ultrasonicGetDistance(usTrigPins[U_R], usEchoPins[U_R]);
  backward(100);
  delay(100);
  if (mod == 2) {
    left(150);
    delay(150);
    nomove(100);
    delay(100);
    forward(300);
    delay(600);
    nomove(100);
    delay(100);
    right(150);
    delay(150);
  }
  else if (mod == 1) {
    right(150);
    delay(150);
    nomove(100);
    delay(100);
    forward(300);
    delay(600);
    nomove(100);
    delay(100);
    left(150);
    delay(150);
  }
  pastx = -1;
  nomove(0);
  delay(100);
  }
*/
void moving() {
  long df, dl, dr;
  df = ultrasonicGetDistance(usTrigPins[U_F], usEchoPins[U_F]);
  dl = ultrasonicGetDistance(usTrigPins[U_L], usEchoPins[U_L]);
  dr = ultrasonicGetDistance(usTrigPins[U_R], usEchoPins[U_R]);
  if (mod == 0) {
    if (dr < 30) mod = 1;
    else if (dl < 30) mod = 2;
    if (mod == 1)  {
      forward(0);
      delay(4700);
      nomove(100);
      delay(100);
      left(100);
    }
    else if (mod == 2) {
      forward2(0);
      delay(4700);
      nomove(100);
      delay(100);
      right(100);
    }
    delay(120);
    nomove(0);
    delay(100);
  }
  else {
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
  }
  //if (df <= 5) movewall();
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
      } while (i < 32 && buf[i - 1] != '\r');

      buf[i - 1] = '\0';
      recv_ID = strtok(buf, "|\0");
      Serial.print(recv_ID);
      Serial.print(":");
      recv_buf = strtok(NULL, "|\0");
      Serial.println(recv_buf);
      sscanf(recv_buf, "POS:(%d, %d)(%d, %d)", &MyPosX, &MyPosY, &DstPosX, &DstPosY);
      
      Serial.println(DstPosX);
      Serial.println(DstPosY);
      Serial.println(MyPosX);
      Serial.println(MyPosY);
      if (strcmp(recv_buf, "Start") == 0) {
        letgo = true;
        send_mes("Treasure", "");
      }
      if (strcmp(recv_buf, "Done") == 0) {
        letgo = false;
        nomove(0);
      }
      //send_phone(MyPosX, MyPosY);
      send_mes("Position", "");
    }
    delay(100);
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
void loop()
{
  //if ((phmessageLen = wifiClientPh.available()) > 0) {
  //bufPh[0] = wifiClientPh.read();
  //bufPh[1] = wifiClientPh.read();
  //bufPh[phmessageLen] = '\0';
  // Serial.println(buf);
  //handleCommand();
  if (letgo == true) {
    moving();
  }
  delay(50);
}
