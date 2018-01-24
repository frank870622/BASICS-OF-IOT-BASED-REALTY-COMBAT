#include "Arduino.h"
#include "LTimer.h"

#define S0     6
#define S1     7
#define S2     8
#define S3     11
#define OUT    12

LTimer timer0(LTIMER_0);
int     g_array[3];     // 儲存 RGB 值
int     g_flag = 0;     // RGB 過濾順序 R:0 G:1 B:2
float   g_SF[3];        // 儲存白平衡計算後之 RGB 補償係數
int     frequency = 0;  // 被感測器偵測的方波頻率
int     count = 0;      // 重複偵測次數
int     temp[3];
int     temp_min[3] = {255, 255, 255};
int     temp_MAX[3] = {255, 255, 255};
void TCS_Init() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);

  digitalWrite(S0, LOW);  // OUTPUT FREQUENCY SCALING 2%
  digitalWrite(S1, HIGH);
}
void setup() {
  // initialization
  TCS_Init();
  Serial.begin(9600);
  timer0.begin();

  // start the execution
  timer0.start(1000, LTIMER_REPEAT_MODE, _callback0, NULL);

  delay(4000);

  Serial.println("RGB : ");
  for (int i = 0; i < 3; i++)
    Serial.println(g_array[i]);

  // 計算補償係數
  Serial.print("The compensation coefficient of RGB is: ");
  for (int i = 0; i < 3; i++) {
    g_SF[i] = 255.0 / g_array[i];  // R G B 補償係數
    Serial.println(g_SF[i]);
  }
}
// callback function for timer0
void _callback0(void *usr_data) {
  TCS_Callback();
}
// 白平衡
void TCS_WB(int Level0, int Level1) {
  frequency = 0;
  g_flag ++;
  TCS_FilterColor(Level0, Level1);
}

/// 選擇過濾顏色
void TCS_FilterColor(int Level01, int Level02) {
  if (Level01 != 0)
    Level01 = HIGH;

  if (Level02 != 0)
    Level02 = HIGH;

  digitalWrite(S2, Level01);
  digitalWrite(S3, Level02);
}

void TCS_Callback()
{
  switch (g_flag)
  {
    case 0:
      Serial.println("->WB Start");
      TCS_WB(LOW, LOW);              // Red
      break;

    case 1:
      Serial.print("->Frequency R=");
      frequency = pulseIn(OUT, LOW);
      Serial.println(frequency);
      g_array[0] = frequency;
      TCS_WB(HIGH, HIGH);            // Green
      break;

    case 2:
      Serial.print("->Frequency G=");
      frequency = pulseIn(OUT, LOW);
      Serial.println(frequency);
      g_array[1] = frequency;
      TCS_WB(LOW, HIGH);             // Blue
      break;

    case 3:
      Serial.print("->Frequency B=");
      frequency = pulseIn(OUT, LOW);
      Serial.println(frequency);
      g_array[2] = frequency;
      Serial.println("->WB End");
      TCS_WB(HIGH, LOW);             // Clear(no filter)
      break;

    default:
      frequency = 0;
      break;
  }
}

void loop() {
  g_flag = 0; //重置flag

  for (int i = 0; i < 3; i++) {
    temp[i] = int(g_array[i] * g_SF[i]);
    Serial.println(temp[i]);

    if (temp[i] < temp_min[i])
      temp_min[i] = temp[i];
    if (temp[i] > temp_MAX[i])
      temp_MAX[i] = temp[i];
    //Serial.println(temp_min[i]);
    //Serial.println(temp_MAX[i]);
  }

  if (count++ >= 10) {
    Serial.println("Map the value from 0 to 255.");
    for (int i = 0; i < 3; i++) {
      temp[i] = map(temp[i], temp_min[i], temp_MAX[i], 255, 0);
      Serial.println(temp[i]);
    }
  }
  delay(4000);

}
