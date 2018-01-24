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

/* Pin assignment */
static const uint8_t usTrigPins[NUM_OF_ULTRASONIC_PIN] = {2, 4, 9 };  // F, L, R
static const uint8_t usEchoPins[NUM_OF_ULTRASONIC_PIN] = {3, 5, 10 };  // F, L, R
static const uint8_t motorPins[NUM_OF_MOTOR_PIN] = {14, 15, 16, 17};  //  L_F, L_B,R_F, R_B
int thiscase = 0;
long ultrasonicGetDistance(uint8_t trig, uint8_t echo)
{
    long duration;

    pinMode(trig, OUTPUT);
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(5);
    digitalWrite(trig, LOW);

    pinMode(echo, INPUT);
    duration = pulseIn(echo, HIGH, 5000000L);
    return duration / 29 / 2;
}
void forward(int t)
{
    analogWrite(motorPins[L_F], 100);
    analogWrite(motorPins[L_B], 0);
    analogWrite(motorPins[R_F], 100);
    analogWrite(motorPins[R_B], 0);
    delay(t);
        
}
void backward(int t)
{
        analogWrite(motorPins[L_F], 0);
        analogWrite(motorPins[L_B], 150);
        analogWrite(motorPins[R_F], 150);
        analogWrite(motorPins[R_B], 0);
        delay(t);
        analogWrite(motorPins[L_F], 0);
        analogWrite(motorPins[L_B], 150);
        analogWrite(motorPins[R_F], 0);
        analogWrite(motorPins[R_B], 150);
        delay(t);

        
}
void left(int t)
{

        analogWrite(motorPins[L_F], 0);
        analogWrite(motorPins[L_B], 150);
        analogWrite(motorPins[R_F], 150);
        analogWrite(motorPins[R_B], 0);
        delay(t);
}
void right(int t)
{
        analogWrite(motorPins[L_F], 150);
        analogWrite(motorPins[L_B], 0);
        analogWrite(motorPins[R_F], 0);
        analogWrite(motorPins[R_B], 150);
        delay(t);
}
void nomove(int t)
{
        analogWrite(motorPins[L_F], 0);
        analogWrite(motorPins[L_B], 0);
        analogWrite(motorPins[R_F], 0);
        analogWrite(motorPins[R_B], 0);
        delay(t);
}
void check(){
    long df, dl, dr;
    df = ultrasonicGetDistance(usTrigPins[U_F], usEchoPins[U_F]);
    dl = ultrasonicGetDistance(usTrigPins[U_L], usEchoPins[U_L]);
    dr = ultrasonicGetDistance(usTrigPins[U_R], usEchoPins[U_R]);
        while(dl > 30)  dl = dl - 30;
        while(dr > 30)  dr = dr - 30;
        
        if(((dl - dr) >= 4) && (dl < 30) && (dr < 30)){
          
          nomove(100);
          analogWrite(motorPins[L_F], 0);
          analogWrite(motorPins[L_B], 150);
          analogWrite(motorPins[R_F], 150);
          analogWrite(motorPins[R_B], 0);
          delay(50);
          nomove(100);
          }
        else if(((dr - dl) >= 4) && (dl < 30) && (dr < 30)){
          
          nomove(100);
          analogWrite(motorPins[L_F], 150);
          analogWrite(motorPins[L_B], 0);
          analogWrite(motorPins[R_F], 0);
          analogWrite(motorPins[R_B], 150);
          delay(50);
          nomove(100);
          }
        
  }
void setup()
{ 
  int motorpins=0;
  while(motorpins<NUM_OF_MOTOR_PIN){
    pinMode(motorPins[motorpins],OUTPUT);
    motorpins++;
  }
  Serial.begin(9600);
  while (!Serial)
    ;
}
void loop()
{
  
    long df, dl, dr;
    df = ultrasonicGetDistance(usTrigPins[U_F], usEchoPins[U_F]);
    dl = ultrasonicGetDistance(usTrigPins[U_L], usEchoPins[U_L]);
    dr = ultrasonicGetDistance(usTrigPins[U_R], usEchoPins[U_R]);
    if(thiscase == 0){
      if(dr >= 30) {
          forward(450);
          nomove(100);
          thiscase = 1;
        }
      else if(df >= 15)  {
          check();
          forward(200);
        }
      else if(dl >= 30)  {
          forward(450);
          nomove(100);
          thiscase = 1;
        }
      else{
          nomove(100);
          thiscase  = 1;
        }

      }
    else if(thiscase == 1){
      if(dr >= 30) {
          nomove(200);
          right(215);
          nomove(200);
          forward(600);
        }
      else if(df >= 15)  {
          check();
          forward(200);
        }
      else if(dl >= 30)  {
          nomove(100);
          left(215);
          nomove(100);
        }
      else{
          nomove(100);
          backward(230);
          nomove(100);
        }
      thiscase = 0;
      }
    
}
