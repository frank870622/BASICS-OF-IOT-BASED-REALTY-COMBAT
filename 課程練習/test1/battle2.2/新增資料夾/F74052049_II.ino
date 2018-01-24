#include <math.h>
#include <WiFiClient.h>
#include <LWiFi.h>
#include <task.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define SSID "2scream"
#define PASSWD "2017scream"
#define TCP_IP "192.168.0.100"
#define TCP_PORT 5000

//#define Kp 45
#define dif_point 3

#define EqualString(a,b) !strcmp(a,b)
#define Distance(a,b) hypot(a.x-b.x,a.y-b.y)

#define BUTTON 6
enum MotorPinID{
    MR_F=14,
    MR_B,
    ML_F,
    ML_B
};

class point{
    public:
        int x=0,y=0;
        point(int a=-1,int b=-1){
            this->x=a;
            this->y=b;
        }
        point operator=(const point &a){
            this->x=a.x;
            this->y=a.y;
            return *this;
        }
        bool operator==(const point &a){return (a.x==this->x)&&(a.y==this->y);}
};
point pos[10], destination,myDestination,treasure[4];

WiFiClient wifiClient;
const char client_ID[]="509";
const char teammate[3][20]={"sakuramiku","zanzan","hihilife"};
float angle=0;
int car_status=0,pre_car_status; //0:stop 1:start 2:GotTreasure 3:stuck

IPAddress ip;

void go(float fix=0,float power=255);
void setup()
{
    for(int i=MR_F;i<=ML_B;++i){
        pinMode(i,OUTPUT);
        digitalWrite(i,LOW);
    }
    pinMode(BUTTON,INPUT);

    int wifi_status = WL_IDLE_STATUS;
    Serial.begin(115200);
    while (!Serial){ delay(500);} 

    // set WiFi
    // WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASSWD);
    while(wifi_status != WL_CONNECTED){
        delay(500);
        WiFi.begin(SSID, PASSWD);
        wifi_status =WiFi.begin(SSID, PASSWD);
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(SSID);
        Serial.println(wifi_status);
    }
    while(!wifiClient.connect(TCP_IP, TCP_PORT)){
        delay(300);
        Serial.print("Attempting to connect to SERVER: ");
        Serial.println(TCP_IP);
    }
    send_mes("Register",client_ID);
    delay(1000);
    xTaskCreate(
            receive_mes,     /* Task function. */
            "receive_mes",   /* String with name of task. */
            10000,           /* Stack size in words. */
            NULL,            /* Parameter passed as input of the task */
            0,               /* Priority of the task. */
            NULL);           /* Task handle. */
}

void loop(){   
    //initial start
    if(digitalRead(BUTTON)==HIGH) car_status=3;
    if(car_status==1){
        //forward a little to acknowledge direction
        if(pre_car_status!=1){
            memset(pos,-1,sizeof(pos));
            go();
            pre_car_status=car_status;
            return;
        }
        //calculate current angle
        if(pos[dif_point]==point()) angle=0;
        else angle=atan2(destination.y-pos[0].y,destination.x-pos[0].x)-atan2(pos[0].y-pos[dif_point].y,pos[0].x-pos[dif_point].x);
        if(angle>M_PI) angle-=M_PI*2;
        if(angle<-M_PI) angle+=M_PI*2;
        //the longer the distance, the larger the turning angle
        double Kp=20000/(Distance(destination,pos[0])+100);
        //use button to detect collision
        go(angle*Kp,Distance(destination,pos[0])/3+150);
    }else if(!car_status){
        go(0,0);
    }else if(car_status==2||car_status==3){
        go(0,-255);
        delay(500);
        go(200,0);
        delay(500);
        car_status=1;
    }
    //update car status
    pre_car_status=car_status;
    delay(50);
} 

//send message to server
void send_mes(const char *ID,const char *mes)
{
    char send_buf[50];
    sprintf(send_buf,"%s|%s",ID,mes);
    Serial.println(send_buf);
    wifiClient.write(send_buf, strlen(send_buf));
    wifiClient.flush();
}
//get and verify message from server
void receive_mes( void * parameter )
{
    char *recv_ID,*pch;
    char recv_buf[50];
    int messageLen;
    uint8_t treasure_count=0,playerIndex=0;
    while(1)
    {
        //get message
        if ((messageLen = wifiClient.available()) > 0) {
            {
                unsigned i = 0;
                do
                {
                    recv_buf[i++] = wifiClient.read();
                }while(i<sizeof(recv_buf) && recv_buf[i-1]!='\r');
                recv_buf[i-1] = '\0';
            }
            Serial.println(recv_buf);
            //get the sender of the message
            recv_ID = strtok(recv_buf,"|");
            pch = strtok(NULL,":");
            //Serial.print(recv_ID);
            //Serial.print(":");
            //Serial.print(pch);
            //get from master
            if(EqualString(recv_ID,"Master")){
                if(pch){
                    if(EqualString(pch,"Start")){
                        delay(100);
                        send_mes("Treasure","");
                        car_status=1;
                    }else if(EqualString(pch,"Done")){
                        myDestination=(point){-1,-1};
                    }else if(EqualString(pch,"Treasure")){
                        for(treasure_count=0;treasure_count<4;++treasure_count){
                            if(!(pch=strtok(NULL,")"))) break;
                            sscanf(pch,"(%d, %d",&treasure[treasure_count].x,&treasure[treasure_count].y);
                        }
                        //change there depend your group rule
                        if(playerIndex>=treasure_count) playerIndex=0;
                        destination=treasure[playerIndex];
                    }else if(EqualString(pch,"POS")){
                        memmove(pos+1,pos,sizeof(pos)-sizeof(point));
                        pch=strtok(NULL,")");
                        sscanf(pch,"(%d, %d",&pos[0].x,&pos[0].y);
                        //car stuck 
                        if(Distance(pos[9],pos[0])<4) car_status=3;
                    }else if(EqualString(pch,"False")){
                        pch=strtok(NULL,"\0");
                        char s[20];
                        sprintf(s,"(%d, %d)",pos[0].x,pos[0].y);
                        send_mes(pch,s);
                        if(myDestination==(point){-1,-1}){
                            //go to another treasure
                            if(playerIndex>=treasure_count) playerIndex=0;
                            else ++playerIndex;
                            destination=treasure[playerIndex];
                        }//go to my treasure
                        else destination=myDestination;
                        car_status=2;
                    }
                }
                //get from other teammates
            }else{
                sscanf(pch,"(%d, %d)",&myDestination.x,&myDestination.y);
            }
        }
        if(car_status==1) send_mes("Position","");
        delay(100);
    }
    vTaskDelete( NULL );
}
void go(float fix,float power){
    float l=power+fix,r=power-fix;
    analogWrite(MR_B,(r<0)? -r:0); 
    analogWrite(MR_F,(r>0)? r:0); 
    analogWrite(ML_B,(l<0)? -l:0); 
    analogWrite(ML_F,(l>0)? l:0); 
}
