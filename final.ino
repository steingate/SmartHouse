#include <Wire.h>
#include <string.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <DFRobot_DHT11.h>
Adafruit_BMP280 bmp; // I2C




//User Modified Part
#define wifi_ssid     "Redmi10X"    
#define wifi_psw      ""     
#define clientIDstr   "device"
#define timestamp     "999"
#define ProductKey    "k0flrc7aOW0"
#define DeviceName    "device"
#define DeviceSecret  "f89fe2995672b18302c40149d16bc26e"
#define password      "8A1677BFECF0B7852BF122DC771F8DC5824142A9"



//Logic Preset
#define OFF           0
#define ON            1
#define MUTE          2
#define KEEP_OFF      2
#define KEEP_ON       3


#define Buzzer_ON   digitalWrite(BuzzerPin,HIGH)
#define Buzzer_OFF  digitalWrite(BuzzerPin,LOW)




//ATcmd Format
#define AT                    "AT\r"
#define AT_OK                 "OK"
#define AT_REBOOT             "AT+REBOOT\r"
#define AT_ECHO_OFF           "AT+UARTE=OFF\r"
#define AT_MSG_ON             "AT+WEVENT=ON\r"

#define AT_WIFI_START         "AT+WJAP=%s,%s\r"
#define AT_WIFI_START_SUCC    "+WEVENT:STATION_UP"

#define AT_MQTT_AUTH          "AT+MQTTAUTH=%s&%s,%s\r"
#define AT_MQTT_CID           "AT+MQTTCID=%s|securemode=3\\,signmethod=hmacsha1\\,timestamp=%s|\r"
#define AT_MQTT_SOCK          "AT+MQTTSOCK=%s.iot-as-mqtt.cn-shanghai.aliyuncs.com,1883\r"

#define AT_MQTT_AUTOSTART_OFF "AT+MQTTAUTOSTART=OFF\r"
#define AT_MQTT_ALIVE         "AT+MQTTKEEPALIVE=500\r"
#define AT_MQTT_START         "AT+MQTTSTART\r"
#define AT_MQTT_START_SUCC    "+MQTTEVENT:CONNECT,SUCCESS"
#define AT_MQTT_PUB_SET       "AT+MQTTPUB=/sys/%s/%s/thing/event/property/post,1\r"
#define AT_MQTT_PUB_ALARM_SET "AT+MQTTPUB=/sys/%s/%s/thing/event/GasAlarm/post,1\r"
#define AT_MQTT_PUB_DATA      "AT+MQTTSEND=%d\r"
#define JSON_DATA_PACK        "{\"id\":\"100\",\"version\":\"1.0\",\"method\":\"thing.event.property.post\",\"params\":{\"RoomTemp\":%d.%02d,\"AC\":%d,\"Fan\":%d,\"Buzzer\":%d,\"GasDetector\":%d}}\r"
#define JSON_DATA_PACK_2      "{\"id\":\"110\",\"version\":\"1.0\",\"method\":\"thing.event.property.post\",\"params\":{\"LightDetector\":%d,\"Curtain\":%d,\"Light\":%d,\"SoilHumi\":%d,\"Pump\":%d,\"eCO2\":%d,\"TVOC\":%d}}\r"
#define JSON_DATA_PACK_3      "{\"id\":\"110\",\"version\":\"1.0.0\",\"method\":\"/sys/%s/%s/thing/event/property/post\",\"params\":{\"temperature\":%d.%02d,\"photores\":%d}}\r"
#define JSON_DATA_PACK_4      "{\"id\":\"100\",\"version\":\"1.0\",\"method\":\"thing.event.property.post\",\"params\":{\"temprature\":%d,\"humidity\":%d,\"fan\":%d,\"heat\":%d,\"pump\":%d,\"settemprature\":%d}}\r"
#define JSON_DATA_PACK_ALARM  "{\"id\":\"110\",\"version\":\"1.0\",\"method\":\"thing.event.GasAlarm.post\",\"params\":{\"GasDetector\":%d}}\r"
#define AT_MQTT_PUB_DATA_SUCC "+MQTTEVENT:PUBLISH,SUCCESS"
#define AT_MQTT_UNSUB         "AT+MQTTUNSUB=2\r"
#define AT_MQTT_SUB           "AT+MQTTSUB=2,/sys/%s/%s/thing/service/property/set,1\r"
#define AT_MQTT_SUB_SUCC      "+MQTTEVENT:2,SUBSCRIBE,SUCCESS"
#define AT_MQTT_CLOSE          "AT+MQTTCLOSE\r"

#define AT_BUZZER_MUTE           "\"Buzzer\":2"


#define DEFAULT_TIMEOUT       10   //seconds
#define BUF_LEN               100
#define BUF_LEN_DATA          190

char      ATcmd[BUF_LEN];
char      ATbuffer[BUF_LEN];
char      ATdata[BUF_LEN_DATA];
#define BuzzerPin             3
int   Buzzer = OFF;
DFRobot_DHT11 DHT;

String data;  //new add
int frequency; // new add
int ColorGreen=0;// new add
int ColorRed=0; // new add
int ColorBlue=0; // new add
int temprature,set_temprature=0,humidity;
bool pump=0,heat=0,fan=0,flag=0;

unsigned long timeStart;
String inString="";


StaticJsonDocument<300> doc;

void setup() {
  //Serial Initial
  Serial3.begin(115200);
  Serial.begin(115200);
  
  pinMode(2,OUTPUT); 
  pinMode(4,OUTPUT); 
  pinMode(7,OUTPUT); 
  pinMode(8,OUTPUT); 
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);

  pinMode(13,OUTPUT); //板载led
  
  
  //  Pin_init();
  BEEP(1);
  
  //Cloud Initial
  while(1)
  {
    if(!WiFi_init())continue;
    BEEP(2);
    if(!Ali_connect())continue;
    break;
  }
  BEEP(3);
  timeStart = millis();
}


//设置了一个解析的函数，利用库ArdunioJson版本是6.2.13
int parse(String data){

  //找到“{”，并截断到倒数第二个字符
  int commaPosition;  
  commaPosition = data.indexOf('{');
  data= data.substring(commaPosition, data.length());
  char *tempdata=data.c_str();
  Serial.println(data);
  
  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, data);

  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  // 获取数据
  const char* method  = doc["method"];
  const char* id      = doc["id"];
  if(strstr(tempdata,"settemprature")!=NULL) set_temprature=doc["params"]["settemprature"];
  if(strstr(tempdata,"pump")!=NULL) {
    pump=doc["params"]["pump"];
    flag=1;
  }
  
//  const char* sensor = doc["sensor"];
//  int time1 = doc["time1"];
//  double latitude = doc["data"][0];
//  double longitude = doc["data"][1];

  // Print values.
  Serial.println("==============Start================");
  //Serial.println(method);
  //Serial.println(id);
 // Serial.println(frequency);
  Serial.println(set_temprature,DEC);
  return frequency;
  
}
  
void loop() {
    delay(10);
    //有串口的数据进来就暂存在inString里
    if (Serial3.available()>0){
      delay(10);
      inString=Serial3.readString();
      if (inString!=""){
        data=inString;
        frequency=parse(data);   
      }
      //Serial.println(data);
      if(pump){
        digitalWrite(4,1);
      } else {
        digitalWrite(4,0);
        digitalWrite(2,0);
        heat=0;
      }
      if(flag){
          if(temprature<set_temprature){
            digitalWrite(2,1);
            heat=1;
          } else {
            digitalWrite(2,0);
            heat=0;
          }
      }
      Upload();
   }
   

 
    //if (frequency==0) { frequency=1;}
      
    //Serial.println(frequency);  
    if(Serial3.available()==0) {    
    
      //端口7,8,9混色
      analogWrite(9, ColorBlue);
      analogWrite(7, ColorGreen);
      analogWrite(8, ColorRed);
    }
  
    if((millis()-timeStart)>1000)
    {
      DHT.read(A5);
      temprature=DHT.temperature;
      humidity=DHT.humidity;
      if(humidity>=60) {
        fan=1;
        digitalWrite(10,1);
      }
      else {
        fan=0;
        digitalWrite(10,0);
      }
      Serial.println(temprature,DEC);
      Serial.println(set_temprature,DEC);
      Upload();
      timeStart=millis();
      /*Serial.print(F("Temperature = "));
      Serial.print(bmp.readTemperature());
      Serial.println(" *C");*/
      Serial.print(F("PhotoRes = "));
      Serial.print(analogRead(A2));
    }






  }


bool Upload()
{
  bool flag;
  int inte1,frac1;
  int len;

  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_SET,ProductKey,DeviceName);
  flag = check_send_cmd(ATcmd,AT_OK,DEFAULT_TIMEOUT);
   
  
  cleanBuffer(ATdata,BUF_LEN_DATA);

  
  len = snprintf(ATdata,BUF_LEN_DATA,JSON_DATA_PACK_4,temprature,humidity,fan,heat,pump,set_temprature);
  
  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_DATA,len-1);
  flag = check_send_cmd(ATcmd,">",DEFAULT_TIMEOUT);
  if(flag) flag = check_send_cmd(ATdata,AT_MQTT_PUB_DATA_SUCC,20);
  
  
//  delay(500);
/*
  cleanBuffer(ATdata,BUF_LEN_DATA);
  len = snprintf(ATdata,BUF_LEN_DATA,JSON_DATA_PACK_2,LightDetector,Curtain,Light,SoilHumi,Pump,eCO2,TVOC);

  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_DATA,len-1);
  flag = check_send_cmd(ATcmd,">",DEFAULT_TIMEOUT);
  if(flag) flag = check_send_cmd(ATdata,AT_MQTT_PUB_DATA_SUCC,20);
*/
  return flag;
}



bool Ali_connect()
{
  bool flag;
  bool flag1;

  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_AUTH,DeviceName,ProductKey,password);
  flag = check_send_cmd(ATcmd,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;

  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_CID,clientIDstr,timestamp);
  flag = check_send_cmd(ATcmd,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;

  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_SOCK,ProductKey);
  flag = check_send_cmd(ATcmd,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;

  flag = check_send_cmd(AT_MQTT_AUTOSTART_OFF,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;

  flag = check_send_cmd(AT_MQTT_ALIVE,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;

  flag = check_send_cmd(AT_MQTT_START,AT_MQTT_START_SUCC,20);
  if(!flag)return false;

  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_SET,ProductKey,DeviceName);
  flag = check_send_cmd(ATcmd,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;

  //flag = check_send_cmd(AT_MQTT_UNSUB,AT_OK,DEFAULT_TIMEOUT);
  //if(!flag)return false;
  
  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_SUB,ProductKey,DeviceName);
  flag = check_send_cmd(ATcmd,AT_MQTT_SUB_SUCC,DEFAULT_TIMEOUT);
  if(!flag){ BEEP(4);flag1 = check_send_cmd(AT_MQTT_CLOSE,AT_OK,DEFAULT_TIMEOUT);}
  return flag;
}

bool WiFi_init()
{
  bool flag;

  flag = check_send_cmd(AT,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;
  
  flag = check_send_cmd(AT_REBOOT,AT_OK,20);
  if(!flag)return false;
  delay(5000);

  flag = check_send_cmd(AT_ECHO_OFF,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;

  flag = check_send_cmd(AT_MSG_ON,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;
  
  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_WIFI_START,wifi_ssid,wifi_psw);
  flag = check_send_cmd(ATcmd,AT_WIFI_START_SUCC,20);
  return flag;
}

bool check_send_cmd(const char* cmd,const char* resp,unsigned int timeout)
{
  int i = 0;
  unsigned long timeStart;
  timeStart = millis();
  cleanBuffer(ATbuffer,BUF_LEN);
  Serial3.print(cmd);
  Serial3.flush();
  while(1)
  {
    while(Serial3.available())
    {
      ATbuffer[i++] = Serial3.read();
      if(i >= BUF_LEN)i = 0;
    }
    if(NULL != strstr(ATbuffer,resp))break;
    if((unsigned long)(millis() - timeStart > timeout * 1000)) break;
  }
  
  if(NULL != strstr(ATbuffer,resp))return true;
  return false;
}

void cleanBuffer(char *buf,int len)
{
  for(int i = 0;i < len;i++)
  {
    buf[i] = '\0';
  } 
}

void BEEP(int b_time)
{
  for(int i = 1;i <= b_time;i++)
  { 
    digitalWrite(BuzzerPin,HIGH);
    delay(100);
    digitalWrite(BuzzerPin,LOW);
    delay(100);
  }
}
void Buzzer_mute()
{
  Buzzer_OFF;
  Buzzer = MUTE;
}
  
