#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <math.h>

#define outpin 3 //PPM

#define deploy_judge_pin_INPUT1  10 //一段階目溶断の抜けピン
#define deploy_judge_pin_INPUT2  9 //二段階目溶断の抜けピン

#define LoRa_sw 6 //MOSFETのスイッチピン
#define LoRa_rst 18 //LoRaのリセット
#define LoRa_RX 17
#define LoRa_TX 19

#define SLEEP 0
#define TRAINING 1
#define STABILIZE_NOSEUP 2
#define STABILIZE 3
#define AUTO 4
#define DEEPSTALL 5

#define goal_latitude 35.6596325
#define goal_longtitude 140.0737739
#define goal_altitude 42.0
#define difference_lat 111316.2056

static const float difference_lon = cos(goal_latitude/180*M_PI)*M_PI*6378.137/180*1000;

SoftwareSerial LoRa(LoRa_RX,LoRa_TX); //LoRaと接続、PixhawkはSerialでつなぐ。

void setup()
{
    pinMode(outpin,OUTPUT);
    pinMode(deploy_judge_pin_INPUT1,INPUT_PULLUP);
    pinMode(deploy_judge_pin_INPUT2,INPUT_PULLUP);
    pinMode(LoRa_sw,OUTPUT);  //LoRaの通信on
    digitalWrite(LoRa_sw,HIGH);
    pinMode(LoRa_rst,OUTPUT);
    digitalWrite(LoRa_rst,HIGH);

    LoRa.begin(19200); //LoRaとの通信開始
    delay(2000);
}

void loop()
{
    LoRa.write("cutoff\r");
    delay(100);
    LoRa.write("config\r");
    delay(100);
    digitalWrite(LoRa_rst,LOW);
    delay(100);
    digitalWrite(LoRa_rst,HIGH);
    delay(2000);
    LoRa.write("2\r");
    delay(100);
    LoRa.write("g 0\r");
    delay(100);
    LoRa.write("q 2\r");
    delay(100);
    LoRa.write("save\r");
    delay(100);
    LoRa.write("start\r");
    delay(100);
    while(true){
        LoRa.write("hoge\r");
        delay(100);
    }
}
