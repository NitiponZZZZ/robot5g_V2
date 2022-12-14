// includes
#include <HardwareSerial.h>
#include <ODriveArduino.h>
#include <ArduinoJson.h>
#include "DHT.h"
#include <NewPing.h>
#define vMax  10
#define TRIGGER_PIN_F  39
#define ECHO_PIN_F     41
#define TRIGGER_PIN_R  43
#define ECHO_PIN_R     45
#define MAX_DISTANCE 500 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define DHTPIN 5
#define DHTPIN1 4
#define DHTTYPE DHT21

String readString;
uint8_t L1_index, R1_index, L2_index, R2_index, T_index;
float L1_value = 0.00, R1_value = 0.00, L2_value = 0.00, R2_value = 0.00;
float cal1, cal2, cal3, cal4, calc1, calc2, calc3, calc4, current;
float Temp_R, Humid_R, Temp_B, Humid_B;
long previousMillis = 0, previousMillis2 = 0, previousMillis3 = 0;
long interval = 250, interval2 = 500, interval3 = 500;
float cmps = 63.83716;
int Status_ = 0;
float wheelControl, a, b, c, d;

unsigned int Ultra_F, Ultra_R;
unsigned long count_Timer = millis();
unsigned long prevoid_Rtemp = 0;
unsigned long prevoid_ultraF = 0;
unsigned long prevoid_ultraR = 0;

bool State  = true;


HardwareSerial& odrive_serial1 = Serial1; ODriveArduino odrive1(odrive_serial1);
HardwareSerial& odrive_serial3 = Serial3; ODriveArduino odrive3(odrive_serial3);
StaticJsonDocument<230> doc_mini;
DHT dhtrobot(DHTPIN, DHTTYPE);
DHT dhtbat(DHTPIN1, DHTTYPE);

NewPing sonic_F(TRIGGER_PIN_F, ECHO_PIN_F, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonic_R(TRIGGER_PIN_R, ECHO_PIN_R, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

void setup() {

  Serial.begin(115200);
  Serial2.begin(115200);
  odrive_serial1.begin(115200);
  odrive_serial3.begin(115200);
  dhtrobot.begin();
  dhtbat.begin();
  while (!Serial1 && !Serial3 && !Serial);
  odrive1.SetVelocity(0, 0.00); odrive1.SetVelocity(1, 0.00);
  odrive3.SetVelocity(0, 0.00); odrive3.SetVelocity(1, 0.00);
  odrive1.run_state(0, 8, false); odrive1.run_state(1, 8, false);
  odrive3.run_state(0, 8, false); odrive3.run_state(1, 8, false);
}
void loop() {
  //test(1);
  readData();
  controlWheel();
  //  if (millis -  prevoid_ultraF > 33) {
  //    readDistanceF();
  //    prevoid_ultraF = millis();
  //  }
  //  if (millis -  prevoid_ultraR > 33) {
  //    readDistanceR();
  //    prevoid_ultraR = millis();
  //  }
  //  if (millis -  prevoid_Rtemp > 2000) {
  //    read_temp();
  //    prevoid_Rtemp = millis();
  //  }
  sendData ();


  //  if (currentMillis - previousMillis > 10) {
  //    previousMillis = currentMillis;
  //    sendData ();
  //  }
}
void readData() {
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '/' ) {

      L1_index = readString.indexOf('L');
      R1_index = readString.indexOf('R', L1_index + 1);
      L1_value = readString.substring(L1_index + 1, R1_index).toFloat();
      R1_value = readString.substring(R1_index + 1).toFloat();

      L2_index = readString.indexOf('L', R1_index + 1);
      R2_index = readString.indexOf('R', L2_index + 1);
      L2_value = readString.substring(L2_index + 1, R2_index).toFloat();
      R2_value = readString.substring(R2_index + 1).toFloat();

      calculateSpeed( L1_value, R1_value, L2_value, R2_value);
      readString = ""; // clear Value
    } else {
      readString += c;
    }
  }
}

void controlWheel() {
  cal1 = (cal1 > vMax) ? vMax : cal1; cal2 = (cal2 > vMax) ? vMax : cal2;
  cal3 = (cal3 > vMax) ? vMax : cal3; cal4 = (cal4 > vMax) ? vMax : cal4;
  if (cal1 < 0.1 && cal1 > -0.1 && cal2 < 0.1 && cal2 > -0.1 && cal3 < 0.1 && cal3 > -0.1 && cal4 < 0.1 && cal4 > -0.1 ) {
    odrive1.run_state(0, 8, false); odrive1.run_state(1, 8, false);
    odrive3.run_state(0, 8, false); odrive3.run_state(1, 8, false);
    odrive1.SetVelocity(0, 0, 0); odrive1.SetVelocity(1, 0, 0);
    odrive3.SetVelocity(0, 0, 0); odrive3.SetVelocity(1, 0, 0);
  }  
  else {
    odrive1.SetVelocity(0, (-1)*cal1);
    odrive1.SetVelocity(1, cal2);
    odrive3.SetVelocity(0, (-1)*cal3);
    odrive3.SetVelocity(1, cal4);
  }
}

void GetVelocity_ ()
{
  doc_mini["VFL"] = -1 * odrive1.GetVelocity(0);
  doc_mini["VFR"] = odrive1.GetVelocity(1);
  doc_mini["VRL"] = -1 * odrive3.GetVelocity(0);
  doc_mini["VRR"] = odrive3.GetVelocity(1);
  doc_mini["SR"] = ( odrive1.GetVelocity(0) + odrive1.GetVelocity(1) + odrive3.GetVelocity(0) + odrive3.GetVelocity(1)) / 4;

}

void GetPosition_ ()
{
  doc_mini["PRL"] = -1 * odrive3.GetPosition(0);
  doc_mini["PRR"] = odrive3.GetPosition(1);

}
float calculateSpeed(float val1, float val2, float val3, float val4) {
  /*
     circular = 2??r
    solution
      vel = 2*??*r
      1 r/s = 63.83716 cm/s
  */
  cal1 = val1 / (cmps / 100) ;
  cal2 = val2 / (cmps / 100) ;
  cal3 = val3 / (cmps / 100 );
  cal4 = val4 / (cmps / 100) ;
}

float calculateSpeedback(float val1, float val2, float val3, float val4) {
  /*
     circular = 2??r
    solution
      vel = 2*??*r
      1 r/s = 63.83716 cm/s
  */

  calc1 = val1 * cmps;
  calc2 = val2 * cmps;
  calc3 = val3 * cmps;
  calc4 = val4 * cmps;
}


void test(float dir) {
  odrive1.run_state(0, 8, false); odrive1.run_state(1, 8, false);
  odrive3.run_state(0, 8, false); odrive3.run_state(1, 8, false);
  odrive1.SetVelocity(0, -1 * dir);
  odrive1.SetVelocity(1, 1 * dir);
  odrive3.SetVelocity(0, -1 * dir);
  odrive3.SetVelocity(1, 1 * dir);
  Serial.println("test drive");
}

void sendData ()
{

  /*
    {
    "PRL":00.00,
    "PRR":00.00,
    "VFL":00.00,
    "VFR":00.00,
    "VRL":00.00,
    "VRR":00.00,
    "SR":00.00,
    "HR":00.00,
    "TR":00.00,
    "HB":00.00,
    "TB":00.00,
    "Seq":00.00
    }
  */
  GetPosition_ ();
  //  GetVelocity_ ();
  //  Odrive_Voltage ();
  //  //doc_mini["USR"] = Ultra_R;
  //  //doc_mini["USF"] = Ultra_F;
  //  doc_mini["HR"] = Humid_R;
  //  doc_mini["TR"] = Temp_R;
  //  doc_mini["HB"] = Humid_B;
  //  doc_mini["TB"] = Temp_B;
  doc_mini["Seq"] = random(0, 1000);


  serializeJson(doc_mini, Serial);
  Serial.println(" ");
}

void read_temp() {
  float h = dhtrobot.readHumidity();
  float t = dhtrobot.readTemperature();
  float h1 = dhtbat.readHumidity();
  float t1 = dhtbat.readTemperature();
  h = (h > 20) ? 0.00 : h;
  t = (t > 20) ? 0.00 : t;
  h1 = (h1 > 20) ? 0.00 : h1;
  t1 = (t1 > 20) ? 0.00 : t1;
  Temp_R = t;
  Humid_R = h;
  Temp_B = t1;
  Humid_B = h1;

}
void readDistanceF()
{
  int read_Ultra_F = sonic_F.ping_cm();
  //Serial.print(read_Ultra_F);
  if (read_Ultra_F > 0 )
  {
    Ultra_F = read_Ultra_F;
  }

}
void readDistanceR()
{
  int read_Ultra_R = sonic_R.ping_cm();
  //  Serial.print("\t");
  //  Serial.println(read_Ultra_R);
  if (read_Ultra_R > 0)
  {
    Ultra_R = read_Ultra_R;
  }

}


void Odrive_Voltage ()
{
  odrive_serial1.println("r vbus_voltage");
  float voltage1 =  odrive1.readFloat();
  doc_mini["VO1"] = voltage1;

  odrive_serial3.println("r vbus_voltage");
  float voltage2 =  odrive3.readFloat();
  doc_mini["VO2"] = voltage2;
}
