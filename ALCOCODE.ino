#include <LCDI2C_Multilingual.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

Adafruit_HMC5883_Unified lv_m_sensor = Adafruit_HMC5883_Unified(12345);
LCDI2C_Generic lv_lcd(0x27, 20, 4); 

void setup() {
  pinMode(A1,INPUT);
  pinMode(34,OUTPUT);
  pinMode(33,OUTPUT);
  pinMode(36,OUTPUT);
  pinMode(37,OUTPUT);
  pinMode(A3,INPUT);
  pinMode(42,OUTPUT);
  pinMode(43,OUTPUT);
  pinMode(40,INPUT);
  pinMode(13,INPUT);
  pinMode(12,INPUT);
  pinMode(11,INPUT);
  pinMode(10,INPUT);
  pinMode(9,INPUT);
  Serial.begin(9600);
   if(!lv_m_sensor.begin()){
    Serial.println("\n\nsystem -> error : hm5883 [not detected]");
  }
  else{
    Serial.println("\n\nsystem -> hm5883 [detected]");
  }
  Serial2.begin(9600);
  lv_lcd.init();
  lv_lcd.backlight();
  delay(1000);
}

int lv_system_step=9;
int lv_count=0;
String lv_response="" ;
int lv_datacount=0;
int lv_readcount=0;
int lv_obstacleon=0;
float lv_batteryvoltage=0.0;
float lv_lowbat=22;
float lv_batterycharge=0;
float lv_batterypercentage=0;
String lv_batterystatus="high";
String lv_binstatus="not full";
String lv_obstaclestatus="none detected";
bool lv_error=false;
int lv_b1=0;
int lv_b2=0;
int lv_b3=0;
int lv_b4=0;
int lv_b5=0;
int lv_o1=0;
int lv_windownumber=0;
float g_heading=0;
float gdegref1=0;
float gdegref2=0;
float gdegref3=0;
long gdistance1=0;
long gdistance2=0;
float gdegdif=0;
float gtargetdeg=0;
long gdegtopulse=0;
long gmotorcurrentposition=0;
long gmotortargetposition=500;
int grunmode=0;
int gloopcount=0;
int gloopdone=0;
bool gauto=false;
String gstatus="ok";
int lv_binon=0;
int gstop=0;
int gstoptimeout=0;

void f_readserial(){
  while(Serial2.available()){
    Serial.println(Serial2.readString());
  }
}

void f_sendmsg(){
  f_readbatterystatus();
  Serial2.println("AT+CMGF=1");
  delay(1000);
  Serial2.println("AT+CMGS=\"+639386071771\"");
  f_readserial();
  delay(1000);
  Serial2.print("                  ALCo NOTICE\r\n");
  Serial2.print("Target Loop/s:"+String(gloopcount) + "\r\n");
  Serial2.print("Completed Loop/s:"+String(gloopdone) + "\r\n");
  Serial2.print("Battery Status:"+String(lv_batterystatus) + " (" + String((int)lv_batterypercentage) + "%)\r\n");
  Serial2.print("Bin Status:"+String(lv_binstatus)+"\r\n");
  Serial2.print("Note: Please turn off the rotary brush!!!\r\n");
  delay(100);
  Serial2.print((char)26);
  delay(100);
  f_readserial();
}

void f_m_sensor_read(){
  sensors_event_t lv_m_event;
      lv_m_sensor.getEvent(&lv_m_event);
      //Serial.print("\nsystem -> x["+String(lv_m_event.magnetic.x)+"],y["+String(lv_m_event.magnetic.y)+"],z["+String(lv_m_event.magnetic.z)+"]");
      float lv_heading=atan2(lv_m_event.magnetic.y,lv_m_event.magnetic.x);
      float lv_declinationangle=0.22;
      lv_heading+=lv_declinationangle;
      if(lv_heading<0){
        lv_heading+=2*PI;
      }
      if(lv_heading>2*PI){
        lv_heading-=2*PI;
      }
      float lv_headingdegrees=lv_heading*180/M_PI;
      g_heading=lv_headingdegrees;
      //Serial.print(",h["+String(lv_headingdegrees)+"]");
      Serial2.flush();
}

int getStableReading(int pin) {
  long total = 0;
  const int samples = 20; // number of samples to average
  for (int i = 0; i < samples; i++) {
    total += analogRead(pin);
    delay(10);
  }
  return total / samples;
}

void f_readbatterystatus(){
  lv_batteryvoltage=(analogRead(A3)/19.90990990990990)-6;
  lv_batterycharge=(lv_batteryvoltage)-21.6;
  lv_batterypercentage=(lv_batterycharge/4)*100;

    if (lv_batteryvoltage >= 26.0) {
    lv_batterypercentage = 100;
  } 
  else if (lv_batteryvoltage >= 24.5 && lv_batteryvoltage < 26.0) {
    lv_batterypercentage = 80;
  } 
  else if (lv_batteryvoltage >= 24.0 && lv_batteryvoltage < 24.5) {
    lv_batterypercentage = 60;
  } 
  else if (lv_batteryvoltage >= 23.0 && lv_batteryvoltage < 24.0) {
    lv_batterypercentage = 40;
  } 
  else if (lv_batteryvoltage >= 22.0 && lv_batteryvoltage < 23.0) {
    lv_batterypercentage = 20;
  } 
  else {
    lv_batterypercentage = 0;
  }
  // --- End of modification ---

  if(lv_batteryvoltage==24){
    lv_batterystatus="low";
    Serial.println("system -> battery[low]");
    lv_error=true;
  }else{
    lv_batterystatus="high";
    Serial.println("system -> battery[high]");
  }
  Serial.println(String("system -> "+String(lv_batteryvoltage))+"V "+String(lv_batterypercentage)+"% "+lv_batterystatus);
}

void f_selfcheck(){
  if(digitalRead(40)==0){
  lv_binstatus="full";
  Serial.println("system -> bin [full]");
  lv_error=true;
  }else{
    lv_binstatus="not full";
    Serial.println("system -> bin [not full]");
  }
  delay(1000);
  lv_batteryvoltage=analogRead(A3)/20.5;
  lv_batterycharge=lv_batteryvoltage-24;
  lv_batterypercentage=(lv_batterycharge/2)*100;
  if(lv_batteryvoltage<22){
    lv_batterystatus="low";
    Serial.println("system -> battery[low]");
    lv_error=true;
  }else{
    lv_batterystatus="high";
    Serial.println("system -> battery[high]");
  }
  Serial.println(String("system -> "+String(lv_batteryvoltage))+"V "+String(lv_batterypercentage)+"% "+lv_batterystatus);
  delay(1000);
  lv_obstacleon=digitalRead(A1);
  if(lv_obstacleon==1){
    Serial.println("system -> obstacle sensor [detected]");
    lv_obstaclestatus="detected";
    lv_error=true;
  }
  else{
    Serial.println("system -> obstacle sensor [none detected]");
    lv_obstaclestatus="none detected";
  }
  delay(1000);
  if(lv_error=true){
    f_notifyuser();
  }
}

void f_notifyuser(){
  Serial.println("system -> sim800l initialization");
  Serial2.println("ATE1");
  f_readserial();
  delay(1000);
  Serial2.println("AT+IPR=9600");
  delay(1000);
  f_readserial();
  delay(1000);
  Serial2.println("AT+CSQ");
  delay(1000);
  f_readserial();
  delay(1000);
  f_sendmsg();
  lv_count=lv_count+1;
  lv_system_step=9;
  Serial.println("system -> sms report [sent]");
  delay(2000);
}

void loop() {
  switch (lv_system_step){
    case 0:
      sensor_t lv_m_info;
      lv_m_sensor.getSensor(&lv_m_info);
      Serial.println("system -> sensor name :"+String(lv_m_info.name));
      Serial.println("system -> driver ver. :"+String(lv_m_info.version));
      Serial.println("system -> unique id :"+String(lv_m_info.sensor_id));
      Serial.println("system -> max value :"+String(lv_m_info.max_value));
      Serial.println("system -> min value :"+String(lv_m_info.min_value));
      Serial.println("system -> resolution :"+String(lv_m_info.resolution));
      lv_system_step=1;
    break;
    case 1:
      lv_lcd.clear();
      lv_lcd.setCursor(0,0);
      lv_lcd.print("PLEASE WAIT!!!");
      lv_lcd.setCursor(0,1);
      lv_lcd.print("SELF CHECK");
      f_selfcheck();
      lv_system_step=9;
    break;
    case 7:
      lv_error=false;
      lv_system_step=6;
      Serial.println("system -> self check [running]");
      delay(1000);
    break;
    case 9:
      digitalWrite(42,HIGH);
      digitalWrite(43,LOW);
      lv_b1=digitalRead(13);
      delay(10);
      lv_b2=digitalRead(12);
      delay(10);
      lv_b3=digitalRead(11);
      delay(10);
      lv_b4=digitalRead(10);
      delay(10);
      lv_b5=digitalRead(9);
      delay(10);
      lv_o1=digitalRead(A1);
      Serial.println(String(lv_b1)+String(lv_b2)+String(lv_b3)+String(lv_b4)+String(lv_b5)+String(lv_o1));
      if(lv_b1==1){
        lv_windownumber=lv_windownumber+1;
      }
      if(lv_windownumber>8){
          lv_windownumber=0;
        }
      if(lv_b5==1){
        lv_windownumber=lv_windownumber-1;
      }
      if(lv_windownumber<0){
          lv_windownumber=8;
        }
      if(lv_windownumber==1){
        lv_lcd.clear();
        lv_lcd.print("SET ALCO TO 1ST PATH");
        f_m_sensor_read();
        lv_lcd.setCursor(0,1);
        lv_lcd.print("FACING:"+String(g_heading));
        if(lv_b3==1){
          gdegref1=g_heading;
        }
        if(lv_b2==1){
          gdegref1=gdegref1+1;
        }
        if(lv_b4==1){
          gdegref1=gdegref1-1;
        }
        lv_lcd.setCursor(0,2);
        lv_lcd.print("SET:"+String(gdegref1));
        lv_lcd.setCursor(0,3);
        lv_lcd.print("B1:SET        B4:NXT");
      }
      if(lv_windownumber==2){
        lv_lcd.clear();
        lv_lcd.print("SET ALCO TO 2ND PATH");
        f_m_sensor_read();
        lv_lcd.setCursor(0,1);
        lv_lcd.print("FACING:"+String(g_heading));
        if(lv_b3==1){
          gdegref2=g_heading;
        }
        if(lv_b2==1){
          gdegref2=gdegref2+1;
        }
        if(lv_b4==1){
          gdegref2=gdegref2-1;
        }
        lv_lcd.setCursor(0,2);
        lv_lcd.print("SET:"+String(gdegref2));
        lv_lcd.setCursor(0,3);
        lv_lcd.print("B1:SET        B4:NXT");
      }
      if(lv_windownumber==3){
        lv_lcd.clear();
        lv_lcd.print("SET ALCO TO 3RD PATH");
        f_m_sensor_read();
        lv_lcd.setCursor(0,1);
        lv_lcd.print("FACING:"+String(g_heading));
        if(lv_b3==1){
          gdegref3=g_heading;
        }
        if(lv_b2==1){
          gdegref3=gdegref3+1;
        }
        if(lv_b4==1){
          gdegref3=gdegref3-1;
        }
        lv_lcd.setCursor(0,2);
        lv_lcd.print("SET:"+String(gdegref3));
        lv_lcd.setCursor(0,3);
        lv_lcd.print("B1:SET        B4:NXT");
      }
      if(lv_windownumber==4){
        lv_lcd.clear();
        lv_lcd.print("SET 1ST DISTANCE");
        if(lv_b2==1){
          gdistance1=gdistance1+50;
        }
        if(lv_b4==1){
          gdistance1=gdistance1-10;
        }
        lv_lcd.setCursor(0,1);
        lv_lcd.print("DISTANCE:"+String(gdistance1)+" CM");
        lv_lcd.setCursor(0,3);
        lv_lcd.print("B2:-CM B3:+CM B4:NXT");
      }
      if(lv_windownumber==5){
          lv_lcd.clear();
          lv_lcd.print("SET 2ND DISTANCE");
          if(lv_b2==1){
            gdistance2=gdistance2+50;
          }
          if(lv_b4==1){
            gdistance2=gdistance2-10;
          }
          lv_lcd.setCursor(0,1);
          lv_lcd.print("DISTANCE:"+String(gdistance2)+" CM");
          lv_lcd.setCursor(0,3);
          lv_lcd.print("B2:-CM B3:+CM B4:NXT");
        }
        if(lv_windownumber==6){
          lv_lcd.clear();
          lv_lcd.print("   VOLTAGE READING");
          f_readbatterystatus();
          lv_lcd.setCursor(0,1);
          lv_lcd.print("VOLTAGE:"+String(lv_batteryvoltage)+"V");
          lv_lcd.setCursor(0,3);
          lv_lcd.print("              B4:NXT");
          }
        if(lv_windownumber==7){
          if(lv_b3==1){
            gloopdone=0;
            gauto=true;
          }
          if(gloopdone==gloopcount && gauto==true){
            lv_lcd.clear();
            lv_lcd.setCursor(0, 0);
            lv_lcd.print("SELF CHECK");
            f_selfcheck();
            gauto=false;
          }
          else if(gauto==true){
            gloopdone=gloopdone+1;
            lv_system_step=4;
          }else{
            gauto=false;
          }
          if(lv_b2==1){
            gloopcount=gloopcount+5;
          }
          if(lv_b4==1){
            gloopcount=gloopcount-1;
          }
          if(lv_error==true){
            gstatus="ERROR";
          }else{
            gstatus="NO ERROR";
          }
          lv_lcd.clear();
          lv_lcd.print("SELECT TARGET LOOP");
          lv_lcd.setCursor(0,1);
          lv_lcd.print("TARGET LOOP:"+String(gloopcount));
          lv_lcd.setCursor(0,2);
          lv_lcd.print("B1:START      B4:NXT");
          lv_lcd.setCursor(0,3);
          lv_lcd.print("     B2:-  B3:+     ");
          }
        if(lv_windownumber==8){
          lv_obstacleon=digitalRead(A1);
          lv_binon=digitalRead(40);
          String obsStatus = (lv_obstacleon == 1) ? "DETECTED" : "NONE DETECTED";
          String binStatus = (lv_binon == 1) ? "NOT FULL" : "FULL";
          lv_lcd.clear();
          lv_lcd.setCursor(0,0);
          lv_lcd.print("     ALCO STATUS");
          lv_lcd.setCursor(0,1);
          lv_lcd.print("OBSTACLE:" + obsStatus);
          lv_lcd.setCursor(0,2);
          lv_lcd.print("BIN:" + binStatus);
           lv_lcd.setCursor(0,3);
          lv_lcd.print("              B4:NXT");

        }
        if(lv_windownumber==0){
          lv_lcd.clear();
          lv_lcd.print("     AUTOMATED");
          lv_lcd.setCursor(0,1);
          lv_lcd.print("   LEAF COLLECTOR");
          lv_lcd.setCursor(0,3);
          lv_lcd.print("              B4:NXT");
        }
      delay(1000);
    break;
    case 4:
    lv_lcd.clear();
    lv_lcd.setCursor(0, 0);
    lv_lcd.print("MOVING FORWARD");
    digitalWrite(33,LOW);
    digitalWrite(36,LOW);
    gmotorcurrentposition=0;
    if(grunmode==0){gmotortargetposition=gdistance1*125;}
    else if(grunmode==1){gmotortargetposition=gdistance2*125;}
    else if(grunmode==2){gmotortargetposition=gdistance1*125;}
      while (gmotorcurrentposition < gmotortargetposition) {
        lv_obstacleon = digitalRead(A1);
        lv_binon = digitalRead(40); // Check bin status inside loop

        if (lv_binon == 0 || lv_obstacleon == 1) {
  gstop = 1;
  if(lv_binon == 0) {
    lv_binstatus = "full";
    Serial.println("system -> bin full detected, notifying user...");
    if(!lv_error) { // notify only once
      lv_error = true;
      f_notifyuser();
    }
  }
  if(lv_obstacleon == 1){
    Serial.println("system -> obstacle detected, waiting...");
  }
  delay(5000); // short delay for loop
} else {
  gstop = 0;
}
       
        if(gstop==0){
          digitalWrite(34,HIGH);
          delayMicroseconds(500);
          digitalWrite(34,LOW);
          delayMicroseconds(5);
          digitalWrite(37,HIGH);
          delayMicroseconds(500);
          digitalWrite(37,LOW);
          delayMicroseconds(5);
          gmotorcurrentposition=gmotorcurrentposition+1;
          digitalWrite(42,LOW);
          digitalWrite(43,HIGH);
        }else{
          digitalWrite(42,HIGH);
          digitalWrite(43,LOW);
        }
      }
      if(grunmode==0){
        gtargetdeg=gdegref2;
        lv_system_step=10;
        }
      else if(grunmode==1){
        gtargetdeg=gdegref3;
        lv_system_step=10;
      }
      else if(grunmode==2){
        gtargetdeg=gdegref1;
        lv_system_step=10;
      }
      delay(1000);
    break;
    case 10:
    lv_lcd.clear();
    lv_lcd.setCursor(0, 0);
    lv_lcd.print("ROTATING");
    gmotorcurrentposition=0;
      f_m_sensor_read();
      gdegdif=g_heading-gtargetdeg;
      lv_lcd.setCursor(0, 1);
      lv_lcd.print("D:"+String(gdegdif));
      lv_lcd.setCursor(0, 2);
      lv_lcd.print("T:"+String(gtargetdeg)+"C:"+String(g_heading));
      if(gdegdif<0){
        digitalWrite(33,HIGH);
        digitalWrite(36,LOW);
      }else{
        digitalWrite(33,LOW);
        digitalWrite(36,HIGH);
      }
      gdegtopulse=abs(gdegdif*60);
      delay(1000);
      while(gmotorcurrentposition<gdegtopulse){
        digitalWrite(34,HIGH);
        delayMicroseconds(500);
        digitalWrite(34,LOW);
        delayMicroseconds(5);
        digitalWrite(37,HIGH);
        delayMicroseconds(500);
        digitalWrite(37,LOW);
        delayMicroseconds(5);
        gmotorcurrentposition=gmotorcurrentposition+1;
      }
      gmotorcurrentposition=0;
      delay(1000);
      f_m_sensor_read();
      gdegdif=g_heading-gtargetdeg;
      lv_lcd.clear();
      lv_lcd.setCursor(0, 0);
      lv_lcd.print("CHECKING");
      lv_lcd.setCursor(0, 1);
      lv_lcd.print("D:"+String(gdegdif));
      lv_lcd.setCursor(0, 2);
      lv_lcd.print("T:"+String(gtargetdeg)+"C:"+String(g_heading));
      if(gdegdif>-1 && gdegdif<1){
        if(grunmode==0){
          lv_system_step=4;
          grunmode=1;
          }
          else if(grunmode==1){
            lv_system_step=4;
            grunmode=2;
          }
          else if(grunmode==2){
            lv_system_step=9;
            grunmode=0;
          }
          else if(grunmode==20){
            lv_system_step=9;
            grunmode=0;
          }
      }else{
        lv_system_step=10;
      }
      delay(1000);
    break;
  }
}
