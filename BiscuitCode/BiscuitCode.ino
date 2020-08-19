// start
// initialise gps
// initialise lora
// initialise rtc
// setRtc variable is false till set by gps time
//
// loop
//    search for gps point
//          if point has come, then  save point.------------------------------- DONE
           // if time has not been set then set the time ----------------------
           // sleep gps -------------------------------------------------------
           // wake lora
           // send point 
           // sleep LoRa    
//    sleep for t seconds
//    wake up 
//   


#include <Wire.h>
#include <RTClibExtended.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
SoftwareSerial myGps(7, 8);
Adafruit_GPS GPS(&myGps);
#include <LowPower.h>

#define wakePin 2 
byte AlarmFlag = 0;
byte gpsSleep = 0;
byte timeAcc = 0;

String collarName = "GPS 2";
boolean rtc = false;
RTC_DS3231 RTC; 
void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("BEGIN");
  delay(1000);
  setupLoRa();
  Serial.println("LoRa Begun");
  
  setupGps();
  Serial.println("GPS Begun");
  
  setupRtc();
  Serial.println("RTC ready");
  Serial.println("Complete");
  delay(1000);
  //LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}
//--------------------------------------- SETUP GPS -----------------------------
void setupGps(){
  
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  digitalWrite(9,HIGH);
  digitalWrite(10,HIGH);
  
  GPS.begin(9600);
  delay(500);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  delay(50);
}
//--------------------------------------- SETUP LORA -----------------------------
void setupLoRa(){
  LoRa.setPins(6, 5); //----------------------- set LoRa (NSS, RST) pins
  
  if (!LoRa.begin(915E6)) {
    Serial.println("LoRa failed!");
    while (1);
  }
  
  // LoRa.setSpreadingFactor(12);
  // 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, and 250E3
  LoRa.sleep(); 
  LoRa.setSignalBandwidth(62.5E3);
  //LoRa.enableCrc();
 // LoRa.setSyncWord(0xF3);
  delay(50);
  LoRa.idle();

  LoRa.beginPacket();
  LoRa.print("GPS 1, 125");
  LoRa.endPacket();
  delay(50);
  
  LoRa.sleep();  
 
}
// ---------------------------------------- SETUP RTC -------------------------------
void setupRtc(){
  pinMode(wakePin, INPUT);

  //Initialize communication with the clock
  Wire.begin();
  RTC.begin();
  //RTC.adjust(DateTime(__DATE__, __TIME__));   //set RTC date and time to COMPILE time
  
  //clear any pending alarms
  RTC.armAlarm(1, false);
  RTC.clearAlarm(1);
  RTC.alarmInterrupt(1, false);
  RTC.armAlarm(2, false);
  RTC.clearAlarm(2);
  RTC.alarmInterrupt(2, false);

  //Set SQW pin to OFF (in my case it was set by default to 1Hz)
  //The output of the DS3231 INT pin is connected to this pin
  //It must be connected to arduino D2 pin for wake-up
  RTC.writeSqwPinMode(DS3231_OFF);

  //Set alarm1 every day at 18:33
  //RTC.setAlarm(ALM1_MATCH_HOURS, 33, 18, 0);   //set your wake-up time here
  //RTC.alarmInterrupt(1, true);
}
uint32_t timer = millis();
uint32_t timer2 = millis();

void loop() {
  // put your main code here, to run repeatedly:
  getPosition();
  delay(1000);
  Serial.println(".");
  //RTC.adjust(DateTime(GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds));
}

void systemSleep(){
  //set alarm
  
  GPS.standby();
  gpsSleep = 1;
  
  Serial.println("Sleep");
  delay(500);
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  delay(500);

}

void getPosition(){
  if(gpsSleep)
  {
    GPS.wakeup();
    Serial.print("wait location ... ");
      while (!GPS.available());
    gpsSleep = 0;
  }
  //GPS.sendCommand(1);
  GPS.read(); 
 
  if (GPS.newNMEAreceived()) {
   //parse
    if (!GPS.parse(GPS.lastNMEA()))  
      return;  
  }

  // if millis() or timer wraps around, reset it
  if (timer > millis())  timer = millis();

  // if millis() or timer2 wraps around, reset it
  if (timer2 > millis()) timer2 = millis();
  
  // if more that the wait time has passed then sleep for an interval
 // if (millis() - timer2 > waitTime) sleep_for_interval(pointInt);

  // approximately every intervalTime seconds or so, print out the current stats
  if (millis() - timer > 5000) {
    timer = millis(); // reset the timer

    
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    
    if (GPS.fix) {
      float myLat = (GPS.latitude - 100*int(GPS.latitude/100))/60.0;
      myLat += int(GPS.latitude/100);
      if(GPS.lat == 'S') myLat *= -1.0;
      
      float myLon = (GPS.longitude - 100*int(GPS.longitude/100))/60.0;
      myLon += int(GPS.longitude/100);
      if(GPS.lon == 'W') myLon *= -1.0;
      
      Serial.print("Location: ");
      Serial.print(myLat,6);
      Serial.print(", ");
      Serial.print(myLon,6);

      LoRa.idle();
      delay(100);
       LoRa.beginPacket();
       LoRa.print(collarName);
       LoRa.print(",");
       LoRa.print(myLat,6);
       LoRa.print(",");
       LoRa.print(myLon,6);
       LoRa.endPacket();
     LoRa.sleep();
       Serial.print("\nGPS Date: ");

       Serial.print(GPS.year, DEC);
       Serial.print("/");

       Serial.print(GPS.month, DEC);
       Serial.print("/");

       Serial.print(GPS.day, DEC);
       Serial.print(" Time: ");
          
       Serial.print(GPS.hour, DEC); 
       Serial.print(':');
         
          
       Serial.print(GPS.minute, DEC);
       Serial.print(':');
          
          
       Serial.print(GPS.seconds, DEC);
       Serial.println();

       if (!timeAcc){
          RTC.adjust(DateTime(GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds));
          timeAcc = 1;
       }

       //Sleep for interval t       
      // systemSleep();
       //timeAccurate = true;  
      }
    }
 // if (timeAccurate) { Check_To_Sleep(); }
}
