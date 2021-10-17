//signal field ,check sleep negative,green 


//const char strJimakoskx[] PROGMEM = "jimakoskx@all";//ser.println((__FlashStringHelper*)strJimakoskx);}

#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>//you must change the private aceess of dht._type to public in order to compile
#include <LowPower.h>///LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 



/**
   using sim commands
   //"AT";"OK";"AT+CSQ";"AT+CBC";"ATD+ +";"ATH";"AT+CMGF=1";"AT+CMGS=\""; //+phone +\" + new line+26||27(escape)
   new version allow php/ThingSpeak
   Email not possible because of SPAM killers...etc
   but Thanks to ThingsSpeak https://thingspeak.com/
   and Zygi.com https://zygi.gr/el/install/
   and php lan+gauge
   we can easilly upload data on the web
   //"AT+SAPBR=3,1,\"Contype\",\"GPRS\"";"AT+SAPBR=3,1,\"APN\",\"";//+apn+\" //example AT+SAPBR=3,1,"APN","internet.vodafone.gr"
   //"AT+SAPBR=1,1";"AT+SAPBR=2,1";//this is to test if we get IP
   //"AT+HTTPINIT";"AT+HTTPPARA=\"CID\",1";//AT+HTTPPARA="CID",1
   //"AT+HTTPPARA=\"URL\",\"";//+url+\" //example AT+HTTPPARA="URL","http://miliohm.com/miliohmSIM800L.php"
   //AT+HTTPACTION max responce is ~5 seconds if normal query
   //"AT+HTTPACTION=0";//GET action
   //"AT+HTTPREAD";//read what get

  //designed for Arduino Pro Mini with A6,A7 pins
  //Hx711 normal operation < 1.5mA, power down < 1uA ,Operation supply voltage range: 2.6 ~ 5.5V
  //DHT11 Measure 0.3mA Standby 60μA ,3 to 5V,,, time data, to be read twice in a row, but not recommended repeatedly r ead sensors, each sensor
  reading interval of more than 5 seconds to obtain accurate data.

  SIM800L typical power consumption in sleep mode is 0.7 mA
  ---------------
  Lets compute for a normal operation of 6 readings per day (every 4 hours)
  and lets assume that a reading/send operation wants 1 minute(1.5 if we have sms also)
  (If signal is ok ..then we need about 30 seconds for sending data to ThingSpeak..)

  case a,Latch Sensor relay ,best solution
  case b,Simple Sensor relay like RAYEX ELEC RS-05 (tested ,EMF can be easilly managed)
    that has coil 167 Ω so typical (in 5 volts-but we run at 4) has 30 mA consumption
    So in 60 readings (opening relay for 1 minute) we will burn 30mA
    That is 30 mA per 10 days (TEN DAYS) !!!!!!
  case c , No sensor relay at all
    then just SIM800 will burn (minimum) 0.7*24 hours=16 mA per day
    adding a little more (just for be sure) we have 20 mA per 1 day !!!...so having relay is better
    -------------------
  That is why we need some sensors relay especially if we are going to operate with some readings per day.

  But if someone wants to operate lets say every 5 minutes then
  it is better (beyond the best choice of having LATCH RELAY)... to NOT have normal relay(non latching).
  Because every 5 minutes means 12 readings per 1 hour , 144 readings(relay opening) per day.
  And 144 openings per day mean about 72 mA per Day.....So it is best to have SIM on sleep mode burning only 16mA per 1 day !



  //Hx711 needs 1.5 mA *5 =7.5    +0.3mA for Dht      =7.8mA < 40 mA than Arduino Pin can give
  //But in order to have only one relay i will give this 7.8mA to sim relay
  //So..only one relay for sim800l ,,,, 5 load sensors and dht
  //have in mind that if no use latch relays or if use only one latch relay we can have 6 scales.....on a pro mini !!!!
  //Dht will be disabled if we use ALL FIVE SCALES because we dont have pins in my 2 LATCH version....
  //If indeed have 5 load scales ...then we are big honey producers and not weather/scientific station !!!

   http://www.gammon.com.au/power
  Below 1.8V - cannot operate
  Below 4 MHz - use at least 1.8V
  Between 4 MHz and 10 MHz you require: Volts = 1.8 + ((M - 4) * 0.15)
  Between 10 Mhz and 20 MHz you require: Volts = 2.7 + ((M - 10) * 0.18)
  You can supply an absolute maximum of 5.5V
  From 1.8V to 2.7V: M = 4 + ((V - 1.8) / 0.15)
  From 2.7V to 4.5V: M = 10 + ((V - 2.7) / 0.18)

  For 16Mhz ->Volts = 2.7 +(6*0.18)= 3.78 =3.8 Volts Minimum
  At 4.0 working also the cheap relays TQ2-L2-5V PANASONIC(min 3.75) ,Axicom  V23079B1201B301 (min 3.75)

  So if want 3.5 (to be able to run SIM800L at lowest plus some ) we need run BELOW Mhz = 10+0.8/0.18=14.445 so we need 3.3 Version of pro mini
  But the problem is that in 3.3 a)We cant? run LCD b)I cant find relay that can operate without transistor.(Meaning 3 Volt relays usually wants more than 40mA to trigger)
  Omron(Latching) G6AK-274P-ST-US4.5 (20x10x9)has  112 coil resistance and can be triggered at 70% *4.5= 3.15 Volts but not possible to find  this on retail.and also dangerous if apply 4.5 volt because 4.5/112 =41 mA...but safe if 4.4
  Omron(Latching) G6AK-274P-ST-US5 has  139 coil resistance and can be triggered at 70% *5=3.5 !!! on the limit (in case we want esp32 that has max=3.6)
  And the cheap Zettler AZ850P2–5 (14x9x5) has 125 coil resistance that is on the limit of 40mA if apply 5 Volt (5/125=40) but safe if apply less .So 3.5/125=28 mA!!! good !!

  Conclusion that with G6AK-274P-ST-US5 we can regulate at 3.55(i choose 4 volts) that is
  a)enough for sim800l
  b)low enough for arduino to not loose power
  c)easy to operate also with esp32(in case someone wants esp32)
  d)Can operate safelly also at maximum of 10 Volts .
  But....in the case of LCD with I2C we need more volts
  ..at least 4...with potensiometer of i2c full.....or just have one more Pololu regulator CHIPE 5 euros ,special for this .

  Note that 3.3 pro mini version has 4.8(5) volts on Vcc when power from usb ...meaning in case of debugging we dont have problem to feed lcd from vcc.

  But i will use 8Mhz Pro mini powered at 4.0 that is ok also for lcd.and with 5 volt relays.!!!!!
  And of-cource eliminate the led from Arduino!!!
*/









#define LATCH_SENSORS_RELAY
#define LATCH_MAIN_RELAY



//#define DEBUG
//#define DEBUGFULL //lower priority showings?!?...
//#define DEBUG_SIM_ANSWER /showing what sim answering on varius commands



//#define USE_DEFINED_MILLIS_PER_READING //uncomment this to debug quicker using the below value
unsigned long millisPerReading = 120000ul ; //2 minutes

byte currMenu = 1; //=1 on release to be quicker for just when changing battery(or every time BEFORE reopen to SMS mode) and not quick for entering password and make adjustments
const byte menuCount = 16;//update this if you add more menus
bool menuUnlocked = 0; //0 on release allows menu_1 even if locked for quick change batteries and adjust the delay hours....


//find your own address if not 0x27
LiquidCrystal_I2C lcd(0x27, 16, 2);//i need a minimum 16 columns and 2 rows lcd.
String lcd0;
String lcd1;
void lcdEmptyStrings01() {
  lcd0.remove(0, lcd0.length());
  lcd1.remove(0, lcd1.length());
}
void lcdClearToStrings01() {
  lcd.clear();
  lcd.print(lcd0);
  lcd.setCursor(0, 1);
  lcd.print(lcd1);
}
void lcdQuickPrint(const char str[], unsigned long millToDelay) {
  lcd.clear();
  lcd.print(str);
  delay(millToDelay);
}



bool ModeLcd;//LCD MODE(administration) vs SMS MODE(running basic purpose)



//Pin 13 (led) may cant go on some DT because has resistor and with the pullup resistor will create divider....
//Basically i wanted 13 to be to MainRelayPowerOff pin but this is not possible due to the fact that default bootloader uses 13
//on startup so the relay will be triggered and even if the solution is just to hold poer On button a little more
//i dont like it (streching the power latch relay
//And even if i can change the bootloader ...i also dont like it for open to public version.
//But if you like ,,,change the bootloader and assign 13 to (best solution) PinRelayMainPowerRESET
//and just having a small switch for debugging cases ....






//5 scales
const uint8_t PinHxDt[] = {11, A2, 6, 2, 8}; //INPUT_PULLUP
const uint8_t PinHxSck[] = {10, 13, 5, 3, 9}; //OUTPUT (13 is problem in case we want power down on Hx711 SCL)

const uint8_t PinDht = 8; //(,or 9 but better 8 because is pulled up from scale!?! )only if 5th scale not working !!!! automatically by library INPUT_PULLUP & OUTPUT

//WARNING : green sim800l module(the smallest/nicest and without the fucking capacitor that needs remove (or a trick or one capacitor in our circuit) to not reset my arduino )
// has REVERSED its Tx Rx pins....meaning Grnd,Rx,Tx,Rst,Vcc ....but RED module has Grnd,Tx,Rx,Rst,Vcc,Antenna !!!!!!!!! SOS SOS SOS
//so...you may reverse the assigned numbers below at PinSimTx,PinSimRx ...!!!!!!!!!!!!!!!!!!!!!
//t7,R4 Red ,,,,, t4,r7 green
uint8_t PinSimTx = 7; // our Rx ,SoftwareSerial INPUT_PULLUP
uint8_t PinSimRx = 4; // our Tx ,SoftwareSerial OUTPUT
const uint8_t PinRelaySensorsSet = A1;  
const uint8_t PinRelaySensorsRESET = A0; 

const uint8_t PinButMenuOk = 12; // INPUT_PULLUP.Pressing this button on power on enter as sms mode.else lcd mode for adminlcd switch must be open ion this case).
const uint8_t PinButRollEdit = A7; //A7 external pulled up





#ifdef LATCH_MAIN_RELAY
const uint8_t PinRelayMainPowerRESET = A3; // OUTPUT  for the case of LATCH MAIN RELAY
const uint8_t PinPowerRead = A6; // INPUT for reading battery for the case of LATCH MAIN RELAY
int powerAnalogReading;
int powerReadingTranslated0;
byte continiusllyTranslatedBelowThreshold = 0;
#endif


void blinc(int times = 1) {
  digitalWrite(13, HIGH);
  delay(10);
  digitalWrite(13, LOW);
  --times;
  while (times > 0) {
    delay(490);
    digitalWrite(13, HIGH);
    delay(10);
    digitalWrite(13, LOW);
    --times;


  }
}



#ifdef DEBUG
SoftwareSerial ser(0, 1); //saving 106 Flush bytes instead of use Serial object (by the way that we must use SoftwareSerial for sim800l)
#endif

void setPinModes() {
  pinMode(PinButMenuOk, INPUT_PULLUP);

  resetPinModes();


  pinMode(PinRelaySensorsSet, OUTPUT);
  pinMode(PinRelaySensorsRESET, OUTPUT);

#ifdef LATCH_MAIN_RELAY
  pinMode(PinRelayMainPowerRESET, OUTPUT);
#endif
}




const byte millisForLatch = 25; //more than enough for omron(5 ms),zettlet(2 ms),panasonic (4 ms (without reset diode!!!)),


//this is for the RED module that has a big capacitor and provokes reseting if not have this trick
//The green module (the smallest) that has REVERSE Tx and Rx pins has no problem
//and we can skip this 3000 delay...but i am leaving this because .....i thing must be initial HIGH in any case for communication
/*
   But i finally removed this capacitor with a plier !!..and all working well...at least for me
  void avoidResetBySimByLoadingSimCapacitorViaComPins() {
  pinMode(PinSimTx, OUTPUT);
  //pinMode(PinSimRx, OUTPUT);//already output
  delay(10);
  digitalWrite(PinSimTx, HIGH); //1)1st line of SALVATION CODE
  digitalWrite(PinSimRx, HIGH); //2)2nd line of SALVATION CODE
  //3)3rd line of SALVATION CODE
  delay(3000);//just for SIM VCC to clamp at 2.24 volt !!! critical for when relay triggered(1000 is ok ...but...)

  }
*/


void relaySensorsTrigger() {

  //avoidResetBySimByLoadingSimCapacitorViaComPins();


#ifndef LATCH_SENSORS_RELAY
  digitalWrite(PinRelaySensorsSet, HIGH);
#endif

#ifdef LATCH_SENSORS_RELAY  //now saving power !!!
  digitalWrite(PinRelaySensorsSet, HIGH);
  delay(millisForLatch);
  digitalWrite(PinRelaySensorsSet, LOW);
#endif

  //delay(100);
  //pinMode(PinSimTx, INPUT_PULLUP);//back to pullup ..just in case for not using resetPinModes()

}





void relaySensorsUnTrigger() {
#ifndef LATCH_SENSORS_RELAY
  digitalWrite(PinRelaySensorsSet, LOW);
#endif

#ifdef LATCH_SENSORS_RELAY  //now saved ,, power !!!
  digitalWrite(PinRelaySensorsRESET, HIGH);
  delay(millisForLatch);
  digitalWrite(PinRelaySensorsRESET, LOW);
#endif
}


#ifdef LATCH_MAIN_RELAY
void relayMain_POWER_OFF() {
  digitalWrite(PinRelayMainPowerRESET, HIGH);
  delay(millisForLatch);//must not go here if switch for main reset allows !!!
  digitalWrite(PinRelayMainPowerRESET, LOW);///but having this in case we have the switch off...!!!!for debugging!
}
#endif


//DONT PRINT FLOATS TO SAVE FLASH memory !!!
DHT dht(PinDht, 11);
float t , h ;
void dhtRead() {
  t = dht.readTemperature();
  h = dht.readHumidity();
}





////////////////////////
////////////////////////
////////////////////////
// MAIN STORAGE LOGIC
byte iter = 0; //the currentposition in whitch i am storing.On iter 0 i am sending the daily sms
const byte ScalesCanHandle = 5;
const byte SensationsPerSms = 4;//4 readings before send sms.If sms length>150...could adjust this according free sram but now we are limited
const byte StoredWeightsLength = ScalesCanHandle * SensationsPerSms * 2;
long StoredWeights[ StoredWeightsLength ] ;//long is ok for ThingSpeak but in sms better send as divided by 100 or at least 10 to save some chars in case using 5 scales
char temperature[SensationsPerSms];
char humidity[SensationsPerSms];

const char DISABLED_CHAR = -2;
const char ERROR_CHAR = -1;
void clearStoredWeights() {
  int i = 0;
  while (i < StoredWeightsLength) {
    StoredWeights[i] = DISABLED_CHAR;
    ++i;
  }
}

//using sram on Callibration
int iTemp32, iTemp33, iTemp34;
long tempStandart[8];
long tempAverage[8];
void sortInsertionTempStandartsAndAveragesByStandarts(int RequestedStandarts) {
  int i,  j;
  long key, followKey;
  for (i = 1; i < RequestedStandarts; i++) {
    key = tempStandart[i];
    followKey = tempAverage[i];
    j = i - 1;
    while (j >= 0 && tempStandart[j] > key) {
      tempStandart[j + 1] = tempStandart[j];
      tempAverage[j + 1] = tempAverage[j];
      j = j - 1;
    }
    tempStandart[j + 1] = key;
    tempAverage[j + 1] = followKey;
  }
}


////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
//WEIGHT LOAD SCALE SENSOR XH711 LOGIC
//////////////////////////////////////
//AVOIDING use library for Flash economy saving 146 FLASH +73 SRAM bytes and i copied the algorithm from https://github.com/bogde/HX711
/*#include <HX711.h>
  HX711 scale0;
  HX711 scale1;
  HX711 scale2;
  HX711 scale3;
  HX711 scale4;
*/

long lastReading;//what we have read from HX711 weight sensor
//SOS not float...loosing FLASH...use float on translating
long lastGrams; //LONG register for translating the reading into Grams or whatever...Unit
byte currScale;
//////////////////////////////////////


//pulses=gain , 1=ChannelA@128 , 2=ChannelB@32 , 3=ChannelA@64
bool readWeight(byte DOUT, byte PD_SCK) {
  lastReading = 0;
  lastGrams = ERROR_CHAR;
  uint8_t filler = 0;

  while (digitalRead(DOUT) == HIGH) {
    ++filler;//waiting xh711 for signal
    delay(1);
    if (filler > 250) { //if no signal after 250 millis....go away
      //ser.println("error on reading scale (HX711)");
      return false;
    }
  }


  //filler = 0x00;//normal use of this variable...
  noInterrupts();
  // Pulse the clock pin 24 times to read the data.
  uint8_t data2 = shiftIn(DOUT, PD_SCK, MSBFIRST);
  uint8_t data1 = shiftIn(DOUT, PD_SCK, MSBFIRST);
  uint8_t data0 = shiftIn(DOUT, PD_SCK, MSBFIRST);

  //reading mode normal 128 amplification,only one pulse (2 for 32 Channel B and 3 for 64 Channel A)
  digitalWrite(PD_SCK, HIGH);//25 th pulse
  digitalWrite(PD_SCK, LOW);


  //////back to interrupts
  interrupts();
  /*
     https://cdn.sparkfun.com/datasheets/Sensors/ForceFlex/hx711_english.pdf
     The output 24 bits of data is in 2’s complement
      format. When input differential signal goes out of
      the 24 bit range, the output data will be saturated
      at 800000h (MIN) or 7FFFFFh (MAX), until the
      input signal comes back to the input range
      8388608
  */
  // Replicate the most significant bit to pad out a 32-bit signed integer
  if (data2 & 0x80) {
    filler = 0xFF;
  } else {
    filler = 0x00;
  }
  // Construct a 32-bit signed integer
  unsigned long value = ( static_cast<unsigned long>(filler) << 24
                          | static_cast<unsigned long>(data2) << 16
                          | static_cast<unsigned long>(data1) << 8
                          | static_cast<unsigned long>(data0) );
  //////////////////////////////////////
  lastReading = static_cast<long>(value);

  lastGrams = lastReading;
  return lastReading != 0 ; //most likelly must not be zero
}


/*
   TRANSLATING HX711 readings to GRAMS or Kg or whatever using Callibration values
*/
void translateByScale(byte scale, bool useTareAlso) {
  //ser.print(F("translateByScale :  "));ser.println(scale);

  byte LoadsUsing = romReadScaleStandartsUsing(scale);
  //if (LoadsUsing < 2) {//ser.println(F("Not callibrated ,leaving new users to callibrate and understand hx711"));}
  if (LoadsUsing > 1) { //1st is for zero average and we need at least 1 more average to calculate



    byte i = LoadsUsing - 1;
    long av = romReadScaleAverage(scale, i);
    long avpreOrForZero;
    //ser.print(F("last average "));ser.println(av);
    if (lastReading < av) {
      i = 1;
      while (i < LoadsUsing) {
        av = romReadScaleAverage(scale, i);
        //ser.println(i);ser.print(F("average "));ser.println(av);
        if (av < lastReading) {
          ++i;
        }
        else { //ser.print(F("breaking "));ser.println(i);
          break;
        }
      }
      if (i > 1) { //ser.print(F("Must decide to choose between i-1 and i "));
        avpreOrForZero = romReadScaleAverage(scale, i - 1);
        if (  lastReading - avpreOrForZero  <   av - lastReading   ) {
          av = avpreOrForZero;
          i--;
          //ser.print(F("Much closer to previus"));
        }
        //ser.println(i);
      }
      //else{ser.println(F("Using first"));}
    }
    //else{ser.println(F("Using last"));}
    //ser.print(i);ser.print(F(" FINAL av "));ser.println(av);


    //Now i must be in proper position
    avpreOrForZero = romReadScaleAverage(scale, 0);
    //x=(Rcurr-avZero) *( W /  [R1-avZero]   )
    float floatLastGrams = lastReading;
    floatLastGrams -= avpreOrForZero;
    floatLastGrams *= romReadScaleStandart(scale, i); //NOT IN PREVIUS LINE to NOT OVERFLOW...
    floatLastGrams /= (av - avpreOrForZero);
    if (useTareAlso) {
      floatLastGrams -= romReadScaleGramsDif(scale);
    }
    lastGrams = (long)floatLastGrams;
  }

}

bool readWeightOld(byte scale, bool useTareAlso, bool translate) {
  bool out = readWeight(PinHxDt[scale], PinHxSck[scale]);
  if (out && translate) {
    translateByScale(scale, useTareAlso);
  }
  return out;
}
bool readWeight(byte scale, bool useTareAlso, bool translate,byte times=1) {
  bool out = readWeight(PinHxDt[scale], PinHxSck[scale]);
  if (out) {
    //first time succesfully reads
    if(times>1){
      ///take average.....to lastReading

      float sums=lastReading;
      float divider=1;
      while(times>1){
        delay(10);
        out = readWeight(PinHxDt[scale], PinHxSck[scale]);//may override the final return....here
        if(out){
          divider+=1;
          sums+=lastReading;
          #ifdef DEBUG
          ser.println(lastReading);
          #endif
        }
        //else not check the failures....i am boring
        --times;
      }

      sums/=divider;
      lastReading=sums;
      
    }
    if(translate){
      translateByScale(scale, useTareAlso);
    }
  }
  return out;
}
bool readCurrScaleWeightOnly() {
  return readWeight(currScale, false, false);
}

////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////





////////////////////////
////////////////////////
////////////////////////



void romReadToStringTrimmed(int address, int addressEnd, const String & output) {
  char chNotSpace;
  while (address < addressEnd) {
    chNotSpace = (char)EEPROM.read(address);
    if (chNotSpace != ' ') {
      output += chNotSpace;
    }
    ++address;
  }
}
void romReadToString(int address, int addressEnd, const String & output) {
  while (address < addressEnd) {
    output += (char)EEPROM.read(address);
    ++address;
  }
}
void romUpdateFromString(int address, int addressEnd, const String & input, int inputIndex, bool fillWithSpaces) {
  int len = input.length();
  int adL = addressEnd;
  int adDif = adL - address;
  if (len < adDif) {
    adL = address + len;
  }
  while (address < adL) {
    EEPROM.update(address, (byte)input.charAt(inputIndex));
    ++address;
    ++inputIndex;
  }
  if (fillWithSpaces) {
    while (address < addressEnd) {
      EEPROM.update(address, 32);
      ++address;
    }
  }
}
long romReadLong(int address) {//saving some Flash not using EEPROM.put,get
  byte b = EEPROM.read(address); ++address;
  long out = b;  out <<= 8;
  b = EEPROM.read(address); ++address;
  out += b;      out <<= 8;
  b = EEPROM.read(address); ++address;
  out += b;      out <<= 8;
  out += EEPROM.read(address);
  return out;
}
void romUpdateLong(int address, long out) {
  address += 3;
  EEPROM.update(address, out); --address;
  out >>= 8;
  EEPROM.update(address, out); --address;
  out >>= 8;
  EEPROM.update(address, out); --address;
  out >>= 8;
  EEPROM.update(address, out);
}
//[-32768,32767]
int romReadInt(int address) {
  byte b = EEPROM.read(address); ++address;
  int out = b;  out <<= 8;
  b = EEPROM.read(address); ++address;
  out += b;
  return out;
}
void romUpdateInt(int address, int out) {
  address += 1;
  EEPROM.update(address, out); --address;
  out >>= 8;
  EEPROM.update(address, out); --address;
}



// A D D R E S S S = 0
const byte programID = 231;
byte romReadProgramId() {//some kind of insurance to default know if must flash EEPROM (on new installations)
  return EEPROM.read(0);
}
//void romUpdateProgramId(){EEPROM.update(0,programID);}
//bool checkProgramId(){return romReadProgramId()==programID;}
// A D D R E S S S = [1,5)
void romReadSimPin(const String & output) {//4 digits pin is also the password for program
  romReadToString(1, 5, output);
}
void romUpdateSimPin(const String & input, int inputIndex) {
  romUpdateFromString(1, 5, input, inputIndex, false);
}
// A D D R E S S S = 5
byte romReadPinEnabled() {//have this disabled when test but most propably enabled (from your phone) before put sim on SIM800L
  return EEPROM.read(5);
}
void romUpdatePinEnabled(byte val) {
  EEPROM.update(5, val);
}
// A D D R E S S S = 6
byte romReadSimSecWaitAfterPowerOn() {//old version using this but know i am reading sim answers for registering to network....most propably!
  return EEPROM.read(6);
}
void romUpdateSimSecWaitAfterPowerOn(byte SecondsSimDelayWaitingNetwork) {
  EEPROM.update(6 , SecondsSimDelayWaitingNetwork);
}
// A D D R E S S S = 7
byte romReadSimSecWaitBeforePowerOff() {//this kind of using but not so important
  return EEPROM.read(7);
}
void romUpdateSimSecWaitBeforePowerOff(byte SecondsSimDelayBeforePowerOff) {
  EEPROM.update(7 , SecondsSimDelayBeforePowerOff);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

// A D D R E S S S = 8
byte romReadSmsEnabled() {//0 disabled,1 primary only ,2 secondary only,3 primary+secondary
  return EEPROM.read(8);
}
void romUpdateSmsEnabled(byte val) {
  EEPROM.update(8, val);
}
// A D D R E S S S = [9,21)
void romReadSmsPrimaryNumber(const String & output) {//the phone number sending the sms
  romReadToString(9, 21, output);
}
void romUpdateSmsPrimaryNumber(const String & input, int inputIndex) {
  romUpdateFromString(9, 21, input, inputIndex, false);
}
// A D D R E S S S = [21,33)
void romReadSmsSecondaryNumber(const String & output) {//secondary phone number sending the sms
  romReadToString(21, 33, output);
}
void romUpdateSmsSecondaryNumber(const String & input, int inputIndex) {
  romUpdateFromString(21, 33, input, inputIndex, false);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

// A D D R E S S S = 33
byte romReadDhtType() {//11,12,21,22,,,just in case testing with chip dht11 and in future want expensive and accurate dht22 (5 euros)
  return EEPROM.read(33);
}
void romUpdateDhtType(byte DhtType) {
  EEPROM.update(33 , DhtType);
}
// A D D R E S S S = 34
byte romReadDhtEnabled() {//better disable this if you dont put dht
  return EEPROM.read(34);
}
void romUpdateDhtEnabled(byte DhtEnabled) {
  EEPROM.update(34 , DhtEnabled);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////


// A D D R E S S S = 35
byte romReadHourDelayBeforeSMSwhile() {//this is for as much possible to adjust the readings IN the day
  return EEPROM.read(35);//example.I am on field 8:00.If normal then i will be taking readings 8,14,20,2
  //but 20:00 may summer some bees are still out...(lets say) and may want 10,16,22,4
  //so i can quickly put 2 hours delay ....about...
}
void romUpdateHourDelayBeforeSMSwhile(byte HourDelay) {
  EEPROM.update(35 , HourDelay);
}
// A D D R E S S S = 36
byte romReadHoursPerReading() {
  return EEPROM.read(36);
}
void romUpdateHoursPerReading(byte HoursPerDay) {
  EEPROM.update(36 , HoursPerDay);
}

// A D D R E S S S = 37
byte romReadCallInsteadSms() {//0 means disabled..1 means call on each reading(if php on and not sending sms....just added this in the end)
  return EEPROM.read(37);
}
void romUpdateCallInsteadSms(byte CallOn) {
  EEPROM.update(37 , CallOn);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// A D D R E S S S = 38
byte romReadPhpEnabled() {//new version use ThingSpeak but may also be used in your own php somewhere in net
  return EEPROM.read(38);
}
void romUpdatePhpEnabled(byte val) {
  EEPROM.update(38, val);
}

//ThingSpeakKey[30],Apn[30],PhpServer[30],PhpQuery[30]
//total=120 chars must be available on EEPROM
//Note 1)php server NO NEED to include final '/'
//Note 2)php server not working on httpS (ssl) on sim800L.But working if you have it simple as http even if real server is https
//Note 3)I am not supporting username and password for APN....most APN have empty those
//Note 4)In thingspeak server fields following update? have names 'field1= up to 'field8='. I am not supporting changing those names ...
//in case thingspeak change or build your own php server.Could make this also for 'api_key=' ...but anyway....now 'api_key=' is editable....
//Note 5)I am supporting editing anly in small letters....should work fine also for api_key.Because will be more hard to roll on +26 ab chars with a simple button...
//So when on lcd rolling and reach 'A' i am going to 'a'
//post example
//https://api.thingspeak.com/update?api_key=7I2NQSK2L0AQEW44&field1=0
//must be WITHOUT S !!!
//http://api.thingspeak.com/update?api_key=7I2NQSK2L0AQEW44&field1=0
void romUpdateSpaceToEmailFields() {
  int address = 39;
  int addressEnd = 159;
  while (address < addressEnd) {
    EEPROM.update(address, 32); //' '=32 Space
    ++address;
  }
}

///////////////////////////////////////////////////
///////////////////////////////////////////////////

void romReadThingSpeakKey(const String & output) {//api_key=7I2NQSK2L0AQEW44
  romReadToStringTrimmed(39, 69, output);
}
void romUpdateThingSpeakKey(const String & input, int inputIndex) {
  romUpdateFromString(39, 69, input, inputIndex, true);
}
////////////////////////////////
void romReadApn(const String & output) {//internet.vodafone.gr
  romReadToStringTrimmed(70, 100, output);
}
void romUpdateApn(const String & input, int inputIndex) {
  romUpdateFromString(70, 100, input, inputIndex, true);
}
////////////////////////////////
void romReadPhpServer(const String & output) {//http://api.thingspeak.com
  romReadToStringTrimmed(101, 131, output);
}
void romUpdatePhpServer(const String & input, int inputIndex) {
  romUpdateFromString(101, 131, input, inputIndex, true);
}
////////////////////////////////
void romReadPhpQuery(const String & output) {//update?
  romReadToStringTrimmed(132, 162, output);
}
void romUpdatePhpQuery(const String & input, int inputIndex) {
  romUpdateFromString(132, 162, input, inputIndex, true);
}

// A D D R E S S S = 162
int romReadEstimatedMillisOf8Seconds() {//default 8500.(8000,9000) the millis of one time LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF) cycle
  return romReadInt(162);
}
void romUpdateEstimatedMillisOf8Seconds(unsigned int PositiveMillisOf8Seconds) {
  romUpdateInt(162, PositiveMillisOf8Seconds);
}

// A D D R E S S S = 164
int romReadBatteryThreshold() {//if connecting battery (via Mega Resistor) to some analog pin
  return romReadInt(164);//then have this value (directly not translated to volts) to know that if read below this
}//we must Power Off Arduino (if we have a latching relay version)
void romUpdateBatteryThreshold(unsigned int PositiveBelowWillTurnOfFromLatchRelay) {
  romUpdateInt(164, PositiveBelowWillTurnOfFromLatchRelay);
}

// A D D R E S S S = 166
int romReadUseGreenSimAndMustChangePins() {//old romReadMaxSimulatedDays now used for Green On card (meaning change pins on runtime)
  return romReadInt(166);
}
void romUpdateUseGreenSimAndMustChangePins(unsigned int PositiveAboveWillReverseThePinoutSim) {
  romUpdateInt(166, PositiveAboveWillReverseThePinoutSim);
}
// A D D R E S S S = 168
byte romReadMinutesPerReading() {
  return EEPROM.read(168);
}
void romUpdateMinutesPerReading(byte MinutesPerDay) {
  EEPROM.update(168 , MinutesPerDay);
}

// A D D R E S S S = 169
byte romReadBelowBatteryThresholdAcceptableTimes() {
  return EEPROM.read(169);
}
void romUpdateBelowBatteryThresholdAcceptableTimes(byte TimesAcceptableFoundBelowThreshold) {
  EEPROM.update(169 , TimesAcceptableFoundBelowThreshold);
}


///////////////////////////////////////////////////
///////////////////////////////////////////////////

///////////////////////////////////////////////////
///////////////////////////////////////////////////
// FREE A D D R E S S S = [170,210)
//leaving some space ...for future even not believe


////////// SCALEs ////A D D R E S S S = 210 ////////////////////////////////////
//each scale has
//1 byte for if is enabled on sms mode .(in lcd always showing)
//1 byte for ScaleStandartsUsing,if 0 or 1 then means UNCALLIBRATED (Callibrartion needs at least 2 averages Zero and a standart weight)
//1 long (TARE)ScaleGramsDif, this is the tare that will be deducted lastly on result (after translation of sensor reading to Grams according callibration)
//8 available callibration standarts for ScaleStandart INCLUDING ZERO,
//8 available callibration averages for ScaleAverage INCLUDING AVERAGE on EMPTY-ZERO
//Generally Callibration need averages(and standarts) for 0%,25%,50%,75%,100% (5 values)
//////////...but lets say someone need 0%,5%,20%,40%,60%,80%,100%,105% (8 values)
//example if i have a load cell for 200 Kg then
//a simple callibration is with 2 values 0 kg and lets say 10 kg
//but a better callibration will be to take the averages on 0 Kg , 50 Kg , 100 Kg ,150 Kg ,200 Kg
// 1 + 1 + 4 + 32 + 32 = 70 bytes on rom
// Scale 0 : A D D R E S S S = 210  +70
// Scale 1 : A D D R E S S S = 280 + 70
// Scale 2 : A D D R E S S S = 350 + 70
// Scale 3 : A D D R E S S S = 420 + 70
// Scale 4 : A D D R E S S S = 490 +70
//free at 560 eeprom
//byte temperatureCallibrated????
byte romReadScaleEnabled(int scale) {
  return EEPROM.read(210 + scale * 70);
}
byte romReadScaleStandartsUsing(int scale) {
  return EEPROM.read(211 + scale * 70);
}
long romReadScaleGramsDif(int scale) {//tare ...in grams(translated reading)

  return romReadLong(212 + scale * 70);
}
long romReadScaleStandart(int scale , int standart) {//12074
  return romReadLong(standart * 4 + 216 + scale * 70);
}
long romReadScaleAverage(int scale , int standart) {
  return romReadLong(standart * 4 + 248 + scale * 70);
}

void romUpdateScaleEnabled(int scale, byte val) {
  EEPROM.update(210 + scale * 70 , val);
}
void romUpdateScaleStandartsUsing(int scale, byte val) {
  EEPROM.update(211 + scale * 70 , val);
}
void romUpdateScaleGramsDif(int scale, long val) {
  romUpdateLong(212 + scale * 70, val);
}
void romUpdateScaleStandart(int scale , byte standart, long val) {
  romUpdateLong(standart * 4 + 216 + scale * 70, val);
}
void romUpdateScaleAverage(int scale , int standart, long val) {
  romUpdateLong(standart * 4 + 248 + scale * 70, val);
}


// FREE A D D R E S S S = 560

bool romMustFlush() {
  if (EEPROM.read(0) != programID) {
    return true;
  }
  byte b = 0;
  while (b < ScalesCanHandle) {
    if (romReadScaleStandartsUsing(b) > 8) {
      return true;//for sure
    }
    ++b;
  }
  return false;//most propably!!!


}
void romFlash() {//reset to defaults
  int addr = 0;
  EEPROM.update(addr, programID);
  ++addr;

#ifdef DEBUG
  while (addr < 31) {
    ser.println(F("Flushing"));//have this to be seen in case of..error.(happens once!)
    ++addr;
  }
#endif

  String str;
  str += F("1111"); romUpdateSimPin(str, 0); str.remove(0, str.length());
  romUpdatePinEnabled(0);//false
  romUpdateSimSecWaitAfterPowerOn(20);//20 seconds MAX wait after sim power on in order to have network.
  romUpdateSimSecWaitBeforePowerOff(13);//13 seconds wait lets say after we send sms
  romUpdateSmsEnabled(0);//NONE phone enabled to ensure new users save money not sending sms
  str += F("306949496595"); romUpdateSmsPrimaryNumber(str, 0); str.remove(0, str.length());
  str += F("306949496595"); romUpdateSmsSecondaryNumber(str, 0); str.remove(0, str.length());
  romUpdateDhtType(11);
  romUpdateDhtEnabled(1);
  romUpdateHourDelayBeforeSMSwhile(0);//0.Directly start day-looping
  romUpdateHoursPerReading(6);//4 readings per day default...1 sms per day...
  romUpdateMinutesPerReading(0);
  romUpdateCallInsteadSms(1);//Always ensure....calls instead of sms to save money...for new users....Change this by lcd mode if you want spend money
  romUpdatePhpEnabled(1);//enabled...new future.You must have or create a channle on thingspeak.http://api.thingspeak.com
  romUpdateSpaceToEmailFields();
  str += F("api_key=7I2NQSK2L0AQEW44"); romUpdateThingSpeakKey(str, 0); str.remove(0, str.length());
  str += F("internet.vodafone.gr"); romUpdateApn(str, 0); str.remove(0, str.length());
  str += F("http://api.thingspeak.com"); romUpdatePhpServer(str, 0); str.remove(0, str.length());
  str += F("update?"); romUpdatePhpQuery(str, 0); str.remove(0, str.length());


  //To adjust estimated millis of 8 seconds here is an example
  //If was 8772
  //and between readings of 2 hours (7.200.000 millis)
  //i had a difference of MINUS -4 minutes(240.000 millis)
  //then
  //a)7.200.000 / 8772 = 820 times we powered down for '8' seconds
  //b)Sharing out the difference to the times ... 240.000 / 820 = 293 diff millis per each time that we power down
  //c)And because we are MINUS 4 minutes the new value (estimated) is 8772 - 292 = 8479 !!!!!
  romUpdateEstimatedMillisOf8Seconds(8500);

  //For a 16 Mhz Pro mini we must have at least 3.78 Voltage.
  //So i will run at about 3.9 volts from 1,2,3! or 4! parallel sets of 3 AA rechargables.
  //Each set starting fully at about 4.35 volts and must not go below ~1.2 volt/cell so totally 3.6 volts.
  //For the range 4.35 till 3.9 we need a step down converter
  //For the range 3.6 till 3.9 we need a step up converter
  //Pololu 2868 and 2869 is fine (step-up/down) .(2868 has also auto shut down when below some voltage)
  //running at 3.9 will read 1023
  //           3.6       x?(=944 = 1023*3.6/3.9)
  //x~=940
  //In case that i use also Pololu Regulator 2868 that has also a second potensiometer for cut-off
  //then i will prefer to adjust this cut-off a little lower than 3.6(latch relay method for power off)
  //in order to most propably power-off from the latch relay and totally close application.
  //But having also a regulator with cut-off future like 2868...is more safest in case that....latch relay fails to work.
  //All this to be easy for someone to go on lithium batteries that need more CARE than Varta NIMH 2100 mA
  romUpdateBatteryThreshold(3450);//this is For MAIN POWER VIA LATCH RELAY VERSION,Cut Off below 3.45 volt
  romUpdateBelowBatteryThresholdAcceptableTimes(2);
  //new future trying to change si pins if using green card
  romUpdateUseGreenSimAndMustChangePins(0);

  byte sc = 0;
  while (sc < ScalesCanHandle) {
    romUpdateScaleEnabled(sc, sc % 2 == 0);
    romUpdateScaleStandartsUsing(sc, 0);//All scales are NOT callibrated on default
    romUpdateScaleGramsDif(sc, sc);//for debug....tares will be 0,4
    byte j = 0;
    while (j < 8) {
      romUpdateScaleStandart(sc, j, j * sc);//just have those ...for debug
      romUpdateScaleAverage(sc, j, -(j * sc));
      ++j;
    }
    ++sc;
  }


}
void romCheckToFlush() {
  if (romMustFlush()) {
    romFlash();
  } //else {ser.println("EEPROM good");}
}

void showeeprom() {
#ifdef DEBUG
  String str;
  ser.println(romReadProgramId());
  romReadSimPin(str); ser.println(str); str.remove(0, str.length());
  ser.println(romReadPinEnabled());
  ser.println(romReadSimSecWaitAfterPowerOn());
  ser.println(romReadSimSecWaitBeforePowerOff());
  romReadSmsPrimaryNumber(str); ser.println(str); str.remove(0, str.length());
  ser.println(romReadSmsEnabled());
  romReadSmsSecondaryNumber(str); ser.println(str); str.remove(0, str.length());
  ser.println(romReadDhtType());
  ser.println(romReadDhtEnabled());
  ser.println(romReadHourDelayBeforeSMSwhile());
  ser.println(romReadHoursPerReading());
  ser.println(romReadMinutesPerReading());
  ser.println(romReadCallInsteadSms());
  ser.println(romReadPhpEnabled());
  romReadThingSpeakKey(str); ser.println(str); str.remove(0, str.length());
  romReadApn(str); ser.println(str); str.remove(0, str.length());
  romReadPhpServer(str); ser.println(str); str.remove(0, str.length());
  romReadPhpQuery(str); ser.println(str); str.remove(0, str.length());

  ser.println(romReadEstimatedMillisOf8Seconds());
  ser.println(romReadBatteryThreshold());
  ser.println(romReadBelowBatteryThresholdAcceptableTimes());
  ser.println(romReadMaxSimulatedDays());
  byte sc = 0;
  while (sc < ScalesCanHandle) {
    ser.print(romReadScaleEnabled(sc)); ser.print(' ');
    ser.print(romReadScaleStandartsUsing(sc)); ser.print(' ');
    ser.print(romReadScaleGramsDif(sc)); ser.println(' ');
    byte j = 8;// 8 !! NO WHILE ...dont need too much debug!(yet)
    while (j < 8) {
      ser.print(romReadScaleStandart(sc, j)); ser.print(':'); ser.println(romReadScaleAverage(sc, j));
      ++j;
    }
    ++sc;
  }

#endif
}



//SoftwareSerial(ARDrxPin, ARDtxPin, inverse_logic)//INPUT_PULLUP,OUTPUT
SoftwareSerial* sim;

void resetPinModes() {
  byte b = 0;
  while (b < 5) {
    pinMode(PinHxDt[b], INPUT_PULLUP);
    pinMode(PinHxSck[b], OUTPUT);
    //digitalWrite(PinHxSck[b], LOW); //enable Hx711 in case we use HIGH on release...
    ++b;
  }
  pinMode(PinSimTx, INPUT_PULLUP);
  pinMode(PinSimRx, OUTPUT);
  digitalWrite(PinSimRx, HIGH); //critical....to be as when SoftwareSerial initilised !!!

}
void releasePinModesForPowerSaving() {

  byte b = 0;
  while (b < 5) {
    pinMode(PinHxDt[b], OUTPUT);
    //pinMode(PinHxSck[b], OUTPUT);//already output
    digitalWrite(PinHxDt[b], LOW);
    digitalWrite(PinHxSck[b], LOW); //HIGH);//ensuring Hx711 is in power down mode in all cases !??!! Problem if 13 is SCL.
    /*When PD_SCK pin changes from low to high
      and stays at high for longer than 60μs, HX711
      enters power down mode (Fig.3). When internal
      regulator is used for HX711 and the external
      transducer, both HX711 and the transducer will be
      powered down. When PD_SCK returns to low,
      chip will reset and enter normal operation mode.
      After a reset or power-down event, input
      selection is default to Channel A with a gain of
      128.*/
    ++b;
  }
  pinMode(PinSimTx, OUTPUT);
  //pinMode(PinSimRx, OUTPUT);//already output
  digitalWrite(PinSimTx, LOW);
  digitalWrite(PinSimRx, LOW);

}




////////////////////////
////////////////////////
////////////////////////
long parselong(const String & str, int from, int len) {
  long out = 0;
  int maxlen = str.length();
  char cha;
  len += from;
  if (len > maxlen) {
    len = maxlen;
  }
  while (from < len) {
    cha = str.charAt(from); //skipping high zeros
    if (cha != '0') {
      break;
    }++from;
  }
  while (from < len) {
    cha = str.charAt(from);
    if (cha >= '0' && cha <= '9') {
      cha -= 48;
      out += cha;
      out *= 10;
    }
    else {
      break;
    }++from;
  } out /= 10;
  return out;
}

byte MenuOkPressed, MenuOkPrevPressed;
byte RollEditPressed, RollEditPrevPressed;

//12,A7 just Input with external pull up resistor buttons.Change if using other pins
void readButtons() {
  MenuOkPressed = !digitalRead(PinButMenuOk);
  RollEditPressed = analogRead(PinButRollEdit);
  //ser.print(MenuOkPressed);ser.print(" , ");ser.println(RollEditPressed);

  if (RollEditPressed > 126) {
    RollEditPressed = 0;
  } else {
    RollEditPressed = 1;
  }
  //ser.print(MenuOkPressed);ser.print(" , ");ser.println(RollEditPressed);

}
void storeButtons() {
  MenuOkPrevPressed = MenuOkPressed;
  RollEditPrevPressed = RollEditPressed;
}
//releasing below this time millis(666) then Ok button mean next menu and Roll button mean next char position
//else Ok mean store or act (depending menu) and Roll button rolls to next DIGIT (from '4' to '5'etc)
const unsigned long smallDiff = 666;//use 555 is better(easier editing especially on php -apn long strings) but 666 is 'safer'...

unsigned long lastMenuOkPressed;
unsigned long lastMenuOkReleased;
unsigned long lastRollEditPressed;
unsigned long lastRollEditReleased;

byte OkActionType;//0 no action,1 simpleAction (without pressing the other button),2 special Action (pressing both buttons)

byte lcdModeEditing;//0 lcd0 only,1 lcd1 only ,2 both ,3 none
byte lcdRowEditing;//0 or 1.Need lcd 16x2 at least
byte lcdCheckOnOff;//'n' or 'f' trick to enable disable
char lcdCharMin, lcdCharMax;//[0,9] for numbers or else for letters...skipping big letters
byte lcdColumnEditing;//[0,15]
byte lcd0Inc, lcd0Min, lcd0Max;//how much to increase when hold button more than 555 and release
byte lcd1Inc, lcd1Min, lcd1Max;//min,max the current limits i allow editing
void done() {//usually going to next menu...on Oks (Oks means hold more than 555 millis the Ok button)...else Ok button is used for rolling menu that does not do changes

  OkActionType = 0;//this is for loop break
  lcdQuickPrint("done", 666);
  nextMenu();
}

void nextMenu() {
  //ser.println("nextMenu");
  ++currMenu;
  if (menuUnlocked) {
    currMenu %= menuCount;
    if (currMenu == 0) {
      ++currMenu;//skip again entering password
    }
  }
  else {
    currMenu %= 2;
  }
  switchingPrepare();
}
void prevMenu() {
  //ser.println("prevMenu");
  if (menuUnlocked) { //only on unlocked menu have prev....to save some Flash
    if (currMenu < 2) {
      currMenu = menuCount;
    }
    --currMenu;
    switchingPrepare();
  }
}

void rollDigitPosition() {
  if (lcdModeEditing < 3) {
    if (lcdRowEditing == 0) {
      lcdColumnEditing += lcd0Inc;
      if (lcdColumnEditing > lcd0Max) {
        if (lcdModeEditing == 2) {
          lcdColumnEditing = lcd1Min;
          lcdRowEditing = 1;
        }
        else {
          lcdColumnEditing = lcd0Min;
        }
      }
    }
    else if (lcdRowEditing == 1) {
      lcdColumnEditing += lcd1Inc;
      if (lcdColumnEditing > lcd1Max) {
        if (lcdModeEditing == 2) {
          lcdColumnEditing = lcd0Min;
          lcdRowEditing = 0;
        }
        else {
          lcdColumnEditing = lcd1Min;
        }
      }
    }
    lcd.setCursor(lcdColumnEditing, lcdRowEditing);
  }

}
void rollDigitChar() {
  if (lcdModeEditing < 3) {
    if (lcdRowEditing > 0) {
      char cha = lcd1.charAt(lcdColumnEditing);
      checkChar(cha);
      lcd1.setCharAt(lcdColumnEditing, cha);
    }
    else {
      char cha = lcd0.charAt(lcdColumnEditing);
      checkChar(cha);
      lcd0.setCharAt(lcdColumnEditing, cha);
    }
    lcdClearToStrings01();
    lcd.setCursor(lcdColumnEditing, lcdRowEditing);
  }
}





///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////



void checkChar(char & cha) {
  if (lcdCheckOnOff && (cha == 'n' || cha == 'f')) {
    if (cha == 'n') {
      cha = 'f';
    }
    else if (cha == 'f') {
      cha = 'n';
    }
  }
  else if (cha < 124) { //'~'=126 trick char to not turn...
    ++cha;

    if (cha == 'A') {
      cha = '_';//i had it 'a' but may undersore need
    }
    if (cha > lcdCharMax) {
      cha = lcdCharMin;
    }
  }
}

byte checkToRoll() {
  if (RollEditPressed != RollEditPrevPressed) {
    if (RollEditPressed) {
      lastRollEditPressed = millis();
      return 1;
    }
    else {
      lastRollEditReleased = millis();
      if (lastRollEditReleased - lastRollEditPressed < smallDiff) {
        rollDigitChar();
        return 2;
      }
      else {
        rollDigitPosition();
        return 3;
      }
    }
  }
  return 0;
}
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////















void prepare0PasswordPinRequesting() {
  //ser.println("PasswordPinRequesting");
  //i am not showing in lcd "Enter password 1111"...in case of some stranger...user must know
  lcd0 += "1111";
  lcdModeEditing = 0;
  lcdRowEditing = 0;
  lcdColumnEditing = 0;
  lcdCharMin = '0';
  lcdCharMax = '9';
  lcdCheckOnOff = 0;
  lcd0Inc = 1;
  //lcd1Inc = 1;
  lcd0Min = 0;
  //lcd1Min = 0;
  lcd0Max = 3;
  //lcd1Max = 1;
  lcd.blink();
}
void loop0PasswordPinRequesting() {
  if (OkActionType) {
    String currPin;
    romReadSimPin(currPin);
    int i = 0;

    while (i < 4) {
      if (lcd0.charAt(i) != currPin.charAt(i)) {
        //ser.println("wrong");I am not showing on lcd that is wrong....in case some stranger....
        OkActionType = 0; //for discart the ok released
        return;
      }
      ++i;
    }
    menuUnlocked++;
    done();
  }
  else {
    checkToRoll();
  }
}

void prepare1SetHoursDelayAndSimulation() {
  //ser.println("SetHoursDelayAndSimulation");

  byte hoursDelay = romReadHourDelayBeforeSMSwhile();
  if (hoursDelay < 10) {
    lcd0 += '0';
  }
  lcd0 += hoursDelay;
  lcd0 += F(" HoursBefStart");

  lcd1 += F("/hs ");
  byte hoursSim = romReadHoursPerReading();
  if (hoursSim < 10) {
    lcd1 += '0';
  }
  lcd1 += hoursSim;
  lcd1 += F("|");
  byte minSim = romReadMinutesPerReading();
  if (minSim < 10) {
    lcd1 += '0';
  }
  lcd1 += minSim;
  lcd1 += F(" Minutes");

  lcdModeEditing = 2;
  lcdRowEditing = 0;
  lcdColumnEditing = 0;
  lcdCharMin = '0';
  lcdCharMax = '9';
  lcdCheckOnOff = 0;
  lcd0Inc = 1;
  lcd1Inc = 1;
  lcd0Min = 0;
  lcd1Min = 4;
  lcd0Max = 1;
  lcd1Max = 8;

  lcd.blink();
}
void loop1SetHoursDelayAndSimulation() {
  if (OkActionType) {
    //byte newHoursDelay = parselong0p(lcd0, 0, 2);
    //byte newHoursSimulation = parselong0p(lcd1, 0, 2);
    //ser.print(newHoursDelay); ser.print(" , "); ser.println(newHoursSimulation);
    romUpdateHourDelayBeforeSMSwhile(parselong(lcd0, 0, 2));//not checking errors ..not nothing.Just primitive!
    romUpdateHoursPerReading(parselong(lcd1, 4, 2));//need flash !!!
    romUpdateMinutesPerReading(parselong(lcd1, 7, 2));//need flash !!!
    done();
  }
  else {
    checkToRoll();
  }
}


byte loopCounter;//usually showing every 256/3....but can change
bool showingReadingsAlso = false;
void checkRollOnTaringAndCallibratingMenus() {
  if (RollEditPressed != RollEditPrevPressed) {
    if (RollEditPressed) {
      lastRollEditPressed = millis();
    }
    else {
      lastRollEditReleased = millis();
      if (lastRollEditReleased - lastRollEditPressed < smallDiff) {
        //ser.println("show readings on/off");// and also util to tell to refresh simplest way in case of lets say utilise as simple scale !!!;
        showingReadingsAlso = !showingReadingsAlso;
        loopCounter = 0;//clear counter to directly take reading
      }
      else {
        //ser.println("next scale");
        ++currScale;
        currScale %= ScalesCanHandle;
        loopCounter = 0;//on next scale clear counter to directly take reading
        iTemp32 = 0;//only this different that we need on callibration
      }
    }
  }
}



void prepare2ShowingTaringSensors() {//roll light next scale,heavy ok tare.heavy roll readings view=!readings view
  //ser.println("ShowingTaringSensors");
  loopCounter = 0;
}
void loop2ShowingTaringSensors() {
  if (OkActionType) {
    //long newAver=takeAverage();
    //romUpdateScaleAverage();
    readWeight(currScale, false, true); //not taring but translating
    //long newTareValue=(long)lastGrams;
    romUpdateScaleGramsDif(currScale, lastGrams);
    //ser.print(currScale); ser.print(" scale new Tare :");ser.println(newTareValue);
    //now the GRAMS must shown to ZEROOOOOOOOOOOOOOOOOO
    OkActionType = 0;
    loopCounter = 0;
  }
  else {
    checkRollOnTaringAndCallibratingMenus();
  }

  if (loopCounter == 0) { //about every 10 seconds
    //ser.println(currScale+String("Reading curr scale "));
    lcd.clear();
    if (readWeight(currScale, true, true)) {
      lcd.print(lastGrams);
    }
    else {
      lcd.print("error");
    }
    if (showingReadingsAlso) {
      lcd.print('<');
      lcd.print(lastReading);
    }
    lcd.setCursor(0, 1);
    lcd.print(currScale);
    lcd.print('#');
    lcd.print(romReadScaleGramsDif(currScale));
    if (romReadDhtEnabled()) {
      lcd.print(",");
      dhtRead();//i know is stupid showing dht here but...what to do!?
      lcd.print((int)t);//WARNING printing floats consume FLASH
      lcd.print('C');
      lcd.print((int)h);//so i am casting.Dont care for 20,63 degrees and humidity .just 20
      lcd.print('%');
    }
  }

  ++loopCounter;

}




















//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
// CALLIBRATION MENU


void refreshCallibratingSensors() {
  lcdEmptyStrings01();
  iTemp33 = romReadScaleStandartsUsing(currScale);
  if (iTemp33 > 0) {
    readWeight(currScale, true, true);

    lcd0 += iTemp33;
    lcd0 += '/';
    lcd0 += (iTemp32 + 1);
    lcd0 += '#';
    lcd0 += currScale;
    lcd0 += ' ';
    lcd0 += lastGrams;


    lcd1 += romReadScaleAverage(currScale, iTemp32);
    lcd1 += '=';
    lcd1 += romReadScaleStandart(currScale, iTemp32);
  }
  else {
    lcd0 += '#';
    lcd0 += currScale;
    lcd1 += F("Hold Ok To Start");
  }
  lcdClearToStrings01();

}
void prepare3CallibratingSensors() {
  //ser.println("CallibratingSensors");
  lcdModeEditing = 1;
  lcdRowEditing = 0;
  lcdCheckOnOff = 0;
  lcd0Inc = 1;
  //currScale = 0; //scale to be as it was on showing-taring
  loopCounter = 0; //for just showing callibration values .....to be prettier but needs to remind to user that if hold key longer than 555 millis callibration process will start
  iTemp32 = 0; //the index that we are showing / or sampling
  //iTemp33();//the index we are storing currentScaleSamples / requestedScaleSamples
  iTemp34 = 0; //current Callibration Step
  refreshCallibratingSensors();
}
void loop3CallibratingSensors() {
  switch (iTemp34) {
    case 0: LoopCallibratingSensors_Step_0(); break;
    case 1: LoopCallibratingSensors_Step_1(); break;
    case 2: LoopCallibratingSensors_Step_2(); break;
    case 3: LoopCallibratingSensors_Step_3(); break;
      //default: ser.println("diaole");
  }
}
void LoopCallibratingSensors_Step_0() {
  if (OkActionType) {
    //ser.print(currScale); ser.println(" scale callibration starting");

    if (readCurrScaleWeightOnly()) {
      iTemp34++;

      //clearing  to Zero temp Storage for standarts and averages because on finishing we will STORE on EPROM all 1+(8*2) values
      byte i = 0;
      while (i < 8) {
        tempStandart[i] = 0;
        tempAverage[i] = 0;
        ++i;
      }


      lcdCharMin = '1';
      lcdCharMax = '7';
      lcd0Min = 0;
      lcd0Max = 0;
      lcdColumnEditing = 0;

      lcdEmptyStrings01();
      lcd0 += F("1 known weights");
      lcd1 += F("ScaleIsEmpty..Ok");
      lcdClearToStrings01();
      lcd.blink();
      lcd.setCursor(0, 0);

    }
    else {
      //ser.println("cant start");//callibration
      lcdQuickPrint("cant start", smallDiff);
    }
    OkActionType = 0;
    loopCounter = 0;
    return;
  }
  else {
    checkRollOnTaringAndCallibratingMenus();
  }

  if (loopCounter % 100 == 0) {
    refreshCallibratingSensors();
    ++iTemp32;
    iTemp32 %= iTemp33;
  }

  ++loopCounter;


}
void resetlcd0fprcallstep() {
  lcdEmptyStrings01();
  lcd1 += F("Weight Is Up..Ok");
  lcd0 += (iTemp32 + 1);
  lcd0 += '/';
  lcd0 += iTemp33;
  lcd0 += ':';
  lcd0 += F("000000 grams");
  //starting with 1 here..not zero...thats why 10-iTemp32() = for sure 9 index on lcd of 16 columns
  lcdColumnEditing = 10 - iTemp32;
  if (lcdColumnEditing < 4) {
    lcdColumnEditing = 4; //reming user to ...put bigger weights even if we are sorting at the end...
  }

  lcdClearToStrings01();
  lcd.setCursor(lcdColumnEditing, 0);
}

void printCallibrationAverageProcess(long & minv, long & maxv) {
  lcd.clear(); lcd.print(iter); lcd.print('/'); lcd.print(100); lcd.print(' '); lcd.print((long)h);//..dont print floats.saving flash
  lcd.setCursor(0, 1); lcd.print(minv); lcd.print(','); lcd.print(maxv);
}
long takeAverage() {
  //we already check that sensor works...
  //sum2 dividing every time by 2..Use only this?
  /*t=1;//avoiding devide by zero?....(but adding 1 uniti of scale HX711 reading) to sum
    if(readWeight(currScale,false,false)){
      t=lastReading;
    }*/
  readCurrScaleWeightOnly(); //warming up....

  iter = 0;
  h = 0; //total sum..using h from humidity for sram
  byte errors = 0;
  byte goods = 0;
  byte samples = 100;
  long minv = 2147483646;//not really need this but ok....
  long maxv = -2147483647;
  while (iter < samples) {
    if (iter % 10 == 0) {
      printCallibrationAverageProcess(minv, maxv);
    }
    //ser.println(iter);
    delay(123 + iter); //8-5 readings per second

    if (readCurrScaleWeightOnly()) {
      //t += lastReading; //continiuslly have middle from the time i was thinking that float in arduino overflows long max!!!
      //t /= 2.0;

      h += lastReading;

      if (lastReading < minv) {
        minv = lastReading;
      }
      if (lastReading > maxv) {
        maxv = lastReading;
      }
      ++goods;
    }
    else {
      ++errors;
    }
    ++iter;
  }

  if (goods > 0) {
    h /= goods;
  }

  //printCallibrationAverageProcess(minv, maxv);delay(2345);//may want to see min-max !!! saved 80 Flash bytes

  ///???????????????????
  //if (errors > 0) {//ser.println("Dont care!please upgrade!Average with errors");}

  return (long)h;// h=sum/sampls ....or t that is divided by 2 in every step
}
void LoopCallibratingSensors_Step_1() {
  if (OkActionType) {
    //ser.println("Procced to step 2");
    iTemp34++;

    iTemp32 = 0; //
    iTemp33 = (lcd0.charAt(0) - 47); //+1 for zero,,here is what how much known weight the user have
    //ser.println(iTemp33());
    //taking zero average here

    tempStandart[0] = 0;
    tempAverage[0] = takeAverage(); //the


    iTemp32++;

    lcdCharMin = '0';
    lcdCharMax = '9';
    lcd0Min = 4;
    lcd0Max = 9;
    lcdColumnEditing = 9;

    resetlcd0fprcallstep();

    OkActionType = 0;

  }
  else {
    checkToRoll();
  }
}


void LoopCallibratingSensors_Step_2() {
  if (OkActionType) {
    long gramsKnowned = parselong(lcd0, 4, 6);


    //#ifdef DEBUG
    //ser.println(lcd0);
    //ser.println(gramsKnowned);
    //ser.print(index);ser.print(F(" Try Averaging for : "));ser.println(gramsKnowned);
    //#endif

    //checking if user already took average for such weight
    int i = 0;
    while (i < iTemp32) {
      if (gramsKnowned == tempStandart[i]) {
        break;
      }++i;
    }
    if (i < iTemp32) {
      //ser.println(F("Already have this standard weight"));
      lcdQuickPrint("WeightAlready In", smallDiff);
      resetlcd0fprcallstep();
    }
    else {
      //taking known weights average here
      tempStandart[iTemp32] = gramsKnowned;
      tempAverage[iTemp32] = takeAverage();


      iTemp32++;//increasing to next known weight
      if (iTemp32 < iTemp33) {
        resetlcd0fprcallstep();
      }
      else {
        iTemp34++; //skipping step if finish taking averages of all known weights...and going to finilize this callibration
      }
    }

    OkActionType = 0;
  }
  else {
    checkToRoll();
  }


}


void LoopCallibratingSensors_Step_3() {
  //ser.println("_Step_3()");

  int i = 0; //iTemp33 standarts using.Also because unsigned long some; NOT going to zero...have this as =0 ion case.....(learning to initilise always on before loop)

  /*#ifdef DEBUG
    while (i < iTemp33) {
      ser.print(i);
      ser.print(F(": "));
      ser.print(tempStandart[i]);
      ser.print(F(" <- "));
      ser.println(tempAverage[i]);
      ++i;
    }
    ser.print(F("sorting")); ser.println(iTemp33);
    #endif
  */
  sortInsertionTempStandartsAndAveragesByStandarts(iTemp33);
  /*
    #ifdef DEBUG
    i = 0;
    while (i < 8) {//show all eight-8 values after sort
      ser.print(i);
      ser.print(F(": "));
      ser.print(tempStandart[i]);
      ser.print(F(" <- "));
      ser.println(tempAverage[i]);
      ++i;
    }
    #endif
  */
  /////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////
  //lcdQuickPrintAndRefresh(F("updating EEPROM"),smallDiff);
  //ser.println("updating");
  romUpdateScaleStandartsUsing(currScale, iTemp33);
  i = 0;// QUESTION SOS +50 Flash if comment this ?????amazing!!!(jn case i have un-commented the sorting print out above)
  while (i < 8) {
    romUpdateScaleStandart(currScale, i, tempStandart[i]);
    romUpdateScaleAverage(currScale, i, tempAverage[i]);
    ++i;
  }
  /*if (!romReadScaleEnabled(currScale)) {
    //automatically turn this on for sms mode to work.....freeing user from second action..most propably
    romUpdateScaleEnabled(currScale, true);
    //lcdQuickPrintAndRefresh(F("ScaleTurned On"),smallDiff);
    //ser.println("scale turned on");
    }*/
  romUpdateScaleEnabled(currScale, true);//above check replaced with this single line foir saving 22 flash
  /////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////



  currMenu -= 2; //showing weights and get ready to tare if want
  done();


}



// END END E N D CALLIBRATION MENU
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////







void prepare4OnOffSensorsAndDht() {
  //ser.println("OnOffSensorsAndDht");
  lcd0 += F("Of#0 Of#1 Of#2  ");
  lcd1 += F("Of#3 Of#4 Of#dht");
  byte b = 0, bb = 1;
  String * str = &lcd0;
  while (b < 5) { //-74 flash with this while
    if (romReadScaleEnabled(b)) {
      str->setCharAt(bb, 'n');
    }
    ++b;
    bb += 5;
    if (b == 3) {
      str = &lcd1;
      bb = 1;
    }
  }
  if (romReadDhtEnabled()) {
    lcd1.setCharAt(11, 'n');
  }
  lcdModeEditing = 2;
  lcdRowEditing = 0;
  lcdColumnEditing = 1;
  //lcdCharMin='0';
  //lcdCharMax='9';
  lcdCheckOnOff = 1;
  lcd0Inc = 5;
  lcd1Inc = 5;
  lcd0Min = 1;
  lcd1Min = 1;
  lcd0Max = 11;
  lcd1Max = 11;

  lcd.blink();
}
void loop4OnOffSensorsAndDht() {
  if (OkActionType) {
    byte b = 0, bb = 1;
    String * str = &lcd0;
    while (b < 5) { //-64 flash with this while
      romUpdateScaleEnabled(b, (str->charAt(bb) == 'n'));
      ++b;
      bb += 5;
      if (b == 3) {
        str = &lcd1;
        bb = 1;
      }
    }
    romUpdateDhtEnabled((lcd1.charAt(11) == 'n'));
    done();
  }
  else {
    checkToRoll();
  }

}

void prepare5SetDhtTypeAndPin() {
  //ser.println("SetDhtTypeDaysOffPin");
  //lcd0+="CallOn|11DhtType";
  //lcd1+="NeedPIN On|1111";

  lcd0 += F("CallOf");//this is not used for the moment....
  if (romReadCallInsteadSms()) {
    lcd0.setCharAt(5, 'n');
  }
  lcd0 += '|';

  byte dhtType = romReadDhtType();
  if (dhtType < 10) {
    lcd0 += '0';
  }
  lcd0 += dhtType;
  lcd0 += F("DhtType");


  lcd1 += F("NeedPIN Of|");
  if (romReadPinEnabled()) {
    lcd1.setCharAt(9, 'n');
  }
  romReadSimPin(lcd1);



  lcdModeEditing = 2;
  lcdRowEditing = 0;
  lcdColumnEditing = 5;
  lcdCharMin = '0'; //for dht we must do check and accept only '1' and '2'
  lcdCharMax = '9';
  lcdCheckOnOff = 1;
  lcd0Inc = 1;
  lcd1Inc = 1;
  lcd0Min = 5;
  lcd1Min = 9;
  lcd0Max = 8;
  lcd1Max = 14;

  lcd.blink();
}
void loop5SetDhtTypeAndPin() {
  if (OkActionType) {

    romUpdateCallInsteadSms(lcd0.charAt(5) == 'n');
    byte dhtType = parselong(lcd0, 7, 2);
    if (dhtType == 11 || dhtType == 12 || dhtType == 21 || dhtType == 22 ) {
      romUpdateDhtType(dhtType);
      //WARNING....must have change to public from library...in order to compile
      dht._type = dhtType; //no need begin() again ?...no need!
    }
    romUpdatePinEnabled(lcd1.charAt(9) == 'n');
    romUpdateSimPin(lcd1, 11);

    done();
  }
  else {
    checkToRoll();
  }
}


void prepare6SetSimSeconds() {
  //ser.println("SetSimSecondsNeedPin");
  byte secOn = romReadSimSecWaitAfterPowerOn();//WARNING new version dont use this .....
  if (secOn < 10) {
    lcd0 += '0';
  }
  lcd0 += secOn;
  lcd0 += F(" SecWait@SimOn");

  byte secOff = romReadSimSecWaitBeforePowerOff();
  if (secOff < 10) {
    lcd1 += '0';
  }
  lcd1 += secOff;
  lcd1 += F(" SecWait@SimOf");


  lcdModeEditing = 2;
  lcdRowEditing = 0;
  lcdColumnEditing = 0;
  lcdCharMin = '0';
  lcdCharMax = '9';
  lcdCheckOnOff = 0;
  lcd0Inc = 1;
  lcd1Inc = 1;
  lcd0Min = 0;
  lcd1Min = 0;
  lcd0Max = 1;
  lcd1Max = 1;
  lcd.blink();

}
void loop6SetSimSeconds() {
  if (OkActionType) {
    //byte secOn = parselong0p(lcd0, 0, 2);//-4 Flash bytes direct on arguments
    //byte secOff = parselong0p(lcd1, 0, 2);//-4 Flash bytes
    romUpdateSimSecWaitAfterPowerOn(parselong(lcd0, 0, 2));
    romUpdateSimSecWaitBeforePowerOff(parselong(lcd1, 0, 2));
    done();
  }
  else {
    checkToRoll();
  }
}



void prepare7SetSmsPhonesAndEnabled() {
  //ser.println("SetPhonesAndEnabled");

  lcd0 += F("aOf|");//a mean primary phone number
  romReadSmsPrimaryNumber(lcd0);
  lcd1 += F("bOf|");//b mean secondary phone number
  romReadSmsSecondaryNumber(lcd1);
  byte smsEnabledFlag = romReadSmsEnabled();
  if (smsEnabledFlag % 2 == 1) {
    lcd0.setCharAt(2, 'n');
  }
  if (smsEnabledFlag > 1) {
    lcd1.setCharAt(2, 'n');
  }

  lcdModeEditing = 2;
  lcdRowEditing = 0;
  lcdColumnEditing = 2;
  lcdCharMin = '0';
  lcdCharMax = '9';
  lcdCheckOnOff = 1;
  lcd0Inc = 1;
  lcd1Inc = 1;
  lcd0Min = 2;
  lcd1Min = 2;
  lcd0Max = 15;
  lcd1Max = 15;
  lcd.blink();
}
void loop7SetSmsPhonesAndEnabled() {
  if (OkActionType) {

    byte smsEnabledFlag = lcd1.charAt(2) == 'n';
    smsEnabledFlag <<= 1;
    smsEnabledFlag += lcd0.charAt(2) == 'n';

    romUpdateSmsEnabled(smsEnabledFlag);
    romUpdateSmsPrimaryNumber(lcd0, 4);
    romUpdateSmsSecondaryNumber(lcd1, 4);

    if (smsEnabledFlag == 0) {
      if (!romReadSmsEnabled()) { //ser.println("php turned on");
        romUpdatePhpEnabled(1);
      }
    }

    done();
  }
  else {
    checkToRoll();
  }
}

void prepare8SetPhpEnabled() {
  //ser.println("SignalingAndBattery");
  //digitalWrite(PinRelaySim,HIGH);
  lcd0 += F("PhpOf");
  if (romReadPhpEnabled()) {
    lcd0.setCharAt(4, 'n');
  }
  lcd1 += F("GreenOf");
  if (romReadUseGreenSimAndMustChangePins()){
    lcd1.setCharAt(6, 'n');
  }
  lcdModeEditing = 2;
  lcdRowEditing = 0;
  lcdColumnEditing = 4;
  //lcdCharMin = '0';
  //lcdCharMax = '9';
  lcdCheckOnOff = 1;
  lcd0Inc = 1;
  lcd1Inc = 1;
  lcd0Min = 4;
  lcd1Min = 6;
  lcd0Max = 4;
  lcd1Max = 6;
  lcd.blink();

}
void loop8SetPhpEnabled() {
  if (OkActionType) {
    //ser.println(lcd0.charAt(4) == 'n');
    romUpdatePhpEnabled(lcd0.charAt(4) == 'n');
    romUpdateUseGreenSimAndMustChangePins(lcd1.charAt(6) == 'n');
    if (!romReadPhpEnabled()) { //ser.println("php setted off");
      if (romReadSmsEnabled() == 0) { //ser.println("1st sms turned on");
        romUpdateSmsEnabled(1);//must do something on works !!!aytomatically changing!
        currMenu -= 2;
      }
    }

    done();
  }
  else {
    checkToRoll();
  }
}








//this function will be used editing long strings (apikey,apm,server,query)
//str30Chars is trimmed(with ' ' spaces from romRead....
void prepareStr30(char ch0, char ch1, const String & str30Chars) {
  lcd0 += ch0;
  lcd0 += ch1;
  lcd0 += "              ";//14 spaces
  lcd1 += "                ";//+16 spaces =30 max length for ....key,apn,server,querry strings
  byte len = str30Chars.length();
  byte index = 0;
  byte maxl = 14;
  byte at = 2;
  if (len < maxl) {
    maxl = len;
  }
  while (index < maxl) {//copying first 14 to lcd0 String
    lcd0.setCharAt(at, str30Chars.charAt(index));
    //ser.print(at+String(":"));ser.println((int)lcd0.charAt(at));
    ++index;
    ++at;
  }
  at = 0;
  while (index < len) {//copying 16 more to lcd1 String
    lcd1.setCharAt(at, str30Chars.charAt(index));
    //ser.print(at+String(":"));ser.println((int)lcd1.charAt(at));
    ++index;
    ++at;
  }
  //ser.println(lcd0.length()+lcd0);
  //ser.println(lcd1.length()+lcd1);
  lcdModeEditing = 2;
  lcdRowEditing = 0;
  lcdColumnEditing = 2;
  lcdCharMin = ' ';
  lcdCharMax = 'z';//[A,Z]U[Z,'_') is hacked ..only small letters can write .No need those 4 chars [\]^ after Z
  lcdCheckOnOff = 0;
  lcd0Inc = 1;
  lcd1Inc = 1;
  lcd0Min = 2;
  lcd1Min = 0;
  lcd0Max = 15;
  lcd1Max = 15;
  lcd.blink();
}
void fillChars14Fromlcd0And16Fromlcd1(const String & str30) {
  byte at = 2;
  while (at < 16) {
    str30 += lcd0.charAt(at);
    ++at;
  }
  at = 0;
  while (at < 16) {
    str30 += lcd1.charAt(at);
    ++at;
  }

}
void prepare9SetThingsKey() {
  //ser.println("SetPhpEnabledThingsKey");
  String ThingsKey;
  romReadThingSpeakKey(ThingsKey);
  prepareStr30('K', ':', ThingsKey);//prompt 'K' letter stands for KEY

}
void loop9SetThingsKey() {
  if (OkActionType) {
    String ThingsKey;
    fillChars14Fromlcd0And16Fromlcd1(ThingsKey);
    romUpdateThingSpeakKey(ThingsKey, 0);//can be empty if you build your own php server?!!?
    done();
  }
  else {
    checkToRoll();
  }
}

void prepare10SetApn() {//12916-13202=286 Flash
  //ser.println("SetApn");
  String apn;
  romReadApn(apn);
  prepareStr30('A', ':', apn);//prompt 'A' letter stands for APN

}
void loop10SetApn() {
  if (OkActionType) {
    String apn;
    fillChars14Fromlcd0And16Fromlcd1(apn);
    romUpdateApn(apn, 0);
    //currMenu -= 2;//debugging
    done();
  }
  else {
    checkToRoll();
  }
}


//WARNING...use http and not https even if the real server is ssl (https).
//SIM800L not working for https (or i dont know)(neither AT+EMAILSSL=1 command working for me)
void prepare11SetPhpServer() {
  //ser.println("SetPhpServer");
  String phpserver;
  romReadPhpServer(phpserver);
  prepareStr30('S', ':', phpserver);//prompt 'S' letter stands for SERVER
}
void loop11SetPhpServer() {
  if (OkActionType) {
    String phpserver;
    fillChars14Fromlcd0And16Fromlcd1(phpserver);
    romUpdatePhpServer(phpserver, 0);
    //currMenu -= 2;
    done();
  }
  else {
    checkToRoll();
  }
}

void prepare12SetPhpQuery() {
  //ser.println("SetPhpQuery");
  String PhpQuery;
  romReadPhpQuery(PhpQuery);
  prepareStr30('Q', ':', PhpQuery);//prompt 'Q' letter stands for QUERRY
}
void loop12SetPhpQuery() {
  if (OkActionType) {
    String PhpQuery;
    fillChars14Fromlcd0And16Fromlcd1(PhpQuery);
    romUpdatePhpQuery(PhpQuery, 0);
    //currMenu -= 2;
    done();
  }
  else {
    checkToRoll();
  }
}















//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////



//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
// TEST SIM MENU - SIGNALING - CALL , SMS , PHP (ThingSpeak)



String simAnswer;//warning ....
bool foundOK, foundeRRor;
int lastCharIndex;
int lastNumberRead;
void readSimAnswer(unsigned long maxTime, char charSearchingLast, bool breakOuterOnCharfound = false) {
  foundeRRor = false;
  foundOK = false;
  lastCharIndex = 0;
  lastNumberRead = 0;
  simAnswer.remove(0, simAnswer.length());//warning = better to have control on each call so to be empty AFTER use.But leave this...just for flash
  char chCurr, chPrev;
  unsigned long timeStartRead = millis();
  unsigned long timeEndRead = timeStartRead + maxTime;
  byte protectRamMax = 30;
  byte counts = 0;
  while (millis() < timeEndRead) {
    while (sim->available()) {
      chCurr = (char)sim->read();
#ifdef DEBUG_SIM_ANSWER
      ser.print(chCurr);
#endif
      if (chCurr == 'K' && chPrev == 'O') {
        foundOK = true;
        timeEndRead = 0; //to return immidiatelly reading only what is NOW available
      }
      else if (chCurr == 'R' && chPrev == 'R') {
        foundeRRor = true;
        timeEndRead = 0;
      }
      chPrev = chCurr;
      ++counts;
      if (counts > protectRamMax) { ///do nothing...just keep to find OK,Error.No further surch for requested char
      }
      else {
        if (chCurr == charSearchingLast) {
          lastCharIndex = simAnswer.length();
          if (breakOuterOnCharfound) {
            //ser.println("will break");
            //delay(10);
            timeEndRead = 0;
          }
        }
        simAnswer += chCurr;
        if (counts > 10) { //this is because httpAction answer like +HTTPACTION: 0,200,2 sometimes comes ? as V⸮QAJr⸮$bj⸮
          //but i am calling this function from readSimAnswer(1000 * romReadSimSecWaitBeforePowerOff(), ',',true);
          timeEndRead = 0;//so to not wait the ...13 seconds ...loosing time....but have comment here for someone wants to play with sim and then erase this if
          //example the answer of battery +CBC: 0,58,3867 may not work if iot lates to give next chars any from this ceil=7
          //or in signal...so MIGHT ...in rare case be wrong and must be erased
        }
      }
    }
    delay(100);
  }

}

void readSimToSerial(unsigned long maxTime) {
  foundOK = false;
  foundeRRor = false;
  unsigned long timeStartRead = millis();
  unsigned long timeEndRead = timeStartRead + maxTime;
  char chCurr, chPrev;
  while (millis() < timeEndRead) {
    while (sim->available()) {
      chCurr = (char)sim->read();
#ifdef DEBUG_SIM_ANSWER
      ser.print(chCurr);
#endif
      if (chCurr == 'K' && chPrev == 'O') {
        foundOK = true;
        timeEndRead = 0; //to return immidiatelly reading only what is NOW available
      }
      else if (chCurr == 'R' && chPrev == 'R') {
        foundeRRor = true;
        timeEndRead = 0;
      }
      chPrev = chCurr;
    }
    delay(100);
  }

}
/*
  void readSimToSerialDontCheckDontBreak(unsigned long maxTime) {
  unsigned long timeStartRead = millis();
  unsigned long timeEndRead = timeStartRead + maxTime;
  char chCurr;
  while (millis() < timeEndRead) {
    while (sim.available()) {
      chCurr = (char)sim.read();
  #ifdef DEBUG_SIM_ANSWER
      ser.print(chCurr);
  #endif
    }
    delay(100);
  }

  }*/
byte printSpace;//trick to be visible when refresh signal
void lcdPrintLastSignalRead() {
  lcd.setCursor(0, 0);
  if (printSpace % 2 == 0) {
    lcd.print(' ');//to be visible when refresh
  }
  ++printSpace;
  lcd.print(lastNumberRead);
  lcd.print(F(" Signal "));
}

/*
 * 2-9 Marginal
 * 10-14 OK
 * 15-19 Good
 * 20-30 Excellent
 * 99 Not known or not detectable
 */
void execAndReadATSignal() {
  sim->println(F("AT+CSQ"));//Max ResponseTime -?
  delay(100);
  readSimAnswer(1000, ':');
  //ser.println(simAnswer);
  if (foundOK && lastCharIndex > 0) {
    lastNumberRead = parselong(simAnswer, lastCharIndex + 2, simAnswer.length());
  }
}
int lastSimBatteryRead;
void execAndReadATBattery() {
  sim->println(F("AT+CBC"));//Max ResponseTime -?
  delay(100);
  readSimAnswer(1000, ',');
  lastSimBatteryRead = -1;
  if (foundOK && lastCharIndex > 0) {
    lastNumberRead = parselong(simAnswer, lastCharIndex + 1, simAnswer.length());
    lastSimBatteryRead = lastNumberRead;
  }
}
bool execAndReadATNetworkRegistration() {
  lastNumberRead = 2; //unregistered
  sim->println(F("AT+CREG?"));//Max ResponseTime -?
  delay(100);
  readSimAnswer(1000, ',');
  if (foundOK && lastCharIndex > 0) {
    lastNumberRead = parselong(simAnswer, lastCharIndex + 1, simAnswer.length());
  }
#ifdef DEBUG
  //ser.println(simAnswer);
  //ser.println(lastNumberRead);
#endif
  return lastNumberRead == 1;//now registered

}


/*
  Note: A HEX string such as "00 49 49 49 49 FF FF FF FF" will be sent out through serial
  port at the baud rate of 115200 immediately after SIM800 Series is powered on. The string
  Smart Machine Smart Decision
  SIM800 Series_AT Command Manual_V1.09 23 2015-08-03
  shall be ignored since it is used for synchronization with PC tool. Only enter AT Command
  through serial port after SIM800 Series is powered on and Unsolicited Result Code "RDY"
  is received from serial port. If auto-bauding is enabled, the Unsolicited Result Codes
  "RDY" and so on are not indicated when you start up the ME, and the "AT" prefix, or
  "at" prefix must be set at the beginning of each command line.
*/
bool openSimAndHasNetwork() {

  unsigned long maxEndTime = romReadSimSecWaitAfterPowerOn();
  //ser.println(maxEndTime);
  if (maxEndTime < 11) {//minimum seconds time for network registration including below delay
    maxEndTime = 11;
  }
  maxEndTime *= 1000;
  maxEndTime += millis();
  delay(6000);//minimum time for BASIC power up.Dont make questions to sim with no reason

  /*foundOK = false;
    while (!foundOK) {
    sim.println("AT");
    readSimToSerial(500);//WAITING for synchronisation and RDY
    if (millis() > maxEndTime) {
      break;
    }
    }
    if (foundOK) {*/
  foundOK = false;
  while (!foundOK) {
    if (execAndReadATNetworkRegistration()) {
#ifdef DEBUGFULL
      ser.println(F("sim ready"));
#endif
      return true;
    }
    foundOK = false;
    if (millis() > maxEndTime) {
      break;
    }
    delay(3000);//so  1st question 6 sec,2nd 9 sec,3rd 12 sec...etc
    //ser.println("One more try to find network");
  }
  //}
  return false;
}

void unlockSimWithPinIfNeed() {
  if (romReadPinEnabled()) {
    String pin;
    romReadSimPin(pin);
    sim->print(F("AT+CPIN="));//Max ResponseTime 5s
    sim->println(pin);
    delay(100);
    readSimToSerial(5000);
    /*not enough memory
      if(foundOk){ser.println("Pin OK");}
      else{ser.println("Pin NOT good");}
    */
  }

  //readSimToSerialDontCheckDontBreak(4000); //delaying 4 sec just for in case sim sending data like Pin ready,sms ready,call ready
}
bool openSimHasNetAndPinOk() {
  if (openSimAndHasNetwork()) {
    unlockSimWithPinIfNeed();
  }
  return foundOK;
}

void lcdPrintFailed() {
  lcd.print(F(" Failed"));
}
bool nowSignaling;

void prepare13Test() {
  loopCounter = 0;
  nowSignaling = false;
  lcd0 += F("OkToTestPhpSms");
  lcd1 += F("RollForSignaling");
  lcdModeEditing = 3;//none

}
void loop13Test() {
  if (OkActionType) {
    //ser.println("TestPinBattCallSmsPhp");
    lcd.clear();
    lcd.print(F("working SIM800L"));
    work();
    currMenu -= 1;
    done();
  }
  else {
    if (checkToRoll() > 1) {
      //ser.println("Signaling on/off");
      if (nowSignaling) {
        currMenu -= 1;
        done();
        nowSignaling = false;
      }
      else {
        lcd.clear();
        nowSignaling = true;
        loopCounter = 0;
      }
    }
  }

  if (nowSignaling && (loopCounter % 86 == 0)) { //every ~5 sec
    //ser.println("signaling");
    execAndReadATSignal();
    lcdPrintLastSignalRead();
  }
  ++loopCounter;

}


//Volt<3450|1Times
void prepare14SetCutOff() {//24450,1020
  lcd0 += F("Volt<3450|1Times");//3.45 volt to 3 AA batteries ...meaning Power off to save rechargable batteries !!!

  int val = romReadBelowBatteryThresholdAcceptableTimes();
  int index = 10;
  lcd0.setCharAt(index, (val % 10) + 48);
  val = romReadBatteryThreshold();
  index = 8;
  while (index > 4) {
    lcd0.setCharAt(index, (val % 10) + 48);
    val /= 10;
    --index;
  }

  /*
    val = romReadMaxSimulatedDays();
    lcd1 += F("9999 MaxDays OFF");
    index = 3;
    while (index >= 0) {
    lcd1.setCharAt(index, (val % 10) + 48);
    val /= 10;
    --index;
    }*/

  lcdModeEditing = 1;
  lcdRowEditing = 0;
  lcdColumnEditing = 5;
  lcdCharMin = '0';
  lcdCharMax = '9';
  lcdCheckOnOff = 0;
  lcd0Inc = 1;
  //lcd1Inc = 1;
  lcd0Min = 5;
  //lcd1Min = 0;
  lcd0Max = 10;
  //lcd1Max = 3;

  lcd.blink();
  loopCounter = 0;

}
void loop14SetCutOff() {
  if (OkActionType) {
    romUpdateBatteryThreshold(parselong(lcd0, 5, 4));
    romUpdateBelowBatteryThresholdAcceptableTimes(parselong(lcd0, 10, 1));
    //romUpdateMaxSimulatedDays(parselong(lcd1, 0, 4));
    --currMenu;
    done();
  }
  else {
    checkToRoll();
  }

  if (loopCounter == 0 || loopCounter == 127) {
    execAndReadATBattery();
    readPowerAndTranslateAccordingSimsPower();
    lcd.setCursor(0, 1);
    lcd.print(lastSimBatteryRead);
    lcd.print(' ');
    lcd.print(powerAnalogReading);
    lcd.print(' ');
    lcd.print(powerReadingTranslated0);
    lcd.setCursor(lcdColumnEditing, 0);

  }
  ++loopCounter;
}










void prepare15SetEstimated() {
  lcd0 += F("-Of|000 secAdj");

  int val = romReadEstimatedMillisOf8Seconds();
  lcd1 += F("8453 Millis8Sec");
  int index = 3;
  while (index >= 0) {
    lcd1.setCharAt(index, (val % 10) + 48);
    val /= 10;
    --index;
  }

  lcdModeEditing = 2;
  lcdRowEditing = 0;
  lcdColumnEditing = 2;
  lcdCharMin = '0';
  lcdCharMax = '9';
  lcdCheckOnOff = 1;
  lcd0Inc = 1;
  lcd1Inc = 1;
  lcd0Min = 2;
  lcd1Min = 0;
  lcd0Max = 6;
  lcd1Max = 3;

  lcd.blink();
}
void loop15SetEstimated() {
  if (OkActionType) {
    int val = romReadEstimatedMillisOf8Seconds();
    unsigned long est8 = parselong(lcd1, 0, 4);
    if (val != est8) {//only one change at a time ...this menu !
      if (est8 > 8000 && est8 < 10000) {
        romUpdateEstimatedMillisOf8Seconds(est8);
      }
    }
    else { //checking to adjust estimated 8 seconds by curr diff!!!!
      unsigned long difSeconds = parselong(lcd0, 4, 3);
      if (difSeconds > 0) {

        unsigned long hs = romReadHoursPerReading();
        byte ms = romReadMinutesPerReading();
        est8 = romReadEstimatedMillisOf8Seconds();
        if ( (hs > 0 || ms > 0) && est8 > 8000) {

          bool minus = lcd0.charAt(2) == 'n';

          /*#ifdef DEBUG
                    ser.print(F("Adjusting 8 sec estim ")); ser.print(est8);
                    ser.print(F(" hs ")); ser.print(hs);
                    ser.print(F(" ms ")); ser.println(ms);
                    ser.print(F("Seconds diff ")); ser.print(difSeconds);
                    if (minus) {
                      ser.println(F(" Sooner"));
                    }
                    else {
                      ser.println(F(" Later"));
                    }
            #endif*/

          hs *= 60; //minutes
          hs += ms; //total minutes per reading
          hs *= 60; //total seconds per reading
          hs *= 1000; //total millis per reading
          long powerdowns = hs / est8;
          difSeconds *= 1000;
          unsigned long difSharedToPowerDownTimes = difSeconds / powerdowns;
          if (minus) {
            est8 -= difSharedToPowerDownTimes;
          }
          else {
            est8 += difSharedToPowerDownTimes;
          }


#ifdef DEBUG
          ser.print(F("Millis/Reading ")); ser.println(hs);
          ser.print(F("Downs ")); ser.println(powerdowns);
          ser.print(F("Sharing millis on each power down")); ser.println(difSharedToPowerDownTimes);
          ser.print(F("New estimated 8 seconds ")); ser.println(est8);
#endif


          if (est8 > 8000 && est8 < 10000) {
            romUpdateEstimatedMillisOf8Seconds(est8);
          }

        }
      }//else no change
    }
    --currMenu;
    done();
  }
  else {
    checkToRoll();
  }
}


void switchingPrepare() {
  lcdModeEditing = 3; //none
  lcdEmptyStrings01();
  lcd.noBlink();
  lcd.clear();

  switch (currMenu) {
    case 0: prepare0PasswordPinRequesting(); break;
    case 1: prepare1SetHoursDelayAndSimulation(); break;

    case 2: prepare2ShowingTaringSensors(); break;
    case 3: prepare3CallibratingSensors(); break;

    case 4: prepare4OnOffSensorsAndDht(); break;
    case 5: prepare5SetDhtTypeAndPin(); break;
    ///SIM
    case 6: prepare6SetSimSeconds(); break;
    case 7: prepare7SetSmsPhonesAndEnabled(); break;

    case 8: prepare8SetPhpEnabled(); break;
    case 9: prepare9SetThingsKey(); break;
    case 10: prepare10SetApn(); break;
    case 11: prepare11SetPhpServer(); break;
    case 12: prepare12SetPhpQuery(); break;

    case 13: prepare13Test(); break;
    case 14: prepare14SetCutOff(); break;
    case 15: prepare15SetEstimated(); break;


  }
  lcdClearToStrings01();
  lcd.setCursor(lcdColumnEditing, lcdRowEditing);

}
void switchingLoop() {
  switch (currMenu) {
    case 0: loop0PasswordPinRequesting(); break;
    case 1: loop1SetHoursDelayAndSimulation(); break;
    case 2: loop2ShowingTaringSensors(); break;
    case 3: loop3CallibratingSensors(); break;
    case 4: loop4OnOffSensorsAndDht(); break;
    case 5: loop5SetDhtTypeAndPin(); break;
    case 6: loop6SetSimSeconds(); break;
    case 7: loop7SetSmsPhonesAndEnabled(); break;

    case 8: loop8SetPhpEnabled(); break;
    case 9: loop9SetThingsKey(); break;
    case 10: loop10SetApn(); break;
    case 11: loop11SetPhpServer(); break;
    case 12: loop12SetPhpQuery(); break;

    case 13: loop13Test(); break;
    case 14: loop14SetCutOff(); break;
    case 15: loop15SetEstimated(); break;
  }
}



void loopLcd() {
  readButtons();

  if (MenuOkPressed != MenuOkPrevPressed) {
    //ser.println(MenuOkPressed);
    if (MenuOkPressed) {
      lastMenuOkPressed = millis();
    }
    else {
      lastMenuOkReleased = millis();
      if (lastMenuOkReleased - lastMenuOkPressed < smallDiff) {
        if (RollEditPressed) {
          prevMenu();
        }
        else {
          nextMenu();
        }
      }
      else {
        //ser.println("OK");
        OkActionType = 1;
      }
    }
  }

  switchingLoop();
  storeButtons();
  delay(40);
}


void setup() {

#ifdef DEBUG
  ser.begin(9600);
#endif

  pinMode(PinButMenuOk,INPUT_PULLUP);
  ModeLcd = digitalRead(PinButMenuOk);
  //ModeLcd=1;//for debug
  //blinc(2);//can leave the button now....in case of SMS mode...I understood that we want SMS mode..just for debug

  String jimakos("jimakoskx@");
  if(romReadUseGreenSimAndMustChangePins()){
    byte temp=PinSimRx;
    PinSimRx=PinSimTx;
    PinSimTx=temp;
    jimakos+="green";
  }
  else{
    jimakos+="red";
  }
  sim=new SoftwareSerial(PinSimTx,PinSimRx);
  setPinModes();

  //WHY?????????????????????????????
  //sim.begin(9600);//???if i have it here does not begining....

  if (ModeLcd) {

    romCheckToFlush();

  }
  else { //Sms mode

  }


#ifdef DEBUGFULL
  showeeprom();//1000+ flash bytes this!
#endif


  dht._type = romReadDhtType(); //was initiated as type=11
  if (ModeLcd || romReadDhtEnabled()) { //saving some power? on sms not dht enabled?....
    dht.begin();//library setting pinMode here!...this will also be called when we changing TYPE.No need to  be called every time that we are opening the Sensor Relays!
  }
  
  sim->begin(9600);//???here is ok...


  clearStoredWeights();//to DISABLED_CHAR instead of zero. This will also be used on lcd mode testing.

  //lcd.begin();//for debug

  if (ModeLcd) {
#ifdef DEBUG
    ser.println(F("\nMODE LCD\n"));
#endif
    lcd.begin();
    lcd.print(jimakos);
    relaySensorsTrigger();
    openSimHasNetAndPinOk();
    nextMenu();

  }
  else {
#ifdef DEBUG
    ser.println(F("\nMODE SMS\n"));
#endif

#ifdef LATCH_SENSORS_RELAY //if normal relay then for sure we are not triggered after reset
    relaySensorsUnTrigger();//this is to be sure we are not triggered when coming here after lcd mode
    delay(100);
#endif

    setupSms();
  }


}

void loop() {
  if (ModeLcd) {
    loopLcd();
  }
  else {
    loopSms();
  }

}

unsigned long mustDelayOrSleep;//this is only for debug
unsigned long timesPowerDown8Seconds;//this is only for debug
unsigned long millisRemainForNormalDelay;//this is only for debug

int currDay;  // int..32.768 days MY batteries!!!? or even only byte 255 days to keep batteries?
unsigned long estimatedMillisOf8SecondsPowerDown = 8500; //you must adjust this
#define USE_LOW_POWER
void myDelay(unsigned long someTime) {//finally this function release from RTC need...good enough


#ifdef USE_LOW_POWER
  timesPowerDown8Seconds = someTime / estimatedMillisOf8SecondsPowerDown;
  millisRemainForNormalDelay = someTime % estimatedMillisOf8SecondsPowerDown;
  unsigned long times = timesPowerDown8Seconds; //hold this to be able to show it after for debug
  //WARNING .This delay must be before while because???
  //is not working fine if after
  delay(millisRemainForNormalDelay);
  while (times > 0) {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    --times;
  }
  //delay(millisRemainForNormalDelay);//not working here
#endif







#ifndef USE_LOW_POWER
  delay(someTime);
#endif

}



void checkToSendInitialSmsInCaseOfMustDelaySomeHoursBeforeStart() {
  mustDelayOrSleep = romReadHourDelayBeforeSMSwhile();
  if (mustDelayOrSleep > 0) {
    mustDelayOrSleep *= 3600000ul;
    work();//this is the initial sms/call for user to know that is ok...sms mode
    iter = 0;
    currDay = 0;
    myDelay(mustDelayOrSleep);//does not deduct the time taken(half-2 minutes) to do the initial actions above....
  }
}


void setupSms() {


#ifndef USE_DEFINED_MILLIS_PER_READING
  int minutes = romReadHoursPerReading();
  minutes *= 60;
  minutes += romReadMinutesPerReading();
  if (minutes < 2) { //..just checking.....
    minutes = 360;//6 hours default?
  }

  millisPerReading = minutes;
  millisPerReading *= 60; //seconds
  millisPerReading *= 1000;
#endif //else just use what i have in the beginning of the file (for debugging)





  unsigned long estimcheck = romReadEstimatedMillisOf8Seconds();
  if (estimcheck > 8000 && estimcheck < 10000) { //just a checking !!!...doubled?
    estimatedMillisOf8SecondsPowerDown = estimcheck;
    //ser.println(estimatedMillisOf8SecondsPowerDown);
  }//else{}//not acceptable ..just using default





  //#ifdef DEBUGFULL
  //timesPowerDown8Seconds = millisPerReading / estimatedMillisOf8SecondsPowerDown;

  //ser.print("hours "); ser.println(romReadHoursPerReading());
  //ser.print("minutes "); ser.println(romReadMinutesPerReading());
  //ser.print("reads per millis "); ser.println(millisPerReading);
  //ser.print("/estim "); ser.print(estimatedMillisOf8SecondsPowerDown);
  //ser.print(" = downtimes "); ser.println(timesPowerDown8Seconds );
  //ser.print("sms every sec"); ser.println( (4 * millisPerReading) / 1000);
  //#endif

  checkToSendInitialSmsInCaseOfMustDelaySomeHoursBeforeStart();

}
void loopSms() {
  workingStep();
}


void workingStep() {
  //1.hold time
  unsigned long startMillis = millis();//reference to when on this line

  //2.work
  work();

  //some commands before call ending millis() for more accuracy
#ifdef DEBUG
  //ser.flush();
#endif


  //usually php and sms taking about 2 minutes so hourMillis(declared) for debug must be at least 20.000 that * 6 = 2 minutes debug each reading
  //last call on millis()....quick end this function.....
  //3.compute how much we must sleep-delay
  //mustDelayOrSleep = millisPerReading ;
  //mustDelayOrSleep += startMillis;
  //mustDelayOrSleep -= millis();///this may be negative =very large if workingTime take more than millisPerReading
  //correction of 3.step above to not sleep negative(for ever)

  unsigned long timeTooked=millis();
  timeTooked-=startMillis;//time tooked to work
  if(timeTooked<millisPerReading){
    mustDelayOrSleep=millisPerReading;
    mustDelayOrSleep-=timeTooked;
    myDelay(mustDelayOrSleep);
  }
  //else{dont sleep ....danger to loop and eat battery..thats why i had the old.3...with o check}
  
  
  

}//end simulateReadingWithPowerConsumingDelayAndNoRTC


//Critical out because if low we must turn off application from latch relay(main power)
//because sometimes gives me very low....reminds me the issue with the atmega328 buildin(internal) power measuring function!!
//analogReference()????????
void readPowerAndTranslateAccordingSimsPower() {
  int times = 5;
  int val = analogRead(PinPowerRead);
  int maxVal = analogRead(PinPowerRead); //discart first reading
  while (times > 0) {
    val = analogRead(PinPowerRead);
    if (val > maxVal) {
      maxVal = val;
    }
    delay(10);
    --times;
  }
  powerAnalogReading = maxVal;
  //#ifdef DEBUGFULL
  //ser.print(F("Battery Analog :")); ser.println(powerAnalogReading);
  //#endif

  if (lastSimBatteryRead > 0) {
    // 1023 -> lastSimBatteryRead
    //maxVal ->x?
    //x=lastSimBatteryRead*maxVal /1023;
    float fff = lastSimBatteryRead;
    fff *= powerAnalogReading;
    fff /= 1023;
    powerReadingTranslated0 = fff;
    //#ifdef DEBUGFULL
    //ser.print(F("Translated to Volts :")); ser.println(powerReadingTranslated0);
    //#endif
  }
  else {
    powerReadingTranslated0 = maxVal;
  }
}

/*from ...google....as for batteries
   NiMH cells start at about 1.5 V right when fully charged,
   drop to about 1.2 V most of their discharge life,
   and are pretty much empty at 900 mV.
   Stopping there is usually safe. 800 mV is where you definitely want to stop to avoid damage
*/
/*for step up/down pololu converter
   The output voltage can be up to 3% higher than normal when there is little or no load on the regulator.
   The output voltage can also drop depending on the current draw,
   especially when the regulator is boosting a lower voltage to a higher one (stepping up),
   although it should remain within 5% of the set voltage.
   5% of 4 volts mean 3.8 !!!! ..normal !!!
   a)So below 3800 we may have problem on pololu
   b)between 3800 and 3900 we DONT want to count the analog power because it will give us falt result
   c)Only
*/
bool checkToExitFromLowPower() { //in case no latch main power relay we can just take intoo account sim voltage?!!!
  bool mustExitFromLowPower = false;
  if (lastSimBatteryRead > 0) {//if reading sim answer was succesfull

    if (powerReadingTranslated0 < romReadBatteryThreshold()) {
      ++continiusllyTranslatedBelowThreshold;//now for sure is 1 and will exit if times=0
      mustExitFromLowPower = continiusllyTranslatedBelowThreshold > romReadBelowBatteryThresholdAcceptableTimes();
    }
    else {
      continiusllyTranslatedBelowThreshold = 0;
      //return false;
    }

  }
  //else{return false;}
  return mustExitFromLowPower;
}



void powerOff() {

#ifdef DEBUG
  blinc(5);
#endif


  relayMain_POWER_OFF();
}



void work() {

  //iter:0,1,2,3 //THIS MUST COMING EXACTLY EVERY....THIS IS TO BE TESTED!!!
#ifdef DEBUG
  //no possible to update this before ..like not slept but sleeping...!!!!!!!!!!..because printings must be here in work to not loose time
  //ser.print(F("slept "));
  ser.print(F("EXACTLY ? "));
  ser.print(timesPowerDown8Seconds);
  ser.print(F(" *~8 + "));
  ser.print(millisRemainForNormalDelay);
  ser.print(F(" = "));
  ser.print(mustDelayOrSleep);
  ser.print(F(" iter:")); ser.print(iter); ser.print('/'); ser.println(currDay);
#endif

  if (!ModeLcd) {

    resetPinModes();
    relaySensorsOpenNice();//now sensors and sim is powered
    delay(100);
    //ser.println("resetPinModes()/relaySensorsOpenNice()");
  }


  storeIterReadings();//BASIC JOB


  if (!ModeLcd) {
    //execute safety CFUN=1 in case relays does not close?
    openSimHasNetAndPinOk();
    //ser.println("openSimHasNetAndPinOk()");
  }//else already have opened on setup

  //this will read battery from SIM800L but in case i have a pololu step up-down will be always at 4.3 Volts....
  //this is very nice because i can compute dynamically (without contant reference to running voltage)...and can play with pololu VOUT
  //prefering VOUT low just to be able to open relays ,run SIM (3.4-4.4),,,and lcd can be seen.
  delay(1000);//to stabilize -a little- step up converter?NOT SURE that this have effect on voltage teardrops in diagram
  execAndReadATBattery();//ser.print("Battery Voltage");
#ifdef LATCH_MAIN_RELAY
  delay(1000);//to stabilize -a little- step up converter?NOT SURE that this have effect on voltage teardrops in diagram
  readPowerAndTranslateAccordingSimsPower();
  if (ModeLcd) {
    lcd.setCursor(0, 1);
    lcd.print(lastSimBatteryRead);
    lcd.print(' ');
    lcd.print(powerAnalogReading);
    lcd.print(' ');
    lcd.print(powerReadingTranslated0);
  }
#endif

  if (romReadPhpEnabled()) {
    sendPhpData();
  }
  if (iter == 1) {
    byte smsMode = romReadSmsEnabled();
    sendDailySms(smsMode);//will check if zero....do nothing
    ++currDay;
  }


  if (!ModeLcd) {



#ifdef LATCH_MAIN_RELAY
    if (checkToExitFromLowPower()) {
      powerOff();
    }
#endif




    relaySensorsUnTrigger();
    releasePinModesForPowerSaving();
    //execute safety CFUN=0  command  in case relays does not close?
    //or even better AT+CPOWD=[0 or 1] but this need 1 pin to wake up!!!
    //ser.println("relaySensorsUnTrigger()/releasePinModes()");
  }


#ifdef DEBUG
  ser.println();
#endif


}// end work


void relaySensorsOpenNice() {
  relaySensorsTrigger();
  unsigned long sl = 1000;
  if (romReadDhtEnabled()) {
    sl += 2000; //Dht wants some time....Here we can also take some HX711 readings?
  }
  delay(sl);//now must be ready for sensing
}
void storeIterReadings() {
  char chi;
  if (romReadDhtEnabled()) {
    dhtRead();
    if (isnan(t)) {
      chi = ERROR_CHAR;
#ifdef DEBUG
      ser.print('?');
#endif
    }//error ...means not showing on sms
    else {
      chi = t;
#ifdef DEBUG
      ser.print((int)chi); ser.print(' ');
#endif
    }
    temperature[iter] = chi;

    if (isnan(h)) {
      chi = ERROR_CHAR;
#ifdef DEBUG
      ser.print('?');
#endif
    }//error ...means not showing on sms
    else {
      chi = h;
#ifdef DEBUG
      ser.print((int)chi); ser.print(' ');
#endif
    }
    humidity[iter] = chi;
  }
  else {
    temperature[iter] = DISABLED_CHAR; //disabled ...means not showing on sms
    humidity[iter] = DISABLED_CHAR; //disabled ...means not showing on sms

  }



  chi = 0;
  int posOfFreshReading = iter * ScalesCanHandle;
  int half = SensationsPerSms * ScalesCanHandle + posOfFreshReading;
  while (chi < ScalesCanHandle) {
    StoredWeights[half] = StoredWeights[posOfFreshReading]; //store previus
    if (romReadScaleEnabled(chi)) {
      if (readWeight(chi, true, true)) {
        StoredWeights[posOfFreshReading] = lastGrams;
#ifdef DEBUG
        ser.print(StoredWeights[posOfFreshReading]); ser.print('<'); ser.print(lastReading); ser.print(',');
#endif
      }
      else {
        StoredWeights[posOfFreshReading] = ERROR_CHAR;
#ifdef DEBUG
        ser.print('?');
#endif
      }
    }
    else {
      StoredWeights[posOfFreshReading] = DISABLED_CHAR;
    }


    ++half;
    ++posOfFreshReading;
    ++chi;
  }

#ifdef DEBUG
  ser.println();
#endif

  ++iter;
  iter %= SensationsPerSms;
}

byte success;
#define SIGNAL_ALSO
long lastSignal;
void sendPhpData() {
  //ser.println("sendPhpData()");
  success = 0; //state 0 beginning

  #ifdef SIGNAL_ALSO
  execAndReadATSignal();
  lastSignal=lastNumberRead;
  #endif;


  String stri;
  romReadApn(stri);
  simGetIP(stri);
  stri.remove(0, stri.length());
  if (foundOK) {
    ++success;//state 2 ip obtained
    sim->println(F("AT+HTTPINIT"));
    delay(100);
    readSimToSerial(3000);
    if (foundOK) {
      ++success;//state 3 OK AT+HTTPINIT

      sim->println(F("AT+HTTPPARA=\"CID\",1"));
      delay(100);
      readSimToSerial(3000);
      if (foundOK) {
        ++success;//state 4 OK AT+HTTPPARA
        simMakeHttpPhpGetSession(stri);
      }
      else {
        //ser.println("Cant set http params");
      }
    }
    else {
      //ser.println("Cant init HTTP");
    }
    sim->println(F("AT+HTTPTERM"));
    delay(100);
    readSimToSerial(3000);
  }
  else {
    //ser.println("Cant get IP");
  }
  //}
  //else {
  //ser.println("Cant open sim");
  //}
  sim->println(F("AT+SAPBR=0,1"));//close bearer ip connection
  delay(100);
  readSimToSerial(65000);

#ifdef DEBUG
  ser.print("php:"); ser.print(success);
#endif

}



bool simGetIP(String & apnWithoutPassword) {
  bool out = false;
  //Step 1                              //9.2.1 AT+SAPBR Bearer Settings for Applications Based on IP

  //For me working without "AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"";//AT+SAPBR=3,1,"Contype","GPRS"....fix this in case...

  sim->print(F("AT+SAPBR=3,1,\"APN\",\"")); //Max ResponseTimeWhen <cmd_type> is 1, 85 secondsWhen <cmd_type> is 0, 65 seconds
  sim->print(apnWithoutPassword);        //+SAPBR: (0-4),(1-3), "ConParamTag","ConParamValue"
  sim->println('\"');                    //AT+SAPBR=<cmd_type>,<cid>[,<ConParamTag>,<ConParamValue>],<cid> Bearer profile identifier
  //<cmd_type>0 Close bearer,1 Open bearer,2 Query bearer,3 Set bearer parameters,4 Get bearer parameters
  delay(3000);                        //"APN" Access point name string: max 64chars,"USER"max32 chars,"PWD"max 32 chars
  readSimToSerial(85000); //SOS ...85 sec max !!!

  //Step 2
  if (foundOK) {
    sim->println(F("AT+SAPBR=1,1"));//allocating IP address here
    delay(3000);
    readSimToSerial(85000); //SOS ...85 sec max !!!
    out = foundOK;
    ////////////////////////////
#ifdef DEBUGFULL
    if (foundOK) {
      sim->println(F("AT+SAPBR=2,1"));//showing IP address for debug
      delay(100);
      readSimToSerial(5000);
    }
#endif
    ////////////////////////////

  }

  return out;

}
void simPrintPhpField(byte fieldId, long val) {
  sim->print(F("&field"));
  sim->print(fieldId);
  sim->print('=');
  sim->print(val);
}
void simMakeHttpPhpGetSession(String &stri) {

  romReadPhpServer(stri);
  sim->print(F("AT+HTTPPARA=\"URL\",\""));//Max ResponseTime-
  sim->print(stri);
  sim->print('/');


  stri.remove(0, stri.length());
  romReadPhpQuery(stri);
  sim->print(stri);

  stri.remove(0, stri.length());
  romReadThingSpeakKey(stri);
  sim->print(stri);

  stri.remove(0, stri.length()); //clearing



  byte tempiter = iter;
  if (tempiter > 0) {
    tempiter--;
  }
  else {
    tempiter = SensationsPerSms - 1; //last
  }
  //ser.print("php_itering ");ser.println(tempiter);

  int posOfFreshReading = tempiter * ScalesCanHandle;
  int i = 0;
  while (i < ScalesCanHandle) {
    if (romReadScaleEnabled(i)) {
      simPrintPhpField(i + 1, StoredWeights[posOfFreshReading]);//&field=[1,5] scale[0,4]
    }
    ++posOfFreshReading;
    ++i;
  }
  if (romReadDhtEnabled()) {
    ++i;
    //combining temp and hum....
    /////////////////////////
    /////////////////////////

    int cc = temperature[tempiter];
    int hh = humidity[tempiter];

    if (hh > 99) {
      hh = 99;
    }
    if (cc < 0) {
      hh = -hh;
    }
    int th = cc;
    th *= 100;
    th += hh; //if 100 will add 1 celsiu and seems as zero!
    simPrintPhpField(i, th);//&field=6 (temperature & humidity)

    //replaced from above to just have one empty space on ThingSpeak.....
    //simPrintPhpField(i, temperature[tempiter]);//&field=6 (temperature)
    //++i;
    //simPrintPhpField(i, humidity[tempiter]);//&field=7 (humidity)
  }

  long newFutureSendingSignalAlso=lastSimBatteryRead;
  #ifdef SIGNAL_ALSO
  newFutureSendingSignalAlso=lastSignal;
  newFutureSendingSignalAlso*=10000l;
  newFutureSendingSignalAlso+=lastSimBatteryRead;
  #endif
  
  simPrintPhpField(8, newFutureSendingSignalAlso);//&field=8 (running voltage)
#ifdef LATCH_MAIN_RELAY
  simPrintPhpField(7, powerReadingTranslated0);//true / translated voltage
  //simPrintPhpField(5,PowerReading);//temp for debug..
#endif


  sim->print('\"');
  sim->println();//AT+HTTPPARA...//Max ResponseTime-

  delay(100);
  sim->println();
  readSimToSerial(2000);

  sim->println(F("AT+HTTPACTION=0"));//get action Max ResponseTimeAbout 5 seconds in test, dependence on network status and the size of request website
  delay(100);
  readSimToSerial(5000);//this OK may followed by HTTP answer like +HTTPACTION: 0,200,2 from wgere we can read if 200 succesfull
  if (foundOK) {

    readSimAnswer(1000 * romReadSimSecWaitBeforePowerOff(), ',', true); //HTTPACTION: 0,603,0 or HTTPACTION: 0,200,0



    ++success;//state 5 OK get action
    /*
      sim.println("AT+HTTPREAD");//this no need in 'our' case ...
      delay(100);
      readSimToSerial(4000);
    */
#ifdef DEBUG_SIM_ANSWER
    ser.println();//this will flush also what previuslly read..
#endif

  }

}

void sendDailySms(byte smsMode) {
  if (smsMode > 0) {
    success = 0; //state 0 beginning


    String phone;
    if (smsMode % 2 == 1) {
      //ser.println("sms to Primary");
      romReadSmsPrimaryNumber(phone);
      smsStoredWeightsAt(phone);
    }
    phone.remove(0, phone.length());
    smsMode >>= 1;
    if (smsMode % 2 == 1) {
      //ser.println("sms to Secondary");
      romReadSmsSecondaryNumber(phone);
      smsStoredWeightsAt(phone);
    }
    //}
#ifdef DEBUG
    ser.print(F(" sms:")); ser.print(success);//zero if call instead of sms....
#endif
  }
}

void makeCallAndHungup(String & phone, unsigned long hungupAfter) {
  sim->print(F("ATD+ +"));//make a call//Max Response Time 20s
  sim->print(phone);//to this phone....command
  sim->println(';');
  //readSimToSerial(20000);
  //ser.println(foundOK);
  delay(hungupAfter);//call for some seconds.Basically is NOT some seconds...must fix but no care for this...
  sim->println(F("ATH"));//Max Response Time 20s ALSO !!!!!!!!!!!!!!!!
  delay(1000);

}
void smsStoredWeightsAt(String & phone) {
#ifdef DEBUG
  ser.print(' '); ser.print(phone);
#endif


  if (romReadCallInsteadSms()) {
    makeCallAndHungup(phone, 12000); //call for 12 seconds.Basically is NOT 12 seconds...must fix but no care for this...
  }
  else {


    //opening sms dialog
    sim->println(F("AT+CMGF=1"));//Max ResponseTime-
    delay(100);
    readSimToSerial(2000);

    //AT+CMGS Max Response Time 60s
    sim->print(F("AT+CMGS=\"")); sim->print(phone); sim->println('\"');
    delay(100);

    //sim.println("sms data here");
    smsStoredWeights();

    //closing sms dialog
    sim->write(26);
    sim->println();

    delay(1000);
    readSimToSerial(60000);
    if (foundOK) {

#ifdef DEBUG
      ser.print(F(" ok "));
#endif
      ++success;
    }
  }


}
/*(example of sms)
   2 (2nd day)(the readings NOW when iter =0)
   19C72%
   154-1(154 grams -1 from yesterday same time reading)

   -6 (the readings -6 hours from now,iter was 3)
   20C75#
   154-1

   -12(the readings -12 hours from now,iter was 2)
   20C75#
   154-1

   -18(the readings -18 hours from now,iter was 1)
   20C74#
   155-0
*/
void smsStoredWeights() {
  int i = 0, timThe = 0, index = iter, half = SensationsPerSms * ScalesCanHandle;
  int posOfFreshReading;
  long prev, neo;
  bool dhtEnabled = romReadDhtEnabled();
  sim->println(currDay);

  while (i < SensationsPerSms) {
    if (i > 0) {
      sim->println(timThe);
    }

    --index; if (index < 0) {
      index += SensationsPerSms;
    }
    posOfFreshReading = index * ScalesCanHandle;

    if (dhtEnabled) {
      neo = (int)temperature[index];
      if (neo >= 0) {
        sim->print(neo); sim->print('C');
      }
      neo = (int)humidity[index];
      if (neo >= 0) {
        sim->print(neo); sim->print('%');
        //++prev;
      }

      sim->println();
    }


    currScale = 0;
    while (currScale < ScalesCanHandle) {
      neo = StoredWeights[posOfFreshReading];
      if (neo != DISABLED_CHAR && neo != ERROR_CHAR) {

        //this may no need finally....its more confusing because user wants also to see scale number in every line
        //but this is impossible because of sms shrinkage
        //if(not have 4 or 5 scales to send data....){output.print(currScale);output.print('#');}


        sim->print(neo);
        prev = StoredWeights[posOfFreshReading + half];
        if (prev != DISABLED_CHAR && prev != ERROR_CHAR) {
          neo -= prev;
          if (neo >= 0) {
            sim->print('+');
          }

          //neo/=10;//char roundingDirection;?...not useing
          sim->print(neo);
        }
        else {
          sim->print('?'); ///previus was error or disabled?...(disabled?!!!)
        }
        sim->println();
      }//else not showing offcource on sms
      ++posOfFreshReading;
      ++currScale;
    }

    ++i;
    timThe -= (24 / SensationsPerSms); // -6th hour on default
    if (i < SensationsPerSms) {
      sim->println();//1 line char more for easy read sms from human between readings
    }
  }
}
