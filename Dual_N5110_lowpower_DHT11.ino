/*
 * (c)2018 Pawel A. Hernik
 * code for https://youtu.be/a2o2LGGaHMM
 */
 
// *** CONNECTIONS ***
// N5110 LCD from left:
// #1 RST      - Pin 9 / 6
// #2 CS/CE    - Pin 10 / 5
// #3 DC       - Pin 8
// #4 MOSI/DIN - Pin 11
// #5 SCK/CLK  - Pin 13
// #6 VCC      - 3.3-5.0V
// #7 LIGHT (not used here)
// #8 GND

// DHT11 pinout from left:
// VCC DATA NC GND

#define DHT11_PIN 2
#define DHT11_VCC 7

// uncomment to store min/max in EEEPROM
//#define USE_EEPROM

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <EEPROM.h>

// define USESPI in LCD driver header for HW SPI version
#include "N5110_SPI.h"
#if USESPI==1
#include <SPI.h>
#endif
N5110_SPI lcd(9,10,8); // RST,CS,DC
N5110_SPI lcd2(6,5,8); // RST,CS,DC

#include "times_dig_16x24_font.h"
#include "c64enh_font.h"
#include "term9x14_font.h"

char buf[25],buf2[15];
float temp,mint=1000,maxt=-1000;
float minh=1000,maxh=-1000;

long readVcc() 
{
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1125300L / result; // Back-calculate AVcc in mV
  return result;
}


void wrFloat(float f, int addr)
{
#ifdef USE_EEPROM
  unsigned int ui = (f+100)*10;
  EEPROM.write(addr+0, ui&0xff);
  EEPROM.write(addr+1, (ui>>8)&0xff);
#endif
}

float rdFloat(int addr)
{
  unsigned int ui = EEPROM.read(addr) + (EEPROM.read(addr+1)<<8);
  return (ui/10.0)-100.0;
}

enum wdt_time {
	SLEEP_15MS,
	SLEEP_30MS,	
	SLEEP_60MS,
	SLEEP_120MS,
	SLEEP_250MS,
	SLEEP_500MS,
	SLEEP_1S,
	SLEEP_2S,
	SLEEP_4S,
	SLEEP_8S,
	SLEEP_FOREVER
};

ISR(WDT_vect) { wdt_disable(); }

void powerDown(uint8_t time)
{
  ADCSRA &= ~(1 << ADEN);  // turn off ADC
  if(time != SLEEP_FOREVER) { // use watchdog timer
    wdt_enable(time);
    WDTCSR |= (1 << WDIE);	
  }
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // most power saving
  cli();
  sleep_enable();
  sleep_bod_disable();
  sei();
  sleep_cpu();
  // ... sleeping here
  sleep_disable();
  ADCSRA |= (1 << ADEN); // turn on ADC
}

#define DHT_OK		0
#define DHT_CHECKSUM	-1
#define DHT_TIMEOUT	-2
int humidity,hum;
int temp1,temp10;

int readDHT11(int pin)
{
  uint8_t bits[5];
  uint8_t bit = 7;
  uint8_t idx = 0;

  for (int i = 0; i < 5; i++) bits[i] = 0;

  // REQUEST SAMPLE
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delay(18);
  digitalWrite(pin, HIGH);
  delayMicroseconds(40);
  pinMode(pin, INPUT_PULLUP);

  // ACKNOWLEDGE or TIMEOUT
  unsigned int loopCnt = 10000;
  while(digitalRead(pin) == LOW) if(!loopCnt--) return DHT_TIMEOUT;

  loopCnt = 10000;
  while(digitalRead(pin) == HIGH) if(!loopCnt--) return DHT_TIMEOUT;

  // READ OUTPUT - 40 BITS => 5 BYTES or TIMEOUT
  for (int i = 0; i < 40; i++) {
    loopCnt = 10000;
    while(digitalRead(pin) == LOW) if(!loopCnt--) return DHT_TIMEOUT;

    unsigned long t = micros();
    loopCnt = 10000;
    while(digitalRead(pin) == HIGH) if(!loopCnt--) return DHT_TIMEOUT;

    if(micros() - t > 40) bits[idx] |= (1 << bit);
    if(bit == 0) {
      bit = 7;    // restart at MSB
      idx++;      // next byte!
    }
    else bit--;
  }
  // bits[1] and bits[3] are always zero???
  hum    = bits[0];
  //humidity    = map(hum,33,55,50,78);
  humidity = hum;
  temp1  = bits[2];
  temp10 = bits[3];
  if(bits[4] != bits[0]+bits[1]+bits[2]+bits[3]) return DHT_CHECKSUM;
  return DHT_OK;
}

void drawBatt(int x, int y, int wd, int perc)
{
  int w = wd*perc/100;
  lcd.fillWin(x,y,1+w,1,B01111111);
  x+=w+1;
  w=wd-w;
  if(w>0) {
    lcd.fillWin(x,y,w,1,B01000001);
    x+=w;
  }
  lcd.fillWin(x++,y,1,1,B01111111);
  lcd.fillWin(x++,y,1,1,B00011100);
  lcd.fillWin(x++,y,1,1,B00011100);
}


int first=1;

void setup() 
{
  first=1;
  Serial.begin(9600);
  lcd.init();
  lcd2.init();
  lcd.clrScr();
  lcd2.clrScr();
  // separate full init is necessary only for my 2nd LCD
  lcd2.setContrast(0x38); // 0x30
  lcd2.setBias(4); //4
  lcd2.setTemp(1); //0
  for(int i=0;i<14;i++) pinMode(i, OUTPUT); 
  digitalWrite(DHT11_VCC, HIGH);  // enable DHT11
#ifdef USE_EEPROM
  mint=rdFloat(0);
  maxt=rdFloat(2);
  minh=rdFloat(4);
  maxh=rdFloat(6);
#endif
  //Serial.println(mint);
  //Serial.println(maxt);
  if(mint<-40 || mint>100) mint=99;
  if(maxt<-40 || maxt>100) maxt=-99;
  if(minh<0 || minh>100) minh=99;
  if(maxh<0 || maxh>100) maxh=0;
  //CLKPR = 0x80; // lower internal clock frequency to save more energy
  //CLKPR = 0x02; // 0-16MHz, 1-8MHz, 2-4MHz, 3-2MHz, ..
}

void loop() 
{
  int x=8;
  long v=readVcc();
  if(v<2900) {
    lcd2.sleep(true);
    lcd.clrScr();
    lcd.setFont(Term9x14);
    lcd.printStr(ALIGN_CENTER, 0, "Low");
    lcd.printStr(ALIGN_CENTER, 2, "Battery");
    lcd.setFont(c64enh);
    lcd.setDigitMinWd(6);
    dtostrf(v/1000.0,1,3,buf);
    x=lcd.printStr(x, 5, "Vcc: ");
    x=lcd.printStr(x, 5, buf);
    lcd.printStr(x+2, 5, "V");
    powerDown(SLEEP_8S);
    powerDown(SLEEP_8S);
    // disable LCD controller and power down forever to save battery
    lcd.sleep(true);
    powerDown(SLEEP_FOREVER);
  }

  digitalWrite(DHT11_VCC, HIGH);  // enable DHT11
  int ret = readDHT11(DHT11_PIN);   // only positive values - room temperatures
  temp = abs(temp1+temp10/10.0);

  if(ret==DHT_OK) {
    if(temp<mint) { mint=temp; wrFloat(mint,0); }
    if(temp>maxt) { maxt=temp; wrFloat(maxt,2); }
    if(humidity<minh) { minh=humidity; wrFloat(minh,4); }
    if(humidity>maxh) { maxh=humidity; wrFloat(maxh,6); }
    if(first) {
#ifndef USE_EEPROM
      mint=maxt=temp;
      minh=maxh=humidity;
#endif
      first=0;
      lcd.clrScr();
      lcd2.clrScr();
    }
  }

  if(first && ret!=DHT_OK) {
    lcd.clrScr();
    lcd.setFont(Term9x14);
    lcd.printStr(ALIGN_CENTER, 1, "Sensor");
    lcd.printStr(ALIGN_CENTER, 3, "init");
    lcd2.clrScr();
    lcd2.setFont(Term9x14);
    lcd2.printStr(ALIGN_CENTER, 1, "Sensor");
    lcd2.printStr(ALIGN_CENTER, 3, "init");
    powerDown(SLEEP_2S);
    return;
  }

  snprintf(buf,10,"%d",(int)temp);
  lcd.setFont(times_dig_16x24);
  lcd.setDigitMinWd(17);
  x=3;
  x=lcd.printStr(x, 0, buf);
  snprintf(buf,10,":%d",(int)temp10);
  x=lcd.printStr(x, 0, buf);
  lcd.setFont(Term9x14);
  lcd.printStr(x+1, 0, "`C");

  snprintf(buf,10,"%d",humidity);
  lcd2.setFont(times_dig_16x24);
  lcd2.setDigitMinWd(17);
  x=lcd2.printStr(22, 1, buf);
  lcd2.setFont(Term9x14);
  lcd2.printStr(x+2, 2, "%");
  lcd2.setFont(c64enh);
  lcd2.printStr(ALIGN_CENTER, 0, "Humidity");

  lcd.setFont(c64enh);
  lcd.setDigitMinWd(6);
  dtostrf(v/1000.0,1,3,buf);
  drawBatt(6,5,24,constrain(map(v,2900,4200,0,100),0,100));
  lcd.printStr(38, 5, buf);
  lcd.printStr(72, 5, "V");

  buf[0]=0;
  strcat(buf," <"); dtostrf(mint,1,1,buf2); strcat(buf,buf2);
  strcat(buf,"' >"); dtostrf(maxt,1,1,buf2); strcat(buf,buf2); strcat(buf,"' ");
  lcd.printStr(ALIGN_CENTER, 4, buf);

  buf[0]=0;
  strcat(buf," <"); dtostrf(minh,1,0,buf2); strcat(buf,buf2);
  strcat(buf,"% >"); dtostrf(maxh,1,0,buf2); strcat(buf,buf2); strcat(buf,"% ");
  lcd2.printStr(ALIGN_CENTER, 5, buf);

  // raw humidity debugging
  //snprintf(buf,10,"%d",hum);
  //lcd2.printStr(ALIGN_RIGHT, 4, buf);

  lcd.printStr(ALIGN_RIGHT, 2, "    ");
  if(ret==DHT_TIMEOUT) lcd.printStr(ALIGN_RIGHT, 2, "ET!"); else
  if(ret==DHT_CHECKSUM) lcd.printStr(ALIGN_RIGHT, 2, "EC!");
 
  digitalWrite(DHT11_VCC, LOW);  // disable DHT11 to save power
  // sleep for 10*8 seconds
  for(int i=0;i<10;i++)
    powerDown(SLEEP_8S);

  digitalWrite(DHT11_VCC, HIGH);  // enable DHT11
  powerDown(SLEEP_2S); // 2 or 4s delay
}
