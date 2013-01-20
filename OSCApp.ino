
/*
 * OSCApp - Oly Leung
 *
 * This sketch reads from two I2C devices, a gyroscope and accelerometer
 * These readings, three each, six in all, are given as integers which are 
 * formatted into an OSC datagram and sent via UDP
 */
#include "udpapp.h"
#include <WiShield.h>
#include "uip-conf.h"
#include <ArdOSCForWiFlyHQ.h>
#include <Wire.h>
#include <WiShield.h>

#define WIRELESS_MODE_INFRA  1
#define WIRELESS_MODE_ADHOC	2
#define MMA7660addr   0x4c
#define MMA7660_X     0x00
#define MMA7660_Y     0x01
#define MMA7660_Z     0x02
#define MMA7660_TILT  0x03
#define MMA7660_SRST  0x04
#define MMA7660_SPCNT 0x05
#define MMA7660_INTSU 0x06
#define MMA7660_MODE  0x07
#define MMA7660_SR    0x08
#define MMA7660_PDET  0x09
#define MMA7660_PD    0x0A
#define WHO	0x00
#define	SMPL	0x15
#define DLPF	0x16
#define INT_C	0x17
#define INT_S	0x1A
#define	TMP_H	0x1B
#define	TMP_L	0x1C
#define	GX_H	0x1D
#define	GX_L	0x1E
#define	GY_H	0x1F
#define	GY_L	0x20
#define GZ_H	0x21
#define GZ_L	0x22
#define PWR_M	0x3E
#define GYRO_ADDRESS 0x68
#define WIRELESS_MODE_INFRA	1
#define WIRELESS_MODE_ADHOC	2
typedef uint8_t u8_t;
// Wireless configuration parameters ----------------------------------------
unsigned char local_ip[] = {
  192,168,1,2};	// IP address of WiShield
unsigned char dest_ip[] ={
  192,168,1,100}; //{255,255,255,255};
unsigned char gateway_ip[] = {
  192,168,1,1};	// router or gateway IP address
unsigned char subnet_mask[] = {
  255,255,255,0};	// subnet mask for the local network
const prog_char ssid[] PROGMEM = {
  "YOURSSID"};		// max 32 bytes

unsigned char security_type = 0;	// 0 - open; 1 - WEP; 2 - WPA; 3 - WPA2

// WPA/WPA2 passphrase
const prog_char security_passphrase[] PROGMEM = {
  "YOURROUTERPASSWORD"};	// max 64 characters

// WEP 128-bit keys
// sample HEX keys
prog_uchar wep_keys[] PROGMEM = {	
  0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d,	// Key 0
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	0x00,	// Key 1
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	0x00,	// Key 2
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	0x00	// Key 3
};

// setup the wireless mode
// infrastructure - connect to AP
// adhoc - connect to another WiFi device
unsigned char wireless_mode = WIRELESS_MODE_INFRA;

unsigned char ssid_len;
unsigned char security_passphrase_len;
int counter;
int accX;
int accY;
int accZ;
int gyroX;
int gyroY;
int gyroZ;
OSCClient client;
//---------------------------------------------------------------------------

void mma7660_init(void)
{
  // Wire.begin(MMA7660addr);
  Wire.beginTransmission( MMA7660addr);
  Wire.write(MMA7660_MODE);   
  Wire.write(byte(0x00));
  Wire.endTransmission();

  Wire.beginTransmission( MMA7660addr);
  Wire.write(MMA7660_SR);   
  Wire.write(byte(0x07));  //   Samples/Second Active and Auto-Sleep Mode
  Wire.endTransmission();

  Wire.beginTransmission( MMA7660addr);
  Wire.write(MMA7660_MODE);   
  Wire.write(0x01);//active mode
  Wire.endTransmission();
}

void getSensorData()
{
  unsigned char val[3];
  int count = 0;
  val[0] = val[1] = val[2] = 64;
  Wire.requestFrom(MMA7660addr, 3);    // request 3 bytes from slave device 0x4c

  while(Wire.available())  
  {
    if(count < 3)
      while ( val[count] > 63 )  // reload the damn thing it is bad
      {
        val[count] = Wire.read();

      }
    count++;
  }

  // transform the 7 bit signed number into an 8 bit signed number.
  accX= ((char)(val[0]<<2))/4;
  accY = ((char)(val[1]<<2))/4;
  accZ = ((char)(val[2]<<2))/4;

  gyroX = (ITG3200Read(GX_H,GX_L));
  gyroY = (ITG3200Read(GY_H,GY_L));
  gyroZ = (ITG3200Read(GZ_H,GZ_L));
 /* Serial.println("got data");
  Serial.println(accX);
  Serial.println(accY);
  Serial.println(accZ);
  Serial.println(gyroX);
  Serial.println(gyroY);
  Serial.println(gyroZ);*/
}


char reading = 0;
char ITG3200Readbyte(unsigned char address)
{
  char data;

  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write((address));
  Wire.endTransmission();
  Wire.requestFrom(GYRO_ADDRESS,1);
  if (Wire.available()>0)
  {
    data = Wire.read();
  }
  return data;

  Wire.endTransmission();
}

char ITG3200Read(unsigned char addressh,unsigned char addressl)
{
  char data;

  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write((addressh));
  Wire.endTransmission();
  Wire.requestFrom(GYRO_ADDRESS,1);
  if (Wire.available()>0)
  {
    data = Wire.read();
  }
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write((addressl));
  Wire.endTransmission();
  if (Wire.available()>0)
  {
    data |= Wire.read()<<8;
  }
  return data;


  //	   Wire.endTransmission();
}



void Gyro_Init(void)
{
  //Wire.begin(GYRO_ADDRESS);
  Wire.begin();

  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(0x3E);
  Wire.write(0x80);  //send a reset to the device
  Wire.endTransmission(); //end transmission

    Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(0x15);
  Wire.write(0x00);   //sample rate divider
  Wire.endTransmission(); //end transmission

    Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(0x16);
  Wire.write(0x18); // ±2000 degrees/s (default value)
  Wire.endTransmission(); //end transmission
}
void setup()
{
 // Serial.begin(9600);  
  //Serial.println("Spatial Awareness Circuit (SAC) v0.8");
  //Serial.println("Booting Sensors...");
  client = OSCClient();
  Gyro_Init();	
  mma7660_init();        
  //Serial.println("Booting Wifi...");
  WiFi.init(0);
  //Serial.println("Wifi Connected");
}

void loop()
{
  getSensorData();
  uint16_t msgSize;
  uint8_t* sendData;
  OSCMessage accMsg = OSCMessage();
  
   accMsg.beginMessage("/OSCAddress");
   accMsg.addArgInt32(accX);
   accMsg.addArgInt32(accY);
   accMsg.addArgInt32(accZ);
   accMsg.addArgInt32(gyroX);
   accMsg.addArgInt32(gyroY);
   accMsg.addArgInt32(gyroZ);
   
   msgSize = accMsg.getMessageSize();
   
   sendData=client.send(&accMsg);
 
   sendOSC(sendData, msgSize);
   WiFi.run();
   accMsg.flush();
   free(sendData);
   msgSize = 0; 
   delay(150);
}
