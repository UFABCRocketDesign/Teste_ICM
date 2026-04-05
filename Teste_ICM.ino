/* Começo Header */

#include <Wire.h>

// BEGIN
#define PWR_MGMT_1 0x06
#define PWR_MGMT_2 0x07
#define LP_CONFIG 0x05
#define GYRO_CONFIG_1 0x01
#define GYRO_SMPLRT_DIV 0x00
#define ACCEL_CONFIG 0x14
#define ACCEL_SMPLRT_DIV_1 0x10
#define ACCEL_SMPLRT_DIV_2 0x11
#define INT_PIN_CFG 0x0F
#define REG_BANK_SEL 0x7F

// ACCEL
#define ACCEL_XOUT_H 0x2D

class ICM20948
{
private:
  uint8_t address = 0x68;

  // ACCEL
  uint16_t xaux_acc;
  uint16_t yaux_acc;
  uint16_t zaux_acc;

  float X;
  float Y;
  float Z;

  static constexpr float ACCEL_G = 9.80665f;
public:
  bool begin();
  bool readAccel();
  bool readGyro(); 
  bool readMagn();

  float getX();
  float getY();
  float getZ();
};

/* Fim Header */

/* --- */

/* Começo CPP */

bool ICM20948::begin() {
	Wire.beginTransmission(address);
	Wire.write(REG_BANK_SEL);               
	Wire.write((0x00 & 0x03) << 4);                
	Wire.endTransmission();
  
  Wire.beginTransmission(address);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x01);
  Wire.endTransmission();
  
  Wire.beginTransmission(address);
  Wire.write(PWR_MGMT_2);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(address);
  Wire.write(LP_CONFIG);
  Wire.write(0x00);
  Wire.endTransmission();
  
  Wire.beginTransmission(address);
  Wire.write(REG_BANK_SEL);               
  Wire.write((0x02 & 0x03) << 4);                
  Wire.endTransmission();

  Wire.beginTransmission(address);
  Wire.write(GYRO_CONFIG_1);
  Wire.write(0x1F);
  Wire.endTransmission();

  Wire.beginTransmission(address);
  Wire.write(GYRO_SMPLRT_DIV);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(address);
  Wire.write(ACCEL_CONFIG);
  Wire.write(0x1F);
  Wire.endTransmission();

  Wire.beginTransmission(address);
  Wire.write(ACCEL_SMPLRT_DIV_1);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(address);
  Wire.write(ACCEL_SMPLRT_DIV_2);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(address);
  Wire.write(REG_BANK_SEL);               
  Wire.write((0x00 & 0x03) << 4);                
  Wire.endTransmission();

  Wire.beginTransmission(address);
  Wire.write(INT_PIN_CFG);
  Wire.write(0x30);
  Wire.endTransmission();
}

bool ICM20948::readAccel() {
  Wire.beginTransmission(address);
  Wire.endTransmission();
  Wire.beginTransmission(address);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission();

  Wire.requestFrom(address, uint8_t(6));
  unsigned long temp = micros();
  while (Wire.available() < 6)
  {
    if (temp + 10 < micros())
      break;
  }

  xaux_acc = Wire.read() << 8 | Wire.read();
  yaux_acc = Wire.read() << 8 | Wire.read();
  zaux_acc = Wire.read() << 8 | Wire.read();

  X = float(xaux_acc) * ACCEL_G;
  Y = float(yaux_acc) * ACCEL_G;
  Z = float(zaux_acc) * ACCEL_G;
}


bool ICM20948::readGyro() {

}

bool ICM20948::readMagn() {

}

float ICM20948::getX() {
  return X;
}

float ICM20948::getY() {
  return Y;
}

float ICM20948::getZ() {
  return Z;
}

/* Fim CPP */

ICM20948 myICM;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  myICM.begin();
  Serial.println("X.accel\tY.accel\tZ.accel");
}

void loop() {
  myICM.readAccel();
  Serial.print(myICM.getX()); Serial.print("\t");
  Serial.print(myICM.getY()); Serial.print("\t");
  Serial.print(myICM.getZ()); Serial.print("\t");
  Serial.println();
}
