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

// GYRO
#define GYRO_XOUT_H 0x33

// MAGN
#define MAG_WIA2 0x01
#define MAG_ST1 0x10
#define MAG_HXL 0x11
#define MAG_CNTL2 0x31
#define MAG_CNTL3 0x32

// BMP
#define BMP3_CHIP_ID_REG 0x00
#define BMP3_ERR_REG 0x02
#define BMP3_STATUS_REG 0x03
#define BMP3_DATA_0 0x04
#define BMP3_DATA_3 0x07
#define BMP3_PWR_CTRL 0x1B
#define BMP3_OSR 0x1C
#define BMP3_ODR 0x1D
#define BMP3_CONFIG 0x1F
#define BMP3_CALIB_DATA 0x31
#define BMP3_CMD 0x7E

class ICM20948 {
private:
  uint8_t address = 0x68;

  // ACCEL
  int16_t xaux_acc;
  int16_t yaux_acc;
  int16_t zaux_acc;

  static constexpr float ACCEL_G = 9.80665f;
  static constexpr float ACCEL_SENSITIVITY_16G = 2048.0f;

  float X_accel;
  float Y_accel;
  float Z_accel;

  // GYRO
  int16_t xaux_gyro;
  int16_t yaux_gyro;
  int16_t zaux_gyro;

  static constexpr float GYRO_SENSITIVITY_2000DPS = 16.4f;

  float X_gyro;
  float Y_gyro;
  float Z_gyro;
public:
  bool begin();
  bool readAccel();
  bool readGyro();

  float getX_accel();
  float getY_accel();
  float getZ_accel();

  float getX_gyro();
  float getY_gyro();
  float getZ_gyro();
};

class AK09916 {
private:
  uint8_t address = 0x0C;
  static constexpr float MAG_SENSITIVITY = 0.15f;

  float X_magn;
  float Y_magn;
  float Z_magn;
public:
  bool begin();
  bool readMagn();

  float getX_magn();
  float getY_magn();
  float getZ_magn();
};

class BMP388 {
private:
  uint8_t address = 0x76;

  struct {
    uint16_t par_t1;
    uint16_t par_t2;
    int8_t par_t3;
    int16_t par_p1;
    int16_t par_p2;
    int8_t par_p3;
    int8_t par_p4;
    uint16_t par_p5;
    uint16_t par_p6;
    int8_t par_p7;
    int8_t par_p8;
    int16_t par_p9;
    int8_t par_p10;
    int8_t par_p11;
    float t_lin;
  } calib;

  float temperature;
  float pressure;

  void readCalibration();
  float compensateTemperature(uint32_t uncomp_temp);
  float compensatePressure(uint32_t uncomp_press);

public:
  bool begin();
  bool readSensor();
  float getTemperature();
  float getPressure();
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
  while (Wire.available() < 6) {
    if (temp + 10 < micros())
      break;
  }

  xaux_acc = Wire.read() << 8 | Wire.read();
  yaux_acc = Wire.read() << 8 | Wire.read();
  zaux_acc = Wire.read() << 8 | Wire.read();

  X_accel = float(xaux_acc / ACCEL_SENSITIVITY_16G) * ACCEL_G;
  Y_accel = float(yaux_acc / ACCEL_SENSITIVITY_16G) * ACCEL_G;
  Z_accel = float(zaux_acc / ACCEL_SENSITIVITY_16G) * ACCEL_G;
}

float ICM20948::getX_accel() {
  return X_accel;
}

float ICM20948::getY_accel() {
  return Y_accel;
}

float ICM20948::getZ_accel() {
  return Z_accel;
}

bool ICM20948::readGyro() {
  Wire.beginTransmission(address);
  Wire.endTransmission();
  Wire.beginTransmission(address);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission();

  Wire.requestFrom(address, uint8_t(6));
  unsigned long temp = micros();
  while (Wire.available() < 6) {
    if (temp + 10 < micros())
      break;
  }

  xaux_gyro = Wire.read() << 8 | Wire.read();
  yaux_gyro = Wire.read() << 8 | Wire.read();
  zaux_gyro = Wire.read() << 8 | Wire.read();

  X_gyro = float(xaux_gyro) / GYRO_SENSITIVITY_2000DPS;
  Y_gyro = float(yaux_gyro) / GYRO_SENSITIVITY_2000DPS;
  Z_gyro = float(zaux_gyro) / GYRO_SENSITIVITY_2000DPS;
}

float ICM20948::getX_gyro() {
  return X_gyro;
}

float ICM20948::getY_gyro() {
  return Y_gyro;
}

float ICM20948::getZ_gyro() {
  return Z_gyro;
}

bool AK09916::begin() {
  Wire.beginTransmission(0x68);
  Wire.write(REG_BANK_SEL);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x03);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(INT_PIN_CFG);
  Wire.write(0x02);
  Wire.endTransmission();

  Wire.beginTransmission(address);
  Wire.write(MAG_CNTL3);
  Wire.write(0x01);
  Wire.endTransmission();

  delayMicroseconds(500);

  Wire.beginTransmission(address);
  Wire.write(MAG_CNTL2);
  Wire.write(0x08);
  Wire.endTransmission();
}

bool AK09916::readMagn() {
  Wire.beginTransmission(address);
  Wire.write(MAG_HXL);
  Wire.endTransmission(false);

  Wire.requestFrom(address, uint8_t(8));

  unsigned long timeout = micros();
  while (Wire.available() < 8) {
    if (micros() - timeout > 1000) return false;
  }

  int16_t x_raw = Wire.read() | (Wire.read() << 8);
  int16_t y_raw = Wire.read() | (Wire.read() << 8);
  int16_t z_raw = Wire.read() | (Wire.read() << 8);
  Wire.read();
  Wire.read();

  X_magn = float(x_raw) * MAG_SENSITIVITY;
  Y_magn = float(y_raw) * MAG_SENSITIVITY;
  Z_magn = float(z_raw) * MAG_SENSITIVITY;
}

float AK09916::getX_magn() {
  return X_magn;
}

float AK09916::getY_magn() {
  return Y_magn;
}

float AK09916::getZ_magn() {
  return Z_magn;
}

bool BMP388::begin() {
  Wire.beginTransmission(address);
  Wire.write(BMP3_CMD);
  Wire.write(0xB6);
  Wire.endTransmission();

  delay(2);

  Wire.beginTransmission(address);
  Wire.write(BMP3_CHIP_ID_REG);
  Wire.endTransmission();
  Wire.requestFrom(address, (uint8_t)1);
  if (Wire.read() != 0x50) return false;

  readCalibration();

  Wire.beginTransmission(address);
  Wire.write(BMP3_OSR);
  Wire.write(0x03);
  Wire.endTransmission();

  Wire.beginTransmission(address);
  Wire.write(BMP3_PWR_CTRL);
  Wire.write(0x33);
  Wire.endTransmission();

  delay(25);

  return true;
}

void BMP388::readCalibration() {
  uint8_t b[21];
  Wire.beginTransmission(address);
  Wire.write(BMP3_CALIB_DATA);
  Wire.endTransmission();
  Wire.requestFrom(address, (uint8_t)21);
  for (int i = 0; i < 21; i++) b[i] = Wire.read();

  calib.par_t1 = (uint16_t)b[1] << 8 | b[0];
  calib.par_t2 = (uint16_t)b[3] << 8 | b[2];
  calib.par_t3 = (int8_t)b[4];
  calib.par_p1 = (int16_t)((b[6] << 8) | b[5]);
  calib.par_p2 = (int16_t)((b[8] << 8) | b[7]);
  calib.par_p3 = (int8_t)b[9];
  calib.par_p4 = (int8_t)b[10];
  calib.par_p5 = (uint16_t)b[12] << 8 | b[11];
  calib.par_p6 = (uint16_t)b[14] << 8 | b[13];
  calib.par_p7 = (int8_t)b[15];
  calib.par_p8 = (int8_t)b[16];
  calib.par_p9 = (int16_t)((b[18] << 8) | b[17]);
  calib.par_p10 = (int8_t)b[19];
  calib.par_p11 = (int8_t)b[20];
}

bool BMP388::readSensor() {
  Wire.beginTransmission(address);
  Wire.write(BMP3_STATUS_REG);
  Wire.endTransmission(false);
  Wire.requestFrom(address, (uint8_t)1);

  uint8_t status = Wire.read();
  if (!(status & 0x20)) return false;

  Wire.beginTransmission(address);
  Wire.write(BMP3_DATA_0);
  Wire.endTransmission(false);

  if (Wire.requestFrom(address, (uint8_t)6) == 6) {
    uint32_t p0 = Wire.read(), p1 = Wire.read(), p2 = Wire.read();
    uint32_t t0 = Wire.read(), t1 = Wire.read(), t2 = Wire.read();

    uint32_t adc_p = p0 | (p1 << 8) | (p2 << 16);
    uint32_t adc_t = t0 | (t1 << 8) | (t2 << 16);

    if (adc_p == 0) return false;

    temperature = compensateTemperature(adc_t);
    pressure = compensatePressure(adc_p);
    return true;
  }
  return false;
}

float BMP388::compensateTemperature(uint32_t uncomp_temp) {
  double partial_data1 = (double)(uncomp_temp - (calib.par_t1 * 256.0));
  double partial_data2 = (double)(partial_data1 * (calib.par_t2 / 1073741824.0));
  calib.t_lin = (float)(partial_data2 + (partial_data1 * partial_data1) * (calib.par_t3 / 281474976710656.0));
  return calib.t_lin;
}

float BMP388::compensatePressure(uint32_t uncomp_press) {
  double partial_data1, partial_data2, partial_data3, partial_data4, partial_out1, partial_out2;

  partial_data1 = calib.par_p6 / 64.0;
  partial_data2 = calib.par_p7 / 256.0;
  partial_data3 = calib.par_p8 / 32768.0;
  partial_out1 = (calib.par_p5 * 8.0) + (partial_data1 * calib.t_lin) + (partial_data2 * calib.t_lin * calib.t_lin) + (partial_data3 * calib.t_lin * calib.t_lin * calib.t_lin);

  partial_data1 = calib.par_p2 / 536870912.0;
  partial_data2 = calib.par_p3 / 4294967296.0;
  partial_data3 = calib.par_p4 / 137438953472.0;
  partial_out2 = (double)uncomp_press * ((calib.par_p1 / 1048576.0) + (partial_data1 * calib.t_lin) + (partial_data2 * calib.t_lin * calib.t_lin) + (partial_data3 * calib.t_lin * calib.t_lin * calib.t_lin));

  partial_data1 = (double)uncomp_press * (double)uncomp_press;
  partial_data2 = (calib.par_p9 / 281474976710656.0) + (calib.par_p10 / 281474976710656.0) * calib.t_lin;
  partial_data3 = partial_data1 * partial_data2;
  partial_data4 = partial_data3 + ((double)uncomp_press * (double)uncomp_press * (double)uncomp_press) * (calib.par_p11 / 36893488147419103232.0);

  return (float)(partial_out1 + partial_out2 + partial_data4);
}

float BMP388::getTemperature() {
  return temperature;
}

float BMP388::getPressure() {
  return pressure;
}

/* Fim CPP */

ICM20948 myICM;
AK09916 myAK;
BMP388 myBMP;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  myICM.begin();
  myAK.begin();
  myBMP.begin();
  Serial.print("X.accel\tY.accel\tZ.accel\t");
  Serial.print("X.gyro\tY.gyro\tZ.gyro\t");
  Serial.print("X.magn\tY.magn\tZ.magn\t");
  Serial.print("C.bmp\tP.bmp");
  Serial.println();
}

void loop() {
  myICM.readAccel();
  myICM.readGyro();
  myAK.readMagn();
  myBMP.readSensor();

  Serial.print(myICM.getX_accel());
  Serial.print("\t");
  Serial.print(myICM.getY_accel());
  Serial.print("\t");
  Serial.print(myICM.getZ_accel());
  Serial.print("\t");

  Serial.print(myICM.getX_gyro());
  Serial.print("\t");
  Serial.print(myICM.getY_gyro());
  Serial.print("\t");
  Serial.print(myICM.getZ_gyro());
  Serial.print("\t");

  Serial.print(myAK.getX_magn());
  Serial.print("\t");
  Serial.print(myAK.getY_magn());
  Serial.print("\t");
  Serial.print(myAK.getZ_magn());
  Serial.print("\t");

  Serial.print(myBMP.getTemperature());
  Serial.print("\t");
  Serial.print(myBMP.getPressure());
  Serial.print("\t");

  Serial.println();
}