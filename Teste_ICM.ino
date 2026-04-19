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
#define BMP388_ADDRESS_DEFAULT 0x77
#define BMP3_REG_DATA 0x04
#define BMP3_REG_PWR_CTRL 0x1B
#define BMP3_REG_OSR 0x1C
#define BMP3_REG_CALIB_DATA 0x31
#define BMP3_REG_CMD 0x7E

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

class BMP388
{
    // Parâmetros de calibração (bmp3_reg_calib_data)
    uint16_t par_t1, par_t2;
    int8_t   par_t3;
    int16_t  par_p1, par_p2;
    int8_t   par_p3, par_p4;
    uint16_t par_p5, par_p6;
    int8_t   par_p7, par_p8;
    int16_t  par_p9;
    int8_t   par_p10, par_p11;
    
    double t_lin; // Variável intermediária de temperatura para o cálculo da pressão
    uint8_t address;

public:
    float celcius;
    float pascal;
    bool state;
    
    unsigned long lastReadT;
    unsigned long lastWorkT;
    unsigned long recalibrateT;

    BMP388(uint8_t addr = BMP388_ADDRESS_DEFAULT, float recalT = 0.1);
    void begin();
    bool readAll();

private:
    double compensate_T(uint32_t uncomp_temp);
    double compensate_P(uint32_t uncomp_press);
};

/* Fim Header */

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

BMP388::BMP388(uint8_t addr, float recalT) {
  address = addr;
  recalibrateT = recalT * 1000000;  // Converte segundos para microssegundos
  state = false;
  celcius = 0;
  pascal = 0;
  lastReadT = 0;
  lastWorkT = 0;
}

void BMP388::begin() {
  // 1. Reset de Software (Opcional, mas recomendado)
  Wire.beginTransmission(address);
  Wire.write(BMP3_REG_CMD);
  Wire.write(0xB6);
  Wire.endTransmission();
  delay(10);

  // 2. Leitura dos 21 bytes de calibração a partir do registrador 0x31
  Wire.beginTransmission(address);
  Wire.write(BMP3_REG_CALIB_DATA);
  Wire.endTransmission();
  Wire.requestFrom(address, uint8_t(21));

  unsigned long start = micros();
  while (Wire.available() < 21) {
    if (micros() - start > 5000) break;
  }

  // Mapeamento dos dados de calibração conforme bmp3.c
  par_t1 = Wire.read() | (Wire.read() << 8);
  par_t2 = Wire.read() | (Wire.read() << 8);
  par_t3 = (int8_t)Wire.read();
  par_p1 = (int16_t)(Wire.read() | (Wire.read() << 8));
  par_p2 = (int16_t)(Wire.read() | (Wire.read() << 8));
  par_p3 = (int8_t)Wire.read();
  par_p4 = (int8_t)Wire.read();
  par_p5 = Wire.read() | (Wire.read() << 8);
  par_p6 = Wire.read() | (Wire.read() << 8);
  par_p7 = (int8_t)Wire.read();
  par_p8 = (int8_t)Wire.read();
  par_p9 = (int16_t)(Wire.read() | (Wire.read() << 8));
  par_p10 = (int8_t)Wire.read();
  par_p11 = (int8_t)Wire.read();

  // 3. Configuração de Oversampling (Padrão: x1)
  Wire.beginTransmission(address);
  Wire.write(BMP3_REG_OSR);
  Wire.write(0x00);
  Wire.endTransmission();

  // 4. Modo de Operação: Normal (0x30), Temp ON (0x02), Press ON (0x01)
  Wire.beginTransmission(address);
  Wire.write(BMP3_REG_PWR_CTRL);
  Wire.write(0x33);
  Wire.endTransmission();
}

bool BMP388::readAll() {
  unsigned long thisReadT = micros();
  Wire.beginTransmission(address);
  state = (Wire.endTransmission() == 0);

  if (state) {
    // Lógica de recalibração periódica do padrão BMP280
    if (thisReadT - lastWorkT > recalibrateT) {
      begin();
    }

    // Leitura de 6 bytes: Dados brutos de Pressão (3 bytes) e Temperatura (3 bytes)
    Wire.beginTransmission(address);
    Wire.write(BMP3_REG_DATA);
    Wire.endTransmission();
    Wire.requestFrom(address, uint8_t(6));

    unsigned long start = micros();
    while (Wire.available() < 6) {
      if (micros() - start > 2000) break;
    }

    // No BMP388, os dados são de 24 bits (LSB, LSB, MSB)
    uint32_t adc_p = (uint32_t)Wire.read() | ((uint32_t)Wire.read() << 8) | ((uint32_t)Wire.read() << 16);
    uint32_t adc_t = (uint32_t)Wire.read() | ((uint32_t)Wire.read() << 8) | ((uint32_t)Wire.read() << 16);

    celcius = (float)compensate_T(adc_t);
    pascal = (float)compensate_P(adc_p);

    lastWorkT = thisReadT;
  }
  lastReadT = thisReadT;
  return state;
}

// Funções de compensação em ponto flutuante extraídas de bmp3.c
double BMP388::compensate_T(uint32_t uncomp_temp) {
  double partial_data1 = (double)(uncomp_temp - (double)par_t1);
  double partial_data2 = (double)(partial_data1 * ((double)par_t2 / 1073741824.0));
  t_lin = partial_data2 + (partial_data1 * partial_data1) * ((double)par_t3 / 281474976710656.0);
  return t_lin;
}

double BMP388::compensate_P(uint32_t uncomp_press) {
  double p1, p2, p3, out1, out2, out3;

  p1 = ((double)par_p6 / 64.0) * t_lin;
  p2 = ((double)par_p7 / 256.0) * (t_lin * t_lin);
  p3 = ((double)par_p8 / 32768.0) * (t_lin * t_lin * t_lin);
  out1 = ((double)par_p5 * 8.0) + p1 + p2 + p3;

  p1 = ((double)par_p2 / 536870912.0) * t_lin;
  p2 = ((double)par_p3 / 4294967296.0) * (t_lin * t_lin);
  p3 = ((double)par_p4 / 137438953472.0) * (t_lin * t_lin * t_lin);
  out2 = (double)uncomp_press * (((double)(par_p1 - 16384) / 1048576.0) + p1 + p2 + p3);

  p1 = (double)uncomp_press * (double)uncomp_press;
  p2 = ((double)par_p9 / 281474976710656.0) + ((double)par_p10 / 281474976710656.0) * t_lin;
  out3 = (p1 * p2) + ((p1 * (double)uncomp_press) * ((double)par_p11 / 36893488147419103232.0));

  return out1 + out2 + out3;
}

/* Fim CPP */

ICM20948 myICM;
AK09916 myAK;
BMP388 myBMP(0x77, 0.1);

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

  Serial.print(myBMP.celcius);
  Serial.print("\t");
  Serial.print(myBMP.pascal);
  Serial.print("\t");

  Serial.println();
}