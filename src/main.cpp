/*
 * MPU6050 I2C example
 * Prints acceleration, angular velocity, temperature
 */

#include "mbed.h"
#include "MadgwickAHRS.h"

#define MPU6050_ADDR      0xD0 // 8-bit address = 7-bit address*2 = 0x68*2 = 0xD0
#define HMC5883L_ADDR     0x3C // 8-bit address = 7-bit address*2 = 0x1E*2 = 0x3C
#define MPU6050_PWR				0x6B
#define MPU6050_ACC_CONF  0x1C
#define MPU6050_GYRO_CONF 0x1B
#define HMC5883L_CONF_A 	0x00
#define HMC5883L_CONF_B 	0x01
#define HMC5883L_MODE 		0x02
#define ACC_XOUT_H 				0x3B
#define ACC_XOUT_L 				0x3C
#define ACC_YOUT_H 				0x3D
#define ACC_YOUT_L 				0x3E
#define ACC_ZOUT_H 				0x3F
#define ACC_ZOUT_L 				0x40
#define TEMP_OUT_H        0x41
#define TEMP_OUT_L        0x42
#define GYRO_XOUT_H 			0x43
#define GYRO_XOUT_L 			0x44
#define GYRO_YOUT_H 			0x45
#define GYRO_YOUT_L 			0x46
#define GYRO_ZOUT_H       0x47
#define GYRO_ZOUT_L       0x48
#define MAG_XOUT_H 				0x03
#define MAG_XOUT_L 				0x04
#define MAG_YOUT_H 				0x07
#define MAG_YOUT_L 				0x08
#define MAG_ZOUT_H 				0x05
#define MAG_ZOUT_L 				0x06

#define PI                3.141592653590

char data_write[2];
char acc_read[6];
char temp_read[2];
char gyro_read[6];
char mag_read[6];
int16_t acc_rawval[3];
int16_t temp_rawval;
int16_t gyro_rawval[3];
int16_t mag_rawval[3];
float acc_val[3];
float temp_val;
float gyro_val[3];
float mag_val[3];
int timer_cnt = 0;

Serial pc(SERIAL_TX, SERIAL_RX); // PA_2, PA_3
I2C i2c(I2C_SDA, I2C_SCL); // PB_9, PB_8
DigitalOut myled(LED1);
Ticker timer_int;

void setup_sensor(void);
void read_sensor(void);
void timer_int_handler(void);

int main()
{
  setup_sensor();
  timer_int.attach(&timer_int_handler, 0.02);

  while (1) {
    /*if(timer_cnt%100 == 0) {
      read_sensor();

    }*/
    // Display result
    //pc.printf("acc = [%f, %f, %f] gyro = [%f, %f, %f] mag = [%f %f %f] temp = %f\n", acc_val[0], acc_val[1], acc_val[2], gyro_val[0], gyro_val[1], gyro_val[2], mag_val[0], mag_val[1], mag_val[2], temp_val);
  }
}

void timer_int_handler(void)
{
  //timer_cnt++;
  //timer_cnt = timer_cnt%1000;
  read_sensor();
  myled = !myled;
}

void setup_sensor(void)
{
  int check = 0;
  data_write[0] = MPU6050_PWR;
  data_write[1] = 0x00;
  check += i2c.write(MPU6050_ADDR, data_write, 2, false); // MPU6050 Power ON
  data_write[0] = MPU6050_ACC_CONF;
  check += i2c.write(MPU6050_ADDR, data_write, 2, false)*10; // Acclerometer 16384 LSB/g
  data_write[0] = MPU6050_GYRO_CONF;
  check += i2c.write(MPU6050_ADDR, data_write, 2, false)*100; // Gyroscope 131 LSB/dps
  /*data_write[0] = HMC5883L_MODE;
  check += i2c.write(HMC5883L_ADDR, data_write, 2, false)*1000; // HMC5883L Continuous mode
  data_write[0] = HMC5883L_CONF_A;
  data_write[1] = 0x18;
  check += i2c.write(HMC5883L_ADDR, data_write, 2, false)*10000; // HMC5883L 75Hz mode
  data_write[0] = HMC5883L_CONF_B;
  data_write[1] = 0x60;
  check += i2c.write(HMC5883L_ADDR, data_write, 2, false)*100000; // Magnetometer 1.52 mG/LSB*/
  if(check != 0) {
    while(1) {
      myled = !myled;
      pc.printf("error count: %d\n", check);
      wait_ms(2500);
    }
  }
}

void read_sensor(void)
{
  // Read accelerometer, gyroscope, magnetometer register
  // i2c.read(int address, char *data, int length{#, bool repeated#})
  // i2c.write(int address, const char *data, int length{#, bool repeated#})
  data_write[0] = ACC_XOUT_H;
  i2c.write(MPU6050_ADDR, data_write, 1, true);
  i2c.read(MPU6050_ADDR, acc_read, 6, false);
  data_write[0] = TEMP_OUT_H;
  i2c.write(MPU6050_ADDR, data_write, 1, true);
  i2c.read(MPU6050_ADDR, temp_read, 2, false);
  data_write[0] = GYRO_XOUT_H;
  i2c.write(MPU6050_ADDR, data_write, 1, true);
  i2c.read(MPU6050_ADDR, gyro_read, 6, false);
  /*data_write[0] = MAG_XOUT_H;
  i2c.write(HMC5883L_ADDR, data_write, 1, true);
  i2c.read(HMC5883L_ADDR, mag_read, 6, false);*/

  // Merge MSB and LSB
  acc_rawval[0] = -(int16_t)((uint8_t)acc_read[0] << 8) | ((uint8_t)acc_read[1]);
  acc_rawval[1] = -(int16_t)((uint8_t)acc_read[2] << 8) | ((uint8_t)acc_read[3]);
  acc_rawval[2] = -(int16_t)((uint8_t)acc_read[4] << 8) | ((uint8_t)acc_read[5]);
  temp_rawval = (int16_t)((uint8_t)temp_read[0] << 8) | ((uint8_t)temp_read[1]);
  gyro_rawval[0] = (int16_t)((uint8_t)gyro_read[0] << 8) | ((uint8_t)gyro_read[1]);
  gyro_rawval[1] = (int16_t)((uint8_t)gyro_read[2] << 8) | ((uint8_t)gyro_read[3]);
  gyro_rawval[2] = (int16_t)((uint8_t)gyro_read[4] << 8) | ((uint8_t)gyro_read[5]);
  /*mag_rawval[0] = (int16_t)((uint8_t)mag_read[0] << 8) | ((uint8_t)mag_read[1]);
  mag_rawval[1] = (int16_t)((uint8_t)mag_read[4] << 8) | ((uint8_t)mag_read[5]);
  mag_rawval[2] = (int16_t)((uint8_t)mag_read[2] << 8) | ((uint8_t)mag_read[3]);*/

  // Apply scale factor
  for(int i = 0; i < 3; i++) acc_val[i] = acc_rawval[i]/16384.0*9.8; // unit: m/s/s
  temp_val = temp_rawval/340.0 + 36.53; // unit: degrees Celsius
  for(int i = 0; i < 3; i++) gyro_val[i] = gyro_rawval[i]/131.0*PI/180; // unit: rad/s
  //for(int i = 0; i < 3; i++) mag_val[i] = mag_rawval[i]*1.52; // unit: mG
}
