/*
 * MPU6050 I2C example
 * Prints acceleration, angular velocity, temperature
 */

#include "math.h"
#include "mbed.h"
#include "MadgwickAHRS.h"
#include "MPU6050.h"

#define PI                3.14159265358979323846264338327950288419716939937510

int timer_cnt = 0;

DigitalOut myled(LED1);
Serial pc(SERIAL_TX, SERIAL_RX); // PA_2, PA_3
//Ticker timer_int;
Madgwick filter;
MPU6050 sensor;

/*void timer_int_handler(void)
{
  //timer_cnt++;
  //timer_cnt = timer_cnt%1000;
  read_MPU6050();
  read_HMC5883L();
  myled = !myled;
}*/

int main()
{
  sensor.begin();
  filter.begin(25);
  //timer_int.attach(&timer_int_handler, 0.02);

  while (1) {
    sensor.read();
    filter.updateIMU(sensor.gx(), sensor.gy(), sensor.gz(), sensor.ax(), sensor.ay(), sensor.az());

    // Display result
    //pc.printf("acc = [%f, %f, %f] gyro = [%f, %f, %f] temp = %f\n", acc_val[0], acc_val[1], acc_val[2], gyro_val[0], gyro_val[1], gyro_val[2], temp_val);
    //pc.printf("acc = [%f, %f, %f] gyro = [%f, %f, %f] mag = [%f %f %f] temp = %f\n", acc_val[0], acc_val[1], acc_val[2], gyro_val[0], gyro_val[1], gyro_val[2], mag_val[0], mag_val[1], mag_val[2], temp_val);
    pc.printf("%f %f %f\n", filter.getRoll(), filter.getPitch(), filter.getYaw());
    myled = !myled;
    wait_ms(10);
  }
}
