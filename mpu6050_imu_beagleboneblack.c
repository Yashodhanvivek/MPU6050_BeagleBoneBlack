//Interfacing MPU6050 with Beaglebone black(BBB) (I2C interface) 
//BBB pin out -> MPU6050 Pinout
// P9.19 > SCL, P9.20 -> SDA, 3.3V external -> Vcc, P9.1> ground 

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>

/*MPU6050 register addresses */

#define MPU6050_IMU_REG_POWER               0x6B // PWR_MGMT_1 pg 8 Register map of MPU6050 datasheet
#define MPU6050_IMU_REG_ACCEL_CONFIG        0x1C // pg  6
#define MPU6050_IMU_REG_GYRO_CONFIG         0x1B // pg 6

/* addresses to read accelerometer x,y,z high and low values */
/* Ref. page 7 of  Register map of MPU6050*/
#define MPU6050_IMU_REG_ACC_X_HIGH          0x3B 
#define MPU6050_IMU_REG_ACC_X_LOW           0x3C
#define MPU6050_IMU_REG_ACC_Y_HIGH          0x3D
#define MPU6050_IMU_REG_ACC_Y_LOW           0x3E
#define MPU6050_IMU_REG_ACC_Z_HIGH          0x3F
#define MPU6050_IMU_REG_ACC_Z_LOW           0x40

/* addresses to read gyroscope x,y,z high and low values */
/* Ref. page 7 of  Register map of MPU6050*/
#define MPU6050_IMU_REG_GYRO_X_HIGH          0x43
#define MPU6050_IMU_REG_GYRO_X_LOW           0x44
#define MPU6050_IMU_REG_GYRO_Y_HIGH          0x45
#define MPU6050_IMU_REG_GYRO_Y_LOW           0x46
#define MPU6050_IMU_REG_GYRO_Z_HIGH          0x47
#define MPU6050_IMU_REG_GYRO_Z_LOW           0x48

/*
 * Different full scale ranges for acc and gyro
 * refer table 6.2 and 6.3 in the document MPU-6000 and MPU-6050 Product Specification Revision 3.4
 *
 */
#define ACCL_FS_SENSITIVITY_0					16384
#define ACCL_FS_SENSITIVITY_1		            8192
#define ACCL_FS_SENSITIVITY_2		            4096
#define ACCL_FS_SENSITIVITY_3		            2048

#define GYRO_FS_SENSITIVITY_0					 131
#define GYRO_FS_SENSITIVITY_1					 65.5
#define GYRO_FS_SENSITIVITY_2					 32.8
#define GYRO_FS_SENSITIVITY_3				 	 16.4


/* I2C slave address of mpu6050  */
#define MPU6050_IMU_SLAVE_ADDR 0x68

#define MAX_VALUE 50

/* This is the linux OS device file for hte I2C3 controller of the SOC */
#define I2C_DEVICE_FILE   "/dev/i2c-2"

int fd;

/*  disable default sleep mode of MPU6050 IMU and configure the full scale ranges for gyro and accelerometer */
void mpu_imu_init()
{
    // disable sleep mode
    mpu_imu_write(MPU6050_IMU_REG_POWER, 0x00);
    usleep(500);

    // Adjust full scale values for accl and gyro 
    mpu_imu_write(MPU6050_IMU_REG_ACCEL_CONFIG, 0x00);
    usleep(500);
    mpu_imu_write(MPU6050_IMU_REG_GYRO_CONFIG, 0x00);
    usleep(500);
}

/*write an 8 bit data to the sensor at the address addr */
int mpu_imu_write(uint8_t addr, uint8_t data)
{
  int ret;
  char buf[2];
  buf[0]=addr;
  buf[1]=data;
  ret = write(fd,buf,2);
  if (ret <= 0)
  {
      perror("write failed\n");
      return -1;
  }
  return 0;
}


int mpu_imu_read(uint8_t base_addr, char *pBuffer,uint32_t len)
{
  int ret;
  char buf[2];
  buf[0]=base_addr;
  ret = write(fd,buf,1);
  if (ret <= 0)
  {
      perror("write address failed\n");
      return -1;
  }

  ret = read(fd,pBuffer,len);
  if(ret <= 0)
  {
      perror("read failed\n");
  }
  return 0;
}

/* read accelerometer values of x,y,z in to the buffer "pBuffer" */
void mpu_imu_read_acc(short int *pBuffer)
{
    //each axis value is of 2byte with two different register for each axis i.e High and low,  hence a buffer of 6 bytes has been assigned. 
    char acc_buffer[6];
    
    //start reading from the base address of accelerometer values i.e MPU6050_IMU_REG_ACC_X_HIGH
    mpu_imu_read(MPU6050_IMU_REG_ACC_X_HIGH,acc_buffer,6);
   
    pBuffer[0] = (int) ( (acc_buffer[0] << 8) |  acc_buffer[1] );
    pBuffer[1] = (int) ( (acc_buffer[2] << 8) |  acc_buffer[3] );
    pBuffer[2] = (int) ( (acc_buffer[4] << 8) |  acc_buffer[5] );

}

/* read gyro values of x,y,z in to the buffer "pBuffer" */
void mpu_imu_read_gyro(short *pBuffer)
{
    char gyro_buffer[6];
    
    //start reading from the base address of gyro values i.e MPU6050_IMU_REG_GYRO_X_HIGH
    mpu_imu_read(MPU6050_IMU_REG_GYRO_X_HIGH,gyro_buffer,6);

    pBuffer[0] =  ( (gyro_buffer[0] << 8) +  gyro_buffer[1] );
    pBuffer[1] =  ( (gyro_buffer[2] << 8) +  gyro_buffer[3] );
    pBuffer[2] =  ( (gyro_buffer[4] << 8) +  gyro_buffer[5] );

}

int main(void)
{

     short accl_value[3],gyro_value[3];
     double accx,accy,accz,gyrox,gyroy,gyroz;

     /*open the I2C device file */
    if ((fd = open(I2C_DEVICE_FILE,O_RDWR)) < 0) {
        perror("Failed to open I2C device file.\n");
        return -1;
    }

    /*set the I2C slave address using ioctl I2C_SLAVE command */
    if (ioctl(fd,I2C_SLAVE,MPU6050_IMU_SLAVE_ADDR) < 0) {
            perror("Failed to set I2C slave address.\n");
            close(fd);
            return -1;
    }

    mpu_imu_init();


    while(1)
    {
        mpu_imu_read_acc(accl_value);
        mpu_imu_read_gyro(gyro_value);

        /*Convert accelerometer raw values in to 'g' values*/
        accx = (double) accl_value[0]/ACCL_FS_SENSITIVITY_0;
        accy = (double) accl_value[1]/ACCL_FS_SENSITIVITY_0;
        accz = (double) accl_value[2]/ACCL_FS_SENSITIVITY_0;

        /* Convert gyroscope raw values in to  "°/s" (deg/seconds) */
        gyrox = (double) gyro_value[0]/GYRO_FS_SENSITIVITY_0;
        gyroy = (double) gyro_value[1]/GYRO_FS_SENSITIVITY_0;
        gyroz = (double) gyro_value[2]/GYRO_FS_SENSITIVITY_0;



#if 1
       printf("%0.2f	%0.2f	%0.2f\n",accx,accy,accz);
#endif
      
      /*delay*/
       usleep(50 * 1000);
    }

}


