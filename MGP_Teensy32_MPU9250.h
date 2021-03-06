// MGP_mpu9250.h

#ifndef _MGP_MPU9250_h
#define _MGP_MPU9250_h

#include "arduino.h"


#include <i2c_t3.h>
#include <SPI.h>

// See also MPU-9250 Register Map and Descriptions, Revision 4.0, RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in 
// above document; the MPU9250 and MPU9150 are virtually identical but the latter has a different register map
//
//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L	 0x03  // data
#define AK8963_XOUT_H	 0x04
#define AK8963_YOUT_L	 0x05
#define AK8963_YOUT_H	 0x06
#define AK8963_ZOUT_L	 0x07
#define AK8963_ZOUT_H	 0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00                  
#define SELF_TEST_Y_GYRO 0x01                                                                          
#define SELF_TEST_Z_GYRO 0x02

/*#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B */

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E    
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E   
#define WOM_THR          0x1F   

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24   
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71 
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

// MPU9250 I2C address when AD0 pin is HI
#define MPU9250_ADDRESS_AD0_HI  0x69
// MPU9250 I2C address when AD0 pin is LO
#define MPU9250_ADDRESS_AD0_LO  0x68


#define SerialDebug true  // set to true to get Serial output for debugging

// Set initial input parameters
enum Ascale {
	AFS_2G = 0,
	AFS_4G,
	AFS_8G,
	AFS_16G
};

enum Gscale {
	GFS_250DPS = 0,
	GFS_500DPS,
	GFS_1000DPS,
	GFS_2000DPS
};

enum Mscale {
	MFS_14BITS = 0, // 0.6 mG per LSB
	MFS_16BITS      // 0.15 mG per LSB
};

#define ADC_256  0x00 // define pressure and temperature conversion rates
#define ADC_512  0x02
#define ADC_1024 0x04
#define ADC_2048 0x06
#define ADC_4096 0x08
#define ADC_8192 0x0A
#define ADC_D1   0x40
#define ADC_D2   0x50



// Specify sensor full scale
uint8_t OSR = ADC_8192;     // set pressure amd temperature oversample rate
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

							 // Pin definitions
int intPin = 8;
bool newData = false;
bool newMagData = false;

int myLed = 13;

uint16_t Pcal[8];         // calibration constants from MS5637 PROM registers
unsigned char nCRC;       // calculated check sum to ensure PROM integrity
uint32_t D1 = 0, D2 = 0;  // raw MS5637 pressure and temperature data
double dT, OFFSET, SENS, T2, OFFSET2, SENS2;  // First order and second order corrections for raw S5637 temperature and pressure data

int16_t MPU9250Data[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output


int16_t tempCount;            // temperature raw count output
float   temperature;          // Stores the MPU9250 gyro internal chip temperature in degrees Celsius
double Temperature, Pressure; // stores MS5637 pressures sensor pressure and temperature

							  // global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (4.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
											  // There is a tradeoff in the beta parameter between accuracy and response speed.
											  // In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
											  // However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
											  // Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
											  // By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
											  // I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
											  // the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
											  // In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
float pitch, yaw, roll;
float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)
float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };    // vector to hold quaternion
float eInt[3] = { 0.0f, 0.0f, 0.0f };       // vector to hold integral error for Mahony method



class MPU9250 {
	// Driver for Invensense 9dof inertial sensor, I2C

private:
	uint8_t address;

public:
	int16_t data_buffer[10];  // buffer for 10df data (9+1 to allow for barometer)
	int16_t * a_buf;    // pointer to raw accelerometer data in buffer
	int16_t * g_buf;    // pointer to raw gyro data
	int16_t * m_buf;    // pointer to raw magnetometer data
	int16_t * b_buf;    // pointer to raw barometer data
	float x[10];       // calibrated measurements
	float * a;        // pointer to calibrated accelerometer data
	float * g;
	float *m; 
	float *b;

	float gyroBias[3] = { 0, 0, 0 };
	float accelBias[3] = { 0, 0, 0 };
	float magBias[3] = { 0, 0, 0 };
	float magScale[3] = { 0, 0, 0 };      // Bias corrections for gyro and accelerometer
	float magCalibration[3] = { 0, 0, 0 };  // Factory mag calibration and mag bias

	// constructor
	// chip has one of two addresses depending on whether AD0 pin is HI or LO
	MPU9250(uint8_t AD0);

	// methods
	void init(void);			// initialize accelerometer, gyro and magnetometer
	void initMPU9250(void);    // initialize accelerometer & gyro
	void initAK8963(void);      // initialize magnetometer
	bool online(void);       // verify MPU9250 online
	bool magnetometer_online(void); // verify magnetometer online
	void getMres(void);			//  magnetometer calibration
	void getAres(void);			//  accelerometer calibration
	void getGres(void);			// gyro calibration
	void readAccelGyro(void);	// read accelerometer and gyro  
	void readCalibratedAccelGyro(void);	// read calibrated accelerometer and gyro data
	void readCalibratedAccel(void);
	void readCalibratedGyro(void);
	void readCalibratedMag(void);
	void readAccel(void);		// read accelerometer 
	void readGyro(void);        // read gyro 
	void readMag(void);         // read magnetometer
	void readRaw(void);            // read all 9df
	void readCalibrated(void);
	int16_t readTemperature(void); // chip temperature
	void calibrateAccelerometerAndGyro(void);  // calibrate accelerometer & gyro
	void calibrateMagnetometer(void);  // calibrate magnetometer
	void selfTest(void);  
};


MPU9250::MPU9250(uint8_t AD0) {
	//// construct driver with I2C address depending on AD0 pin state
	if (AD0 == 1) address = MPU9250_ADDRESS_AD0_HI;
	         else address = MPU9250_ADDRESS_AD0_LO;

	//// point to accelerometer, gyro, magnetometer and barometer data in buffer
	 a_buf = &(data_buffer[0]);
	 g_buf = &(data_buffer[3]);
	 m_buf = &(data_buffer[6]);
	 b_buf = &(data_buffer[9]);  // pointers may be wrong ... b may be in x[0] not x[9]

	 // point to calibrated measurements
	 a = &(x[0]);
	 g = &(x[3]);
	 m = &(x[5]);
	 b = &(x[9]);
}


//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================
// I2C read/write functions for the MPU9250 and AK8963 sensors

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(subAddress);            // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
									   //	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
	//        Wire.requestFrom(address, count);  // Read bytes from slave register address 
	Wire.requestFrom(address, (size_t)count);  // Read bytes from slave register address 
	while (Wire.available()) {
		dest[i++] = Wire.read();
	}         // Put read results in the Rx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data	 
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
											 //	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
											 //	Wire.requestFrom(address, 1);  // Read one byte from slave register address 
	Wire.requestFrom(address, (size_t)1);   // Read one byte from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

void myinthandler()
{
	newData = true;
}

bool MPU9250::online(void) {
	// returns true if MPU9250 responds with correct whoAmI
	return(readByte(address, WHO_AM_I_MPU9250) == 0x71 ? true : false);
}

bool MPU9250::magnetometer_online(void) {
	// returns true if the magnetometer responds with correct whoAmI
	writeByte(address, INT_PIN_CFG, 0x12);  // bypass enable 
	uint8_t who = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);
	writeByte(address, INT_PIN_CFG, 0x10);  // bypass disable
	return(who == 0x48 ? true : false);
}

void MPU9250::getMres() {
	switch (Mscale)
	{
		// Possible magnetometer scales (and their register bit settings) are:
		// 14 bit resolution (0) and 16 bit resolution (1)
	case MFS_14BITS:
		mRes = 10.*4912. / 8190.; // Proper scale to return milliGauss
		break;
	case MFS_16BITS:
		mRes = 10.*4912. / 32760.0; // Proper scale to return milliGauss
		break;
	}
}

void MPU9250::getGres(void) {
	switch (Gscale)
	{
		// Possible gyro scales (and their register bit settings) are:
		// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
	case GFS_250DPS:
		gRes = 250.0 / 32768.0;
		break;
	case GFS_500DPS:
		gRes = 500.0 / 32768.0;
		break;
	case GFS_1000DPS:
		gRes = 1000.0 / 32768.0;
		break;
	case GFS_2000DPS:
		gRes = 2000.0 / 32768.0;
		break;
	}
}

void MPU9250::getAres(void) {
	switch (Ascale)
	{
		// Possible accelerometer scales (and their register bit settings) are:
		// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
	case AFS_2G:
		aRes = 2.0 / 32768.0;
		break;
	case AFS_4G:
		aRes = 4.0 / 32768.0;
		break;
	case AFS_8G:
		aRes = 8.0 / 32768.0;
		break;
	case AFS_16G:
		aRes = 16.0 / 32768.0;
		break;
	}
}



void MPU9250::readAccelGyro(void)
// read 3 accelerations & 3 gyro into 6x16bit array
{
	uint8_t rawData[14];  // x/y/z accel register data stored here
	readBytes(address, ACCEL_XOUT_H, 12, &rawData[0]);  // Read  12 raw data registers into data array
	data_buffer[0] = ((int16_t)rawData[0] << 8) | rawData[1];  // Turn the MSB and LSB into a signed 16-bit value
	data_buffer[1] = ((int16_t)rawData[2] << 8) | rawData[3];
	data_buffer[2] = ((int16_t)rawData[4] << 8) | rawData[5];
	data_buffer[3] = ((int16_t)rawData[6] << 8) | rawData[7];
	data_buffer[4] = ((int16_t)rawData[8] << 8) | rawData[9];
	data_buffer[5] = ((int16_t)rawData[10] << 8) | rawData[11];
	data_buffer[6] = ((int16_t)rawData[12] << 8) | rawData[13];
}

void MPU9250::readCalibratedAccelGyro(void) {

	readAccelGyro();

	// calibrated acceleration measurements
	a[0] = (float)data_buffer[0] * aRes - accelBias[0];  // get actual g value, this depends on scale being set
	a[1] = (float)data_buffer[1] * aRes - accelBias[1];
	a[2] = (float)data_buffer[2] * aRes - accelBias[2];

	// calibrated gyro measurements
	g[0] = (float)data_buffer[4] * gRes;  // get actual gyro value, this depends on scale being set
	g[1] = (float)data_buffer[5] * gRes;
	g[2] = (float)data_buffer[6] * gRes;

}

void MPU9250::readAccel(void)
{
	uint8_t rawData[6];  // x/y/z accel register data stored here
	readBytes(address, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
	*a_buf     = ((int16_t)rawData[0] << 8) | rawData[1];  // Turn the MSB and LSB into a signed 16-bit value
	*(a_buf+1) = ((int16_t)rawData[2] << 8) | rawData[3];
	*(a_buf+2) = ((int16_t)rawData[4] << 8) | rawData[5];
}

void MPU9250::readCalibratedAccel(void) {

	readAccel();

	// calibrated acceleration measurements
	a[0] = (float)data_buffer[0] * aRes - accelBias[0];  
	a[1] = (float)data_buffer[1] * aRes - accelBias[1];
	a[2] = (float)data_buffer[2] * aRes - accelBias[2];
}


void MPU9250::readGyro(void)
{
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	readBytes(address, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	*g_buf     = ((int16_t)rawData[0] << 8) | rawData[1];  // Turn the MSB and LSB into a signed 16-bit value
	*(g_buf+1) = ((int16_t)rawData[2] << 8) | rawData[3];
	*(g_buf+2) = ((int16_t)rawData[4] << 8) | rawData[5];
}

void MPU9250::readCalibratedGyro(void) {
	readGyro();

	// calibrated gyro measurements
	g[0] = (float)data_buffer[4] * gRes;  // get actual gyro value, this depends on scale being set
	g[1] = (float)data_buffer[5] * gRes;
	g[2] = (float)data_buffer[6] * gRes;

}

void MPU9250::readMag(void)
{
	// enable magnetometer
	writeByte(address, INT_PIN_CFG, 0x12);  // bypass enable

	uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
	newMagData = (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01);
	if (newMagData == true) { // wait for magnetometer data ready bit to be set
		readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
		uint8_t c = rawData[6]; // End data read by reading ST2 register
		if (!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
			*m_buf     = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
			*(m_buf+1) = ((int16_t)rawData[3] << 8) | rawData[2];  // Data stored as little Endian
			*(m_buf+2) = ((int16_t)rawData[5] << 8) | rawData[4];
		}
	}

	// disable magnetometer
	writeByte(address, INT_PIN_CFG, 0x10);  // bypass disable

}

void MPU9250::readCalibratedMag(void) {

	readMag();
	// Calculate the magnetometer values in milliGauss
	// Include factory calibration per data sheet and user environmental corrections
	if (newMagData == true) {
		newMagData = false; // reset newMagData flag
		m[0] = ((float)m_buf[0] * mRes*magCalibration[0] - magBias[0])*magScale[0];  // get actual magnetometer value, this depends on scale being set
		m[1] = ((float)m_buf[1] * mRes*magCalibration[1] - magBias[1])*magScale[1];
		m[2] = ((float)m_buf[2] * mRes*magCalibration[2] - magBias[2])*magScale[2];
	}
}



void MPU9250::readRaw(void)
// read all 9 df
{
	readAccelGyro();
	readMag();
}

void MPU9250::readCalibrated(void) {

	readRaw();

	// calibrated acceleration measurements
	a[0] = (float)data_buffer[0] * aRes - accelBias[0];
	a[1] = (float)data_buffer[1] * aRes - accelBias[1];
	a[2] = (float)data_buffer[2] * aRes - accelBias[2];

	// calibrated gyro measurements
	g[0] = (float)data_buffer[4] * gRes;  // get actual gyro value, this depends on scale being set
	g[1] = (float)data_buffer[5] * gRes;
	g[2] = (float)data_buffer[6] * gRes;

	if (newMagData == true) {
		newMagData = false; // reset newMagData flag
		m[0] = ((float)m_buf[0] * mRes*magCalibration[0] - magBias[0])*magScale[0];  // get actual magnetometer value, this depends on scale being set
		m[1] = ((float)m_buf[1] * mRes*magCalibration[1] - magBias[1])*magScale[1];
		m[2] = ((float)m_buf[2] * mRes*magCalibration[2] - magBias[2])*magScale[2];
	}

}

int16_t MPU9250::readTemperature(void)
{
	uint8_t rawData[2];  // x/y/z gyro register data stored here
	readBytes(address, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
	return ((int16_t)rawData[0] << 8) | rawData[1];  // Turn the MSB and LSB into a 16-bit value
}




void MPU9250::initAK8963(void)
{
	// enable magnetometer
	writeByte(address, INT_PIN_CFG, 0x12);  // bypass enable
	// First extract the factory calibration for each magnetometer axis
	uint8_t rawData[3];  // x/y/z gyro calibration data stored here
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
	delay(10);
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
	delay(10);
	readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
	magCalibration[0] = (float)(rawData[0] - 128) / 256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
	magCalibration[1] = (float)(rawData[1] - 128) / 256. + 1.;
	magCalibration[2] = (float)(rawData[2] - 128) / 256. + 1.;
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
	delay(10);
	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
	delay(10);

	// disable magnetometer bypass
	writeByte(address, INT_PIN_CFG, 0x10);  // bypass disable

}


void  MPU9250::initMPU9250(void)
{
	// wake up device
	writeByte(address, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
	delay(100); // Wait for all registers to reset 

				// get stable time source
	writeByte(address, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
	delay(200);

	// Configure Gyro and Thermometer
	// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
	// minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
	// be higher than 1 / 0.0059 = 170 Hz
	// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
	// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
	writeByte(address, CONFIG, 0x03);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	writeByte(address, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
												   // determined inset in CONFIG above

												   // Set gyroscope full scale range
												   // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	uint8_t c = readByte(address, GYRO_CONFIG); // get current GYRO_CONFIG register value
														// c = c & ~0xE0; // Clear self-test bits [7:5] 
	c = c & ~0x02; // Clear Fchoice bits [1:0] 
	c = c & ~0x18; // Clear AFS bits [4:3]
	c = c | Gscale << 3; // Set full scale range for the gyro
						 // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
	writeByte(address, GYRO_CONFIG, c); // Write new GYRO_CONFIG value to register

												// Set accelerometer full-scale range configuration
	c = readByte(address, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
												 // c = c & ~0xE0; // Clear self-test bits [7:5] 
	c = c & ~0x18;  // Clear AFS bits [4:3]
	c = c | Ascale << 3; // Set full scale range for the accelerometer 
	writeByte(address, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

												 // Set accelerometer sample rate configuration
												 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
												 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	c = readByte(address, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
	c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
	c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	writeByte(address, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

												  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
												  // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

												  // Configure Interrupts and magnetometer bypass 
												  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
												  // clear on read of INT_STATUS.
												  // Disable I2C_BYPASS_EN (0x2 = bit 1), will be enabled within magnetometer read function.
												  //   writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);    
	writeByte(address, INT_PIN_CFG, 0x10);  // INT is 50 microsecond pulse and any read to clear  
	writeByte(address, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
	delay(100);
}

void  MPU9250::init(void) {
	initMPU9250();
	initAK8963();

}


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void MPU9250::calibrateAccelerometerAndGyro(void)
{
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

	// reset device
	writeByte(address, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	delay(100);

	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
	// else use the internal oscillator, bits 2:0 = 001
	writeByte(address, PWR_MGMT_1, 0x01);
	writeByte(address, PWR_MGMT_2, 0x00);
	delay(200);

	// Configure device for bias calculation
	writeByte(address, INT_ENABLE, 0x00);   // Disable all interrupts
	writeByte(address, FIFO_EN, 0x00);      // Disable FIFO
	writeByte(address, PWR_MGMT_1, 0x00);   // Turn on internal clock source
	writeByte(address, I2C_MST_CTRL, 0x00); // Disable I2C master
	writeByte(address, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	writeByte(address, USER_CTRL, 0x0C);    // Reset FIFO and DMP
	delay(15);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	writeByte(address, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	writeByte(address, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	writeByte(address, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	writeByte(address, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t  gyrosensitivity = 131;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

										 // Configure FIFO to capture accelerometer and gyro data for bias calculation
	writeByte(address, USER_CTRL, 0x40);   // Enable FIFO  
	writeByte(address, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

			   // At end of sample accumulation, turn off FIFO sensor read
	writeByte(address, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	readBytes(address, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count / 12;// How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };
		readBytes(address, FIFO_R_W, 12, &data[0]); // read data for averaging
		accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
		accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
		gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
		gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
		gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

		accel_bias[0] += (int32_t)accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t)accel_temp[1];
		accel_bias[2] += (int32_t)accel_temp[2];
		gyro_bias[0] += (int32_t)gyro_temp[0];
		gyro_bias[1] += (int32_t)gyro_temp[1];
		gyro_bias[2] += (int32_t)gyro_temp[2];

	}
	accel_bias[0] /= (int32_t)packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t)packet_count;
	accel_bias[2] /= (int32_t)packet_count;
	gyro_bias[0] /= (int32_t)packet_count;
	gyro_bias[1] /= (int32_t)packet_count;
	gyro_bias[2] /= (int32_t)packet_count;

	if (accel_bias[2] > 0L) { accel_bias[2] -= (int32_t)accelsensitivity; }  // Remove gravity from the z-axis accelerometer bias calculation
	else { accel_bias[2] += (int32_t)accelsensitivity; }

	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0] / 4) & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
	data[3] = (-gyro_bias[1] / 4) & 0xFF;
	data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
	data[5] = (-gyro_bias[2] / 4) & 0xFF;

	// Push gyro biases to hardware registers
	writeByte(address, XG_OFFSET_H, data[0]);
	writeByte(address, XG_OFFSET_L, data[1]);
	writeByte(address, YG_OFFSET_H, data[2]);
	writeByte(address, YG_OFFSET_L, data[3]);
	writeByte(address, ZG_OFFSET_H, data[4]);
	writeByte(address, ZG_OFFSET_L, data[5]);

	// Print scaled gyro biases 
	Serial.print("gyro biases (dps): ");
	Serial.print(1000.*(float)gyro_bias[0] / (float)gyrosensitivity); Serial.print(", ");
	Serial.print(1000.*(float)gyro_bias[1] / (float)gyrosensitivity); Serial.print(", ");
	Serial.println(1000.*(float)gyro_bias[2] / (float)gyrosensitivity);



	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.
	// XA_OFFSET is a 15 bit quantity with bits 14:7 in the high byte and 6:0 in the low byte with temperature compensation in bit0
	// so having got it in a 16 bit short, and having preserved the bottom bit, the number must be shifted right by 1 or divide by 2
	// to give the correct value for calculations. After calculations it must be shifted left by 1 or multiplied by 2 to get
	// the bytes correct, then the preserved bit0 can be put back before the bytes are written to registers


	int32_t accel_bias_reg[3] = { 0, 0, 0 }; // A place to hold the factory accelerometer trim biases
	readBytes(address, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
	readBytes(address, YA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[1] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
	readBytes(address, ZA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[2] = (int32_t)(((int16_t)data[0] << 8) | data[1]);

	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = { 0, 0, 0 }; // Define array to hold mask bit for each accelerometer bias axis

	for (ii = 0; ii < 3; ii++) {
		if ((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
		accel_bias_reg[ii] /= 2; //divide accel_bias_reg by 2 to remove the bottom bit and preserve sign
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	for (ii = 0; ii < 3; ii++) {
		accel_bias_reg[ii] -= (accel_bias[ii] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
		accel_bias_reg[ii] *= 2;                     //multiply by two to leave the bottom bit clear and but all the bits in the correct bytes
	}


	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0]) & 0xFE;   // copy bits 1-7, clear bit 0
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1]) & 0xFE;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2]) & 0xFE;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	// KrisWiner says: 
		// Apparently this is not working for the acceleration biases in the MPU-9250
		// Are we handling the temperature correction bit properly?
	// ... apparently not, see the fix at https://github.com/kriswiner/MPU-9250/issues/49 
	// This fix has been implemented here
	// Push accelerometer biases to hardware registers	
	writeByte(address, XA_OFFSET_H, data[0]);
	writeByte(address, XA_OFFSET_L, data[1]);
	writeByte(address, YA_OFFSET_H, data[2]);
	writeByte(address, YA_OFFSET_L, data[3]);
	writeByte(address, ZA_OFFSET_H, data[4]);
	writeByte(address, ZA_OFFSET_L, data[5]);
									 
	// Print scaled accelerometer biases for display in the main program
	Serial.print("accelerometer biases (mg): "); 
	Serial.print(1000.*(float)accel_bias[0] / (float)accelsensitivity); Serial.print(", ");
	Serial.print(1000.*(float)accel_bias[1] / (float)accelsensitivity); Serial.print(", ");
	Serial.println(1000.*(float)accel_bias[2] / (float)accelsensitivity);

}


void MPU9250::calibrateMagnetometer(void)
{
	uint16_t ii = 0, sample_count = 0;
	int32_t mag_bias[3] = { 0, 0, 0 }, mag_scale[3] = { 0, 0, 0 };
	int16_t mag_max[3] = { -32767, -32767, -32767 }, mag_min[3] = { 32767, 32767, 32767 }, mag_temp[3] = { 0, 0, 0 };

	Serial.println("Mag Calibration: Wave device in a figure eight until done!");
	delay(1000);

	// shoot for ~fifteen seconds of mag data
	if (Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
	if (Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
	for (ii = 0; ii < sample_count; ii++) {
		readMag();  // Read the mag data   
		for (int jj = 0; jj < 3; jj++) {
//			Serial.print(*(m + jj)); Serial.print(", ");
			if (*(m_buf+jj) > mag_max[jj]) mag_max[jj] = *(m_buf +jj);
			if (*(m_buf + jj) < mag_min[jj]) mag_min[jj] = *(m_buf + jj);
			//Serial.print(mag_min[jj]); Serial.print(", ");
			//Serial.println(mag_max[jj]);
		}
		if (Mmode == 0x02) delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
		if (Mmode == 0x06) delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
	}

	//    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
	//    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
	//    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

	// Get hard iron correction
	mag_bias[0] = (mag_max[0] + mag_min[0]) / 2;  // get average x mag bias in counts
	mag_bias[1] = (mag_max[1] + mag_min[1]) / 2;  // get average y mag bias in counts
	mag_bias[2] = (mag_max[2] + mag_min[2]) / 2;  // get average z mag bias in counts

	// display
	Serial.print("Magnetometer bias (mG): "); Serial.print((float)mag_bias[0] * mRes*magCalibration[0]); Serial.print(", ");
	Serial.print((float)mag_bias[1] * mRes*magCalibration[1]); Serial.print(", ");
	Serial.println((float)mag_bias[2] * mRes*magCalibration[2]); 
	Serial.println(mRes); Serial.println(magCalibration[0]);

	// Get soft iron correction estimate
	mag_scale[0] = (mag_max[0] - mag_min[0]) / 2;  // get average x axis max chord length in counts
	mag_scale[1] = (mag_max[1] - mag_min[1]) / 2;  // get average y axis max chord length in counts
	mag_scale[2] = (mag_max[2] - mag_min[2]) / 2;  // get average z axis max chord length in counts

	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	avg_rad /= 3.0;

	Serial.print("Magnetometer scale (mG): "); Serial.print(avg_rad / ((float)mag_scale[0])); Serial.print(", ");
	Serial.print(avg_rad / ((float)mag_scale[1])); Serial.print(", ");
	Serial.println(avg_rad / ((float)mag_scale[2])); 


	Serial.println("Mag Calibration done!");
}



// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250::selfTest(void) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
	uint8_t rawData[6] = { 0, 0, 0, 0, 0, 0 };
	uint8_t selfTest[6];
	int32_t gAvg[3] = { 0 }, aAvg[3] = { 0 }, aSTAvg[3] = { 0 }, gSTAvg[3] = { 0 };
	float factoryTrim[6];
	float report[6];
	uint8_t FS = 0;

	writeByte(address, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
	writeByte(address, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	writeByte(address, GYRO_CONFIG, FS << 3);  // Set full scale range for the gyro to 250 dps
	writeByte(address, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	writeByte(address, ACCEL_CONFIG, FS << 3); // Set full scale range for the accelerometer to 2 g

	for (int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer

		readBytes(address, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
		aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
		aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
		aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

		readBytes(address, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
		gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
		gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
		gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
	}

	for (int ii = 0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
		aAvg[ii] /= 200;
		gAvg[ii] /= 200;
	}

	// Configure the accelerometer for self-test
	writeByte(address, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
	writeByte(address, GYRO_CONFIG, 0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	delay(25);  // Delay a while to let the device stabilize

	for (int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer

		readBytes(address, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
		aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
		aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

		readBytes(address, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
		gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
		gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
	}

	for (int ii = 0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
	}

	// Configure the gyro and accelerometer for normal operation
	writeByte(address, ACCEL_CONFIG, 0x00);
	writeByte(address, GYRO_CONFIG, 0x00);
	delay(25);  // Delay a while to let the device stabilize

				// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	selfTest[0] = readByte(address, SELF_TEST_X_ACCEL); // X-axis accel self-test results
	selfTest[1] = readByte(address, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
	selfTest[2] = readByte(address, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
	selfTest[3] = readByte(address, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
	selfTest[4] = readByte(address, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
	selfTest[5] = readByte(address, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

																// Retrieve factory self-test value from self-test code reads
	factoryTrim[0] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[0] - 1.0))); // FT[Xa] factory trim calculation
	factoryTrim[1] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[1] - 1.0))); // FT[Ya] factory trim calculation
	factoryTrim[2] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[2] - 1.0))); // FT[Za] factory trim calculation
	factoryTrim[3] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[3] - 1.0))); // FT[Xg] factory trim calculation
	factoryTrim[4] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[4] - 1.0))); // FT[Yg] factory trim calculation
	factoryTrim[5] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[5] - 1.0))); // FT[Zg] factory trim calculation

																					  
	Serial.print("Accelerometer calibration errors (%):");
	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	// To get percent, must multiply by 100
	for (int i = 0; i < 3; i++) {
		Serial.print(100.0*((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.);
		if (i<2) Serial.print(", ");
	}
	Serial.println("");
	Serial.print("Gyro calibration errors (%):");
	for (int i = 0; i < 3; i++) {
		Serial.print(report[i + 3] = 100.0*((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3] - 100.); 
		if (i<2) Serial.print(", ");
	}
	Serial.println("");

}

// I2C scan function

void I2Cscan()
{
	// scan for i2c devices
	byte error, address;
	int nDevices;

	Serial.println("Scanning...");

	nDevices = 0;
	for (address = 1; address < 127; address++)
	{
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		Wire.beginTransmission(address);
		error = Wire.endTransmission();

		if (error == 0)
		{
			Serial.print("I2C device found at address 0x");
			if (address<16)
				Serial.print("0");
			Serial.print(address, HEX);
			Serial.println("  !");

			nDevices++;
		}
		else if (error == 4)
		{
			Serial.print("Unknow error at address 0x");
			if (address<16)
				Serial.print("0");
			Serial.println(address, HEX);
		}
	}
	if (nDevices == 0)
		Serial.println("No I2C devices found\n");
	else
		Serial.println("done\n");

}






#endif

