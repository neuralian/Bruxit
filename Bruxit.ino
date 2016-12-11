/*
Wearable data logger 
For Mauro Farella's Bruxism project
	Teensy 3.2 MCU
	2x MPU9250 9dof sensors // in dev
	1x Audio          // TBD
	1x Pulse oximeter // TBD

MGP December 2016

MPU9250 code stolen from:
MPU9250_MS5637_t3 Basic Example Code
by: Kris Winer
date: April 1, 2014
license: Beerware - Use this code however you'd like. If you
find it useful you can buy me a beer some time.

Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor,
getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to
allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and
Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.

This sketch is intended specifically for the MPU9250+MS5637 Add-on shield for the Teensy 3.1.
It uses SDA/SCL on pins 17/16, respectively, and it uses the Teensy 3.1-specific Wire library i2c_t3.h.
The MS5637 is a simple but high resolution pressure sensor, which can be used in its high resolution
mode but with power consumption of 20 microAmp, or in a lower resolution mode with power consumption of
only 1 microAmp. The choice will depend on the application.

SDA and SCL should have external pull-up resistors (to 3.3V).
4K7 resistors are on the MPU9250+MS5637 breakout board.

Hardware setup:
MPU9250 Breakout --------- Arduino
VDD ---------------------- 3.3V
VDDI --------------------- 3.3V
SDA ----------------------- A4
SCL ----------------------- A5
GND ---------------------- GND

Note: The MPU9250 is an I2C sensor and uses the Arduino Wire library.
Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
*/
//#include "Wire.h"   

#include <i2c_t3.h>
#include <SPI.h>
//uint16_t MPU9250_ADDRESS;

// Using the MPU9250Teensy 3.1 Add-On shield, ADO is set to 0 
//// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
//#define ADO 1
//#if ADO
//MPU9250_ADDRESS = 0x69;  // Device address when ADO = 1
//
//#else
//MPU9250_ADDRESS = 0x68;  // Device address when ADO = 0
//#endif  

#include "MGP_Teensy32_MPU9250.h"

uint16_t all9df[9];

// IMU hardware addresses
// distal board has AD0 HI, proximal board has AD0 LO
uint8_t MPU9250_ADDRESS_DISTAL = MPU9250_ADDRESS_AD0_HI;
uint8_t MPU9250_ADDRESS_PROXIMAL = MPU9250_ADDRESS_AD0_LO;



void setup()
{
	//  Wire.begin();
	//  TWBR = 12;  // 400 kbit/sec I2C speed for Pro Mini
	// Setup for Master mode, pins 18/19, external pullups, 400kHz for Teensy 3.1
	Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
	delay(4000);
	Serial.begin(38400);

	// Set up the interrupt pin, its set as active high, push-pull
	pinMode(intPin, INPUT);
	pinMode(myLed, OUTPUT);
	digitalWrite(myLed, HIGH);

	I2Cscan();// look for I2C devices on the bus

    // Check distal MPU9250
	byte DISTAL_IMU_WHOAMI = readByte(MPU9250_ADDRESS_DISTAL, WHO_AM_I_MPU9250); 
	//Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); 
	//Serial.print(" I should be "); Serial.println(0x71, HEX);
	if (DISTAL_IMU_WHOAMI == 0x71) {
		Serial.println("Distal IMU responded OK.");
	}

	// Check distal magnetometer
	writeByte(MPU9250_ADDRESS_DISTAL, INT_PIN_CFG, 0x12);  // bypass enable 
	byte DISTAL_MAGNETOMETER_WHOAMI = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);   
	writeByte(MPU9250_ADDRESS_DISTAL, INT_PIN_CFG, 0x10);  // bypass disable
	//Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
	//Serial.print(" I should be "); Serial.println(0x48, HEX);
	if (DISTAL_MAGNETOMETER_WHOAMI == 0x48) {
		Serial.println("Distal magnetometer responded OK.");
	}

	// Check proximal MPU9250
	byte PROXIMAL_IMU_WHOAMI = readByte(MPU9250_ADDRESS_PROXIMAL, WHO_AM_I_MPU9250);
	//Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); 
	//Serial.print(" I should be "); Serial.println(0x71, HEX);
	if (PROXIMAL_IMU_WHOAMI == 0x71) {
		Serial.println("Proximal IMU responded OK.");
	}

	// Check proximal magnetometer
	writeByte(MPU9250_ADDRESS_PROXIMAL, INT_PIN_CFG, 0x12);  // bypass enable 
	byte PROXIMAL_MAGNETOMETER_WHOAMI = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);
	writeByte(MPU9250_ADDRESS_PROXIMAL, INT_PIN_CFG, 0x10);  // bypass disable
														   //Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
														   //Serial.print(" I should be "); Serial.println(0x48, HEX);
	if (PROXIMAL_MAGNETOMETER_WHOAMI == 0x48) {
		Serial.println("Proximal magnetometer responded OK.");
	}

	// proceed if IMU and magnetometer chips responded OK
	if ( (DISTAL_IMU_WHOAMI == 0x71) &&  (PROXIMAL_IMU_WHOAMI == 0x71) &&
		 (DISTAL_MAGNETOMETER_WHOAMI == 0x48) && (PROXIMAL_MAGNETOMETER_WHOAMI == 0x48))
		
	{
		
		MPU9250SelfTest(MPU9250_ADDRESS_DISTAL, SelfTest); // Start by performing self test and reporting values
		Serial.print("Accelerometer calibration errors (%):");
		Serial.print(SelfTest[0], 1); Serial.print(", ");
		Serial.print(SelfTest[1], 1); Serial.print(", ");
		Serial.println(SelfTest[2], 1); 
		Serial.print("Gyro calibration errors (%):");
		Serial.print(SelfTest[3], 1); Serial.print(", ");
		Serial.print(SelfTest[4], 1); Serial.print(", ");
		Serial.print(SelfTest[5], 1); Serial.println(", ");
		delay(1000);

		// get sensor resolutions, only need to do this once
		getAres();
		getGres();
		getMres();

		accelgyrocalMPU9250(MPU9250_ADDRESS_DISTAL, gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
		Serial.print("accelerometer biases (mg): "); 
		Serial.print(1000.*accelBias[0]); Serial.print(", ");
		Serial.print(1000.*accelBias[1]); Serial.print(", "); 
		Serial.println(1000.*accelBias[2]);
		Serial.print("gyro biases (dps): "); 
		Serial.print(gyroBias[0]); Serial.print(", "); 
		Serial.print(gyroBias[1]); Serial.print(", "); 
		Serial.println(gyroBias[2]);



		initMPU9250(MPU9250_ADDRESS_DISTAL);
		Serial.println("Distal IMU initialized OK."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature


		// Get magnetometer calibration from AK8963 ROM
		initAK8963(MPU9250_ADDRESS_DISTAL, magCalibration); 
		Serial.println("Distal magnetometer initialized."); // Initialize device for active mode read of magnetometer

		magcalMPU9250(MPU9250_ADDRESS_DISTAL, magBias, magScale);
		Serial.println("AK8963 mag biases (mG)"); Serial.println(magBias[0]); Serial.println(magBias[1]); Serial.println(magBias[2]);
		Serial.println("AK8963 mag scale (mG)"); Serial.println(magScale[0]); Serial.println(magScale[1]); Serial.println(magScale[2]);
		delay(2000); // add delay to see results before serial spew of data

		if (SerialDebug) {
			//  Serial.println("Calibration values: ");
			Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
			Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
			Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);
		}



		attachInterrupt(intPin, myinthandler, RISING);  // define interrupt for INT pin output of MPU9250

	}
	else
	{
		Serial.print("Could not connect to IMU sensors");
//		Serial.println(c, HEX);
		while (1); // Loop forever if communication doesn't happen
	}


}

void loop()
{

	//	Serial.println("hello!");
	// If intPin goes high, all data registers have new data
	//if(newData == true) {  // On interrupt, read data
	newData = false;  // reset newData flag
	readAccelGyro(MPU9250_ADDRESS_DISTAL, MPU9250Data); // INT cleared on any read
															//   readAccelData(accelCount);  // Read the x/y/z adc values

															// Now we'll calculate the accleration value into actual g's
	ax = (float)MPU9250Data[0] * aRes - accelBias[0];  // get actual g value, this depends on scale being set
	ay = (float)MPU9250Data[1] * aRes - accelBias[1];
	az = (float)MPU9250Data[2] * aRes - accelBias[2];

	//   readGyroData(gyroCount);  // Read the x/y/z adc values

	// Calculate the gyro value into actual degrees per second
	gx = (float)MPU9250Data[4] * gRes;  // get actual gyro value, this depends on scale being set
	gy = (float)MPU9250Data[5] * gRes;
	gz = (float)MPU9250Data[6] * gRes;

	readMag(MPU9250_ADDRESS_DISTAL, magCount);  // Read the x/y/z adc values

							// Calculate the magnetometer values in milliGauss
							// Include factory calibration per data sheet and user environmental corrections
	if (newMagData == true) {
		newMagData = false; // reset newMagData flag
		mx = (float)magCount[0] * mRes*magCalibration[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
		my = (float)magCount[1] * mRes*magCalibration[1] - magBias[1];
		mz = (float)magCount[2] * mRes*magCalibration[2] - magBias[2];
		mx *= magScale[0];
		my *= magScale[1];
		mz *= magScale[2];
	}
	//}

	Now = micros();
	deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
	lastUpdate = Now;

	sum += deltat; // sum for averaging filter update rate
	sumCount++;

	// Sensors x (y)-axis of the accelerometer/gyro is aligned with the y (x)-axis of the magnetometer;
	// the magnetometer z-axis (+ down) is misaligned with z-axis (+ up) of accelerometer and gyro!
	// We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
	// For the MPU9250+MS5637 Mini breakout the +x accel/gyro is North, then -y accel/gyro is East. So if we want te quaternions properly aligned
	// we need to feed into the Madgwick function Ax, -Ay, -Az, Gx, -Gy, -Gz, My, -Mx, and Mz. But because gravity is by convention
	// positive down, we need to invert the accel data, so we pass -Ax, Ay, Az, Gx, -Gy, -Gz, My, -Mx, and Mz into the Madgwick
	// function to get North along the accel +x-axis, East along the accel -y-axis, and Down along the accel -z-axis.
	// This orientation choice can be modified to allow any convenient (non-NED) orientation convention.
	// Pass gyro rate as rad/s
	///MadgwickQuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f,  my,  -mx, mz);
	//  if(passThru)MahonyQuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f,  my,  -mx, mz);

	// Serial print and/or display at 0.5 s rate independent of data rates
	delt_t = millis() - count;
	if (delt_t > 200) { // update 100Hz

		if (SerialDebug) {
			// print xyz acceleration
			Serial.print((int)1000 * ax); Serial.print(", ");
			Serial.print((int)1000 * ay); Serial.print(", ");
			Serial.print((int)1000 * az); Serial.print(", ");

			// print xyz gyro
			Serial.print(gx, 2); Serial.print(", ");
			Serial.print(gy, 2); Serial.print(", ");
			Serial.print(gz, 2); Serial.print(", ");

			// print xyz magnetic
			Serial.print((int)mx); Serial.print(", ");
			Serial.print((int)my); Serial.print(", ");
			Serial.println((int)mz);
			//
			//Serial.print("q0 = "); Serial.print(q[0]);
			//Serial.print(" qx = "); Serial.print(q[1]);
			//Serial.print(" qy = "); Serial.print(q[2]);
			//Serial.print(" qz = "); Serial.println(q[3]);
		}
		//tempCount = readTempData();  // Read the gyro adc values
		//temperature = ((float) tempCount) / 333.87 + 21.0; // Gyro chip temperature in degrees Centigrade
		//// Print temperature in degrees Centigrade
		//Serial.print("Gyro temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
		//

		// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
		// In this coordinate system, the positive z-axis is down toward Earth.
		// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
		// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
		// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
		// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
		// Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
		// applied in the correct order which for this configuration is yaw, pitch, and then roll.
		// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
		//Software AHRS:
		//   yaw   = atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
		//   pitch = -asinf(2.0f * (q[1] * q[3] - q[0] * q[2]));
		//   roll  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
		//   pitch *= 180.0f / PI;
		//   yaw   *= 180.0f / PI;
		//   yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
		//   if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
		//   roll  *= 180.0f / PI;
		//a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
		//a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
		//a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
		//a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
		//a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
		//pitch = -asinf(a32);
		//roll  = atan2f(a31, a33);
		//yaw   = atan2f(a12, a22);
		//pitch *= 180.0f / PI;
		//yaw   *= 180.0f / PI;
		//yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
		//if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
		//roll  *= 180.0f / PI;
		//lin_ax = ax + a31;
		//lin_ay = ay + a32;
		//lin_az = az - a33;
		//if(SerialDebug) {
		//Serial.print("Yaw, Pitch, Roll: ");
		//Serial.print(yaw, 2);
		//Serial.print(", ");
		//Serial.print(pitch, 2);
		//Serial.print(", ");
		//Serial.println(roll, 2);
		//
		//Serial.print("Grav_x, Grav_y, Grav_z: ");
		//Serial.print(-a31*1000, 2);
		//Serial.print(", ");
		//Serial.print(-a32*1000, 2);
		//Serial.print(", ");
		//Serial.print(a33*1000, 2);  Serial.println(" mg");
		//Serial.print("Lin_ax, Lin_ay, Lin_az: ");
		//Serial.print(lin_ax*1000, 2);
		//Serial.print(", ");
		//Serial.print(lin_ay*1000, 2);
		//Serial.print(", ");
		//Serial.print(lin_az*1000, 2);  Serial.println(" mg");
		//
		//Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");
		//}


		// With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and
		// >200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
		// The filter update rate is determined mostly by the mathematical steps in the respective algorithms,
		// the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
		// an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
		// filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively.
		// This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
		// This filter update rate should be fast enough to maintain accurate platform orientation for
		// stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
		// produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
		// The 3.3 V 8 MHz Pro Mini is doing pretty well!


		digitalWrite(myLed, !digitalRead(myLed));
		count = millis();
		sumCount = 0;
		sum = 0;
	}

}
