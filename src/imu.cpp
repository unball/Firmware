#include "imu.hpp"

namespace Imu{

    axis accel, gyro, accelOffset, gyroOffset = {0};
    imuAll imuData;
    double accelScale[4] = {16384.0, 8192.0, 4096.0, 2048.0};
    double gyroScale[4] = {131.0, 65.5, 32.8, 16.4};

	double deltaT(){
		static double timeAnt = 0;
        double delta;
		delta = (micros() - timeAnt)/1000000.0;
        timeAnt = micros();
		return delta;
	}

	/*retorna velocidade linear
	  velocidade linear calculada através de integração trapezoidal*/
	double linearVel(double bias){
		static double vel = 0, Accel[3] = {0.0, 0.0, 0.0}, DeltaT[3] = {0.0, 0.0, 0.0};
        DeltaT[2] = deltaT();
        Accel[2] = imuData.accel.y - bias;
		vel = vel + ((Accel[0] + 4*Accel[1] + Accel[2])*((DeltaT[2] + DeltaT[1] + DeltaT[0])/3.0));
        //Serial.println(((Accel[0] + 4*Accel[1] + Accel[2])*(DeltaT[2] + DeltaT[1] + DeltaT[0])/3.0), 6);
        //Serial.print(Accel[0], 6); Serial.print("\t");Serial.print(Accel[1], 6); Serial.print("\t");Serial.println(Accel[2], 6);
        //vel = vel + (((imuData.accel.y-bias) + oldAccel[1])/2.0)*oldDeltaT[2];
        //vel = vel +(imuData.accel.y-bias)*oldDeltaT[2];
		Accel[0] = Accel[1];
        Accel[1] = Accel[2];
        DeltaT[0] = DeltaT[1];
        DeltaT[1] = DeltaT[2];
		return vel;
	}

    //zera o sleep bit do registrador de power management
    void imuStart(){

        Wire.beginTransmission(IMU_I2C_ADDRESS);
        Wire.write(PWR_MGMT_1);     
        Wire.write(0);              //writes 0 to wake the imu
        #if IMU_DEBUG
        uint8_t error = Wire.endTransmission();
        Serial.print("start: ");
        Serial.println(error);
        #else
        Wire.endTransmission();
        #endif

        Wire.beginTransmission(IMU_I2C_ADDRESS);
        Wire.write(IMU_FIFO_EN);     
        Wire.write(0xF8);              //writes 0 to wake the imu
        #if IMU_DEBUG
		error = Wire.endTransmission();
		Serial.print("start: ");
		Serial.println(error);
        #else
        Wire.endTransmission();
        #endif
    }

    /* Configuring accel scale
       0 -> +/-2g; 1 -> +/-4g; 2 -> +/-8g; 3 -> +/-16g */
    void imuAccelScale(uint8_t scale){
        scale = scale << 3;
        Wire.beginTransmission(IMU_I2C_ADDRESS);
        Wire.write(IMU_ACELL_CONFIG);
        Wire.write(scale);
        #if IMU_DEBUG
		uint8_t error = Wire.endTransmission();
		Serial.print("accel config: ");
		Serial.println(error);
        #else
            Wire.endTransmission();
        #endif
    }

    /* Configuring gyro scale
       0 -> +/-250; 1 -> +/-500; 2 -> +/-1000; 3 -> +/-2000 */
    void imuGyroScale(uint8_t scale){
        scale = scale << 3;
        Wire.beginTransmission(IMU_I2C_ADDRESS);
        Wire.write(IMU_GYRO_CONFIG);
        Wire.write(scale);
        #if IMU_DEBUG
		uint8_t error = Wire.endTransmission();
		Serial.print("gyro config: ");
		Serial.println(error);
        #else
            Wire.endTransmission();
        #endif
    }

    //lê size bytes a partir do endereço address e salva em buffer
    void imuRegRead(uint8_t address, size_t size, int8_t *buffer){
        Wire.beginTransmission(IMU_I2C_ADDRESS);
        Wire.write(address);
        #if IMU_DEBUG
		uint8_t error = Wire.endTransmission();
		Serial.print("imuRegRead: ");
		Serial.println(error);
        #else
		Wire.endTransmission();
        #endif

        Wire.requestFrom((uint8_t)IMU_I2C_ADDRESS, size); 
        for(uint16_t i = 0; i < size; i++)
            buffer[i] = Wire.read();
    }

    //concatena size bytes, em pares, de from e salva em to
    void to16(int16_t *to, int8_t *from, uint8_t size){
        for(uint8_t i = 0; i < size; i++)
            to[i] = ((int16_t)from[i*2]<<8) | from[(i*2)+1];
    }

    //faz a leitura do acelerômetro e calcula o valor em m/s usando accelScale[scale]
    void accelRead(int8_t scale){
        int8_t regBuffer[6];
        int16_t rawBuffer[3];
        imuRegRead(IMU_ACCEL_START, 6, regBuffer);
        to16(rawBuffer, regBuffer, 3);
        accel.x = (rawBuffer[0]*9.81)/accelScale[scale];
        accel.y = (rawBuffer[1]*9.81)/accelScale[scale];
        accel.z = (rawBuffer[2]*9.81)/accelScale[scale];
    }

    //faz a leitura do giroscópio e calcula o valor em graus/s usando gyroScale[scale]
    void gyroRead(int8_t scale){
        int8_t regBuffer[6];
        int16_t rawBuffer[3];
        imuRegRead(IMU_GYRO_START, 6, regBuffer);
        to16(rawBuffer, regBuffer, 3);
        gyro.x = rawBuffer[0]/gyroScale[scale] - gyroOffset.x;
        gyro.y = rawBuffer[1]/gyroScale[scale] - gyroOffset.y;
        gyro.z = rawBuffer[2]/gyroScale[scale] - gyroOffset.z;
    }

    //retorna temperatura lida em celcius
    double tempRead(){
        int8_t regBuffer[2];
        int16_t rawBuffer[1];
        double temp;
        imuRegRead(IMU_TEMP_START, 2, regBuffer);
        to16(rawBuffer, regBuffer, 1);
        temp = (rawBuffer[0]/340.0) + 36.53;
        return temp;
    }

    //filtro de leitura do acelerômetro e giroscópio
    void mediaMovel(){
        double alpha = 0.9;
        imuData.accel.x = (alpha*imuData.accel.x) + (1-alpha)*accel.x;
        imuData.accel.y = (alpha*imuData.accel.y) + (1-alpha)*accel.y;
        imuData.accel.z = (alpha*imuData.accel.z) + (1-alpha)*accel.z;
        imuData.gyro.x = (alpha*imuData.gyro.x) + (1-alpha)*gyro.x;
        imuData.gyro.y = (alpha*imuData.gyro.y) + (1-alpha)*gyro.y;
        imuData.gyro.z = (alpha*imuData.gyro.z) + (1-alpha)*gyro.z;
    }

    void getGyroBias(){
        axis offset = {0};
        for(uint16_t i=0; i<5000; i++){
			gyroRead(0);
			offset.x += gyro.x;
			offset.y += gyro.y;
			offset.z += gyro.z;
		}
        gyroOffset.x = offset.x / 5000.0;
        gyroOffset.y = offset.y / 5000.0;
        gyroOffset.z = offset.z / 5000.0;
    }

    //configura o IMU para leitura
    void Setup(){
        Wire.begin();

        imuStart();
        imuAccelScale(0);
        imuGyroScale(0);

        getGyroBias();
    }

    //chama a leitura do acelerômetro e giroscópio
    imuAll imuRead(){
        accelRead(0);
        gyroRead(0);
        imuData.accel = accel;
        imuData.gyro = gyro;
        imuData.temp = tempRead();
		//imuData.velocidades.lin = linearVel(0);
		imuData.velocidades.ang = gyro.z;
        //mediaMovel();
        return imuData;
    }

}