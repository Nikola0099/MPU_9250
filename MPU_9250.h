#include <stdint.h>
#include <stm32f1xx_hal_conf.h>
#include <stm32f1xx_it.h>
#include <string.h>

/*    PINOUT:
 *  VCC -> 3.3V
 *  GND -> GND
 *  SCL -> B6
 *  SDA -> B7
 */

uint8_t DevAddress =  0b11010000; // Adresa uredjanja (nadjena u dokumentaciji)
uint8_t DevAddressRead =  0b11010001;

uint8_t DevAddressHall = 0b00011000; 
uint8_t DevAddressHallRead = 0b00011001; 

I2C_HandleTypeDef hi2c1;

void sensorSetup(uint8_t Sample_Rate_Divider, uint8_t Digital_Low_Pass_Filter, uint8_t Gyro_Full_Scale_Range, uint8_t Accel_Full_Scale_Range, 
                 uint8_t Sleep, uint8_t Temp_Disable, uint8_t Clock_Select)
{

    /*      sensorSetup funkcija služi podešavanju parametra senzora
     *  Digital_Low_Pass_Filter služi podešavanju koeficijenta low pass filtera, za više informacija pogledati tabelu na strani 13 mape Registra
     *  Gyro_Full_Scale_Range (vrednost može biti n = 0-3, gde će full scale range predstavljati +-250*( 2^(n) ) stepena/sekundi )
     *  Accel_Full_Scale_Range (vrednost može biti n = 0-3, gde će full scale range predstavljati +-( 2^(n+1) )g )
     *  Sleep, setovanjem vrednosti na 1 senzor prelazi u sleep mode (nema merenja)
     *  Temp_Disable, setovanjem vrednosti na 1 senzor preskače merenje temperature
     *  Clock_Select, služi za odabir clock-a (za više informacija pogledati tabelu na strani 40 data sheeta)
     */


    // POWER MANAGMENT 1 Registar (Adresa: 107)
    uint8_t powerManagment6, powerManagment3, powerManagment2_0, powerManagment;

    switch(Sleep)
    {
        case 0: powerManagment6 = 0; break;
        case 1: powerManagment6 = 64; break;
            default: powerManagment6 = 0;
    }


    switch(Temp_Disable)
    {
        case 0: powerManagment3 = 0; break;
        case 1: powerManagment3 = 128; break;
            default: powerManagment3 = 0;
    }

    powerManagment2_0 = Clock_Select;

    powerManagment = powerManagment6 | powerManagment3 | powerManagment2_0;

    uint8_t powerManagmentSet[2] = {107, powerManagment};
    HAL_I2C_Master_Transmit(&hi2c1, DevAddress, powerManagmentSet, 2, HAL_MAX_DELAY);


    // CONFIG Registar (Adresa: 26)
    uint8_t configSet[2] = {26, Digital_Low_Pass_Filter};
    HAL_I2C_Master_Transmit(&hi2c1, DevAddress, configSet, 2, HAL_MAX_DELAY);


    // Sample Rate Divider Registar (Adresa: 25)
    uint8_t sampleRateDividerSet[2] = {25, Sample_Rate_Divider};
    HAL_I2C_Master_Transmit(&hi2c1, DevAddress, sampleRateDividerSet, 2, HAL_MAX_DELAY);


    // Gyro Config Regitar (Adresa: 27)
    uint8_t gyroConfig4_3, gyroConfig;

    switch(Gyro_Full_Scale_Range)
    {
        case 0: gyroConfig4_3 = 0; break;   // +- 250 stepena/sekundi
        case 1: gyroConfig4_3 = 8; break;   // +- 500 stepena/sekundi
        case 2: gyroConfig4_3 = 16; break;  // +- 1000 stepena/sekundi
        case 3: gyroConfig4_3 = 24; break;  // +- 2000 stepena/sekundi
            default: gyroConfig4_3 = 0;     // +- 250 stepena/sekundi
    }

    gyroConfig = gyroConfig4_3;

    uint8_t gyroConfigSet[2] = {27, gyroConfig};
    HAL_I2C_Master_Transmit(&hi2c1, DevAddress, gyroConfigSet, 2, HAL_MAX_DELAY);


    // Accel Config Registar (Adresa: 28)
    uint8_t accelConfig4_3, accelConfig;

    switch(Accel_Full_Scale_Range)
    {
        case 0: accelConfig4_3 = 0; break;  // +- 2g
        case 1: accelConfig4_3 = 8; break;  // +- 4g
        case 2: accelConfig4_3 = 16; break; // +- 8g
        case 3: accelConfig4_3 = 24; break; // +- 16g
            default: accelConfig4_3 = 0;    // +- 2g
    }

    accelConfig = accelConfig4_3;

    uint8_t accelConfigSet[2] = {28, accelConfig};
    HAL_I2C_Master_Transmit(&hi2c1, DevAddress, accelConfigSet, 2, HAL_MAX_DELAY);


    // INT Pin/Bypass Enable Config Registar (Adresa: 55)
    uint8_t IntPinSet[2] = {55, 0x02};
    HAL_I2C_Master_Transmit(&hi2c1, DevAddress, IntPinSet, 2, HAL_MAX_DELAY);   // Prema tehničkoj dokumentaciji, potrebno je setovati prvi bit na 1

}

void readValue(int16_t *AccelX, int16_t *AccelY, int16_t *AccelZ, int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ, double *RealTemp, int16_t *HallX, int16_t *HallY, int16_t *HallZ)
{
    uint8_t DataBuffer[5], niz[5];
    uint8_t Accel_Xout_H = 0x3B;
    uint8_t Accel_Xout_L = 0x3C;
    
    uint8_t Accel_Yout_H = 0x3D;
    uint8_t Accel_Yout_L = 0x3E;
    
    uint8_t Accel_Zout_H = 0x3F;
    uint8_t Accel_Zout_L = 0x40;
    
    uint8_t Gyro_Xout_H = 67;
    uint8_t Gyro_Xout_L = 68;

    uint8_t Gyro_Yout_H = 69;
    uint8_t Gyro_Yout_L = 70;

    uint8_t Gyro_Zout_H = 71;
    uint8_t Gyro_Zout_L = 72;

    uint8_t Temp_Out_H = 65;
    uint8_t Temp_Out_L = 66;
    int16_t Temp;

    HAL_I2C_Master_Transmit(&hi2c1, DevAddress, &Accel_Xout_H, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, DevAddressRead, DataBuffer, 1, HAL_MAX_DELAY);
    niz[0] = DataBuffer[0];
    HAL_I2C_Master_Transmit(&hi2c1, (DevAddress), &Accel_Xout_L, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, DevAddressRead, DataBuffer, 1, HAL_MAX_DELAY);
    niz[1] = DataBuffer[0];
    *AccelX = (uint16_t)niz[0] << 8 | niz[1];

    HAL_I2C_Master_Transmit(&hi2c1, DevAddress, &Accel_Yout_H, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, DevAddressRead, DataBuffer, 1, HAL_MAX_DELAY);
    niz[0] = DataBuffer[0];
    HAL_I2C_Master_Transmit(&hi2c1, DevAddress, &Accel_Yout_L, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, DevAddressRead, DataBuffer, 1, HAL_MAX_DELAY);
    niz[1] = DataBuffer[0];
    *AccelY = (uint16_t)niz[0] << 8 | niz[1];

    HAL_I2C_Master_Transmit(&hi2c1, DevAddress, &Accel_Zout_H, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, DevAddressRead, DataBuffer, 1, HAL_MAX_DELAY);
    niz[0] = DataBuffer[0];
    HAL_I2C_Master_Transmit(&hi2c1, DevAddress, &Accel_Zout_L, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, DevAddressRead, DataBuffer, 1, HAL_MAX_DELAY);
    niz[1] = DataBuffer[0];
    *AccelZ = (uint16_t)niz[0] << 8 | niz[1];

    HAL_I2C_Master_Transmit(&hi2c1, DevAddress, &Gyro_Xout_H, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, DevAddressRead, DataBuffer, 1, HAL_MAX_DELAY);
    niz[0] = DataBuffer[0];
    HAL_I2C_Master_Transmit(&hi2c1, DevAddress, &Gyro_Xout_L, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, DevAddressRead, DataBuffer, 1, HAL_MAX_DELAY);
    niz[1] = DataBuffer[0];
    *GyroX = (uint16_t)niz[0] << 8 | niz[1];

    HAL_I2C_Master_Transmit(&hi2c1, DevAddress, &Gyro_Yout_H, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, DevAddressRead, DataBuffer, 1, HAL_MAX_DELAY);
    niz[0] = DataBuffer[0];
    HAL_I2C_Master_Transmit(&hi2c1, DevAddress, &Gyro_Yout_L, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, DevAddressRead, DataBuffer, 1, HAL_MAX_DELAY);
    niz[1] = DataBuffer[0];
    *GyroY = (uint16_t)niz[0] << 8 | niz[1];

    HAL_I2C_Master_Transmit(&hi2c1, DevAddress, &Gyro_Zout_H, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, DevAddressRead, DataBuffer, 1, HAL_MAX_DELAY);
    niz[0] = DataBuffer[0];
    HAL_I2C_Master_Transmit(&hi2c1, DevAddress, &Gyro_Zout_L, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, DevAddressRead, DataBuffer, 1, HAL_MAX_DELAY);
    niz[1] = DataBuffer[0];
    *GyroZ = (uint16_t)niz[0] << 8 | niz[1];

    HAL_I2C_Master_Transmit(&hi2c1, DevAddress, &Temp_Out_H, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, DevAddressRead, DataBuffer, 1, HAL_MAX_DELAY);
    niz[0] = DataBuffer[0];
    HAL_I2C_Master_Transmit(&hi2c1, DevAddress, &Temp_Out_L, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, DevAddressRead, DataBuffer, 1, HAL_MAX_DELAY);
    niz[1] = DataBuffer[0];
    Temp = (uint16_t)niz[0] << 8 | niz[1];
    *RealTemp = Temp/340.0 + 21;

    uint8_t Hall_Xout_H = 0x04;
    uint8_t Hall_Xout_L = 0x03;
    
    uint8_t Hall_Yout_H = 0x06;
    uint8_t Hall_Yout_L = 0x05;
    
    uint8_t Hall_Zout_H = 0x08;
    uint8_t Hall_Zout_L = 0x07;

    uint8_t configHallSet[2] = {0x0A, 0b00010110};
    HAL_I2C_Master_Transmit(&hi2c1, DevAddressHall, configHallSet, 2, HAL_MAX_DELAY);

    HAL_I2C_Master_Transmit(&hi2c1, DevAddressHall, &Hall_Xout_H, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, DevAddressHallRead, DataBuffer, 1, HAL_MAX_DELAY);
    niz[0] = DataBuffer[0];
    HAL_I2C_Master_Transmit(&hi2c1, DevAddressHall, &Hall_Xout_L, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, DevAddressHallRead, DataBuffer, 1, HAL_MAX_DELAY);
    niz[1] = DataBuffer[0];
    *HallX = (uint16_t)niz[0] << 8 | niz[1];

    HAL_I2C_Master_Transmit(&hi2c1, DevAddressHall, &Hall_Yout_H, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, DevAddressHallRead, DataBuffer, 1, HAL_MAX_DELAY);
    niz[0] = DataBuffer[0];
    HAL_I2C_Master_Transmit(&hi2c1, DevAddress, &Hall_Yout_L, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, DevAddressHallRead, DataBuffer, 1, HAL_MAX_DELAY);
    niz[1] = DataBuffer[0];
    *HallY = (uint16_t)niz[0] << 8 | niz[1];

    HAL_I2C_Master_Transmit(&hi2c1, DevAddressHall, &Hall_Zout_H, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, DevAddressHallRead, DataBuffer, 1, HAL_MAX_DELAY);
    niz[0] = DataBuffer[0];
    HAL_I2C_Master_Transmit(&hi2c1, DevAddressHall, &Hall_Zout_L, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, DevAddressHallRead, DataBuffer, 1, HAL_MAX_DELAY);
    niz[1] = DataBuffer[0];
    *HallZ = (uint16_t)niz[0] << 8 | niz[1];

    uint8_t configHallSet1[2] = {0x0A, 0b00010000};
    HAL_I2C_Master_Transmit(&hi2c1, DevAddressHall, configHallSet1, 2, HAL_MAX_DELAY);
}