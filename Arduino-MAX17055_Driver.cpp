/**********************************************************************
*
* MIT License
*
* Copyright (c) 2018 Awot Ghirmai
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
* 
* Authors: 
* Awot Ghirmai; ghirmai.awot@gmail.com
* Ole Dreessen; ole.dreessen@maximintegrated.com
* 
**********************************************************************/

#include <Arduino-MAX17055_Driver.h>
#include <Wire.h>

/**********************************************************************
* @brief MAX17055 - The MAX17055 is a low 7μA operating current fuel gauge that implements 
* Maxim ModelGauge™ m5 EZ algorithm. ModelGauge m5 EZ makes fuel gauge implementation
* easy by eliminating battery characterization requirements and simplifying host software interaction. 
* The ModelGauge m5 EZ robust algorithm provides tolerance against battery diversity for most 
* lithium batteries and applications. ModelGauge m5 EZ algorithm combines the short-term
* accuracy and linearity of a coulomb counter with the longterm stability of a voltage-based fuel 
* gauge, along with temperature compensation to provide industry-leading fuel gauge accuracy. 
* The MAX17055 automatically compensates for cell aging, temperature, and discharge rate, and 
* provides accurate state of charge (SOC in %) and remaining capacity in milliampere-hours (mAh). 
* As the battery approaches the critical region near empty, the ModelGauge m5 algorithm invokes 
* a special compensation that eliminates any error. It also provides three methods for reporting 
* the age of the battery: reduction in capacity, increase in battery resistance, and cycle odometer.
* The MAX17055 provides precision measurements of current, voltage, and temperature. Temperature 
* of the battery pack is measured using an internal temperature measurement or external thermistor. 
* A 2-wire I2C interface provides access to data and control registers. The MAX17055 is available 
* in a tiny, lead-free 0.4mm pitch 1.4mm x 1.5mm, 9-pin WLP package, and a 2mm x 2.5mm, 10-pin 
* TDFN package.
*
* Resources can be found at
* https://www.maximintegrated.com/en/products/power/battery-management/MAX17055.html
* https://www.maximintegrated.com/en/app-notes/index.mvp/id/6365
* https://www.maximintegrated.com/en/app-notes/index.mvp/id/6358
**********************************************************************/


// Constructors 
MAX17055::MAX17055(void)
  
{
	//doesn't need anything here
}

MAX17055::MAX17055(uint16_t batteryCapacity)
{
	//calcuation based on AN6358 page 13 figure 1.3 for Capacity, but reversed to get the register value 
	writeReg16Bit(DesignCap, batteryCapacity*2);
}
// Public Methods
bool MAX17055::init(void (*wait)(uint32_t), uint16_t batteryCapacity, uint16_t vEmpty, uint16_t vRecovery, uint8_t modelID, bool vCharge, float resistSensor)
{
    Wire.beginTransmission(I2CAddress);
    byte error = Wire.endTransmission();
    if (error == 0) //Device Acknowledged
    {
        // TODO: never used anyway, values other than 0.01 not supported ?
        setResistSensor(resistSensor);

        // see MAX17055 Software Implementation Guide
        // 1.
        bool POR = getPOR();
        if (POR)
        {
            // 2. do not continue until FSTAT.DNR == 0
            while(readReg16Bit(FStat)&1) {
                wait(10);
            }

            // 3. Initialize configuration
            uint16_t hibCfg = readReg16Bit(HibCfg);
            writeReg16Bit(CommandReg, 0x90);
            writeReg16Bit(HibCfg, 0x0);
            writeReg16Bit(CommandReg, 0x0);

            // 3.1 OPTION 1 EZ Config (no INI file is needed): 
            setCapacity(batteryCapacity);
            writeReg16Bit(DQAcc, batteryCapacity/32);
            // writeReg16Bit(IchgTerm, 0x640); // leave default for now
            setEmptyVoltage(vEmpty, vRecovery);

            // leave out dQAcc for now
            setModelCfg(vCharge, modelID);

            // Do not continue until ModelCFG.Refresh == 0
            while (readReg16Bit(ModelCfg) & 0x8000) {
                wait(10);
            }
            writeReg16Bit(HibCfg, hibCfg); // Restore Original HibCFG value 

            // 4. clear POR bit
            resetPOR();
        }
        return true;
    }
    return false; //device not found
}

void MAX17055::getLearnedParameters(uint16_t& rComp0, uint16_t& tempCo, uint16_t& fullCapRep, uint16_t& cycles, uint16_t& fullCapNom) 
{
    rComp0 = readReg16Bit(RComp0);
    tempCo = readReg16Bit(TempCo);
    fullCapRep = readReg16Bit(FullCapRep);
    cycles = readReg16Bit(Cycles);
    fullCapNom = readReg16Bit(FullCapNom);
}

void MAX17055::restoreLearnedParameters(void (*wait)(uint32_t), uint16_t rComp0, uint16_t tempCo, uint16_t fullCapRep, uint16_t cycles, uint16_t fullCapNom)
{
    writeReg16Bit(RComp0, rComp0);
    writeReg16Bit(TempCo, tempCo);
    writeReg16Bit(FullCapNom, fullCapNom);

    wait(350);
    
    uint16_t mixCap = (readReg16Bit(MixSOC)*readReg16Bit(FullCapNom))/25600;
    writeReg16Bit(MixCap, mixCap);
    writeReg16Bit(FullCapRep, fullCapRep);

    //Write dQacc to 200% of Capacity and dPacc to 200%  
    uint16_t dQacc = (FullCapNom / 16);
    writeReg16Bit(DPAcc, 0x0C80);
    writeReg16Bit(DQAcc, DQAcc); 

    wait(350);

    writeReg16Bit(Cycles, cycles);
}

bool MAX17055::getPOR() 
{
    return readReg16Bit(Status)&0x0002;
}

void MAX17055::resetPOR() 
{
    writeReg16Bit(Status,readReg16Bit(Status)&0xFFFD); //reset POR Status
}

void MAX17055::setCapacity(uint16_t batteryCapacity)
{
	//calcuation based on AN6358 page 13 figure 1.3 for Capacity, but reversed to get the register value 
	writeReg16Bit(DesignCap, batteryCapacity*2);	
}	

void MAX17055::setEmptyVoltage(uint16_t vEmpty, uint16_t vRecovery){
    uint16_t regVal = (vEmpty << 7) & 0xFF80;

    // vRecovery has a resolution of 40mV in the Reg
    regVal = regVal | ((vRecovery >> 2) & 0x007F);

	writeReg16Bit(VEmpty, regVal);
}

uint16_t MAX17055::getEmptyVoltage(){
	return (readReg16Bit(VEmpty) & 0xFF80) >> 7;
}

void MAX17055::setModelCfg(bool vChg, uint8_t modelID) {
  uint16_t val = 0x8000;
  if(vChg) {
    val = val | 0x400;
  }
  val = val | (modelID & 0xF0);

  writeReg16Bit(ModelCfg, val);
}

uint16_t MAX17055::getModelCfg(){
	return readReg16Bit(ModelCfg);
}

uint16_t MAX17055::getCycles(){
	return readReg16Bit(Cycles);
}

float MAX17055::getCapacity()
{
   	// uint16_t capacity_raw = readReg16Bit(RepCap);
   	uint16_t capacity_raw = readReg16Bit(DesignCap);
	return (capacity_raw * capacity_multiplier_mAH);
}

void MAX17055::setResistSensor(float resistorValue)
{
	resistSensor = resistorValue;
}

float MAX17055::getResistSensor()
{
	return resistSensor;
}

float MAX17055::getMinCurrent() //+ve current is charging, -ve is discharging
{
    float multiplier = 0.4 / resistSensor; // different resolution
   	int8_t current_raw = readReg16Bit(MaxMinCurr) & 0x00FF;
	return current_raw * multiplier;
}

float MAX17055::getMaxCurrent() //+ve current is charging, -ve is discharging
{
    float multiplier = 0.4 / resistSensor; // different resolution
   	int8_t current_raw = readReg16Bit(MaxMinCurr) >> 8;
	return current_raw * multiplier;
}

void MAX17055::resetMaxMinCurrent()
{
    writeReg16Bit(MaxMinCurr, 0x807F);
}


float MAX17055::getAverageCurrent() //+ve current is charging, -ve is discharging
{
   	int16_t current_raw = readReg16Bit(AvgCurrent);
	return current_raw * current_multiplier_mV;
}

float MAX17055::getInstantaneousCurrent() //+ve current is charging, -ve is discharging
{
   	int16_t current_raw = readReg16Bit(Current);
	return current_raw * current_multiplier_mV;
}

float MAX17055::getAverageVoltage()
{
   	uint16_t voltage_raw = readReg16Bit(AvgVCell);
	return voltage_raw * voltage_multiplier_V;
}

float MAX17055::getInstantaneousVoltage()
{
   	uint16_t voltage_raw = readReg16Bit(VCell);
	return voltage_raw * voltage_multiplier_V;
}

float MAX17055::getSOC()
{
   	uint16_t SOC_raw = readReg16Bit(RepSOC);
	return SOC_raw * percentage_multiplier;
}

float MAX17055::getTimeToEmpty()
{
	uint16_t TTE_raw = readReg16Bit(TimeToEmpty);
	return TTE_raw * time_multiplier_Hours;
}

float MAX17055::getTemperature() {
    uint16_t temp_raw= readReg16Bit(Temperature);
    return temp_raw * percentage_multiplier;
}

float MAX17055::getAge() {
    uint16_t age_raw= readReg16Bit(Age);
    return age_raw * percentage_multiplier ; //Return value is % age, with 100% being a fully healthy, new battery.
}

bool MAX17055::getPresent() { //TODO: Doesn't seem to detect battery removal/re-insertion
    uint16_t pres_raw= readReg16Bit(Status) & 8; //returns just the 4th bit, with 0 = battery present, 1 = battery missing
    return !pres_raw; //hence we invert to return true if battery is present
}

// Private Methods

void MAX17055::writeReg16Bit(uint8_t reg, uint16_t value)
{
  //Write order is LSB first, and then MSB. Refer to AN635 pg 35 figure 1.12.2.5
  Wire.beginTransmission(I2CAddress);
  Wire.write(reg);
  Wire.write( value       & 0xFF); // value low byte
  Wire.write((value >> 8) & 0xFF); // value high byte
  uint8_t last_status = Wire.endTransmission();
}

uint16_t MAX17055::readReg16Bit(uint8_t reg)
{
  uint16_t value = 0;  
  Wire.beginTransmission(I2CAddress); 
  Wire.write(reg);
  uint8_t last_status = Wire.endTransmission(false);
  
  Wire.requestFrom(I2CAddress, (uint8_t) 2); 
  value  = Wire.read();
  value |= (uint16_t)Wire.read() << 8;      // value low byte
  return value;
}
