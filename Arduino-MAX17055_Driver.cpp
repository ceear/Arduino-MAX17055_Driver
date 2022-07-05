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
bool MAX17055::init(uint16_t batteryCapacity, uint16_t vEmpty, uint16_t vRecovery, uint8_t modelID, bool vCharge, 
              float resistSensor, bool& por, TwoWire *theWire, void (*wait)(uint32_t)) 
{
    _wire = theWire;
    _wait = wait;

    _wire->beginTransmission(I2CAddress);
    byte error = _wire->endTransmission();
    if (error == 0) //Device Acknowledged
    {
        // TODO: never used anyway, values other than 0.01 not supported ?
        setResistSensor(resistSensor);

        // see MAX17055 Software Implementation Guide
        // 1.
        por = getPOR();

        //set lerncfg
        //WriteAndVerifyRegister(LearnCfg, readReg16Bit(LearnCfg) | 0x70);

        if (por)
        {
            // 2. do not continue until FSTAT.DNR == 0
            while(readReg16Bit(FStat)&1) {
                _wait(10);
            }

            // 3. Initialize configuration
            uint16_t hibCfg = readReg16Bit(HibCfg);
            writeReg16Bit(CommandReg, 0x90);
            writeReg16Bit(HibCfg, 0x0);
            writeReg16Bit(CommandReg, 0x0);

            // 3.1 OPTION 1 EZ Config (no INI file is needed): 
            setCapacity(batteryCapacity);
            WriteAndVerifyRegister(DQAcc, batteryCapacity/32);
            WriteAndVerifyRegister(IchgTerm, 0x0640); // Default 0x0640
            setEmptyVoltage(vEmpty, vRecovery);

            uint16_t dQAcc=int(batteryCapacity/32);
            WriteAndVerifyRegister (0x46 , dQAcc*44138/batteryCapacity); //Write dPAcc
            setModelCfg(vCharge, modelID);

            //Set additional LiFePo4 settings
            if (modelID = LiFePO4) {
                // Min: 2900
                // Max: 2900 + 285 = 3185
                setOCV_Low_Lim(2900);
                setOCV_Delta(285);
            }

            // Do not continue until ModelCFG.Refresh == 0
            while (readReg16Bit(ModelCfg) & 0x8000) {
                _wait(20);
            }
            WriteAndVerifyRegister(hibCfg, 0x870c);
            
            //3.5 check for init error 
            // sometimes the max17055 is not initialized correctly which results in incorrect RepSOC and RepCap values. 
            // TODO: find status register which indicates this. for now check for SOC=100% and CAP=0

            _wait(50);
            if (! checkInit()) {
                return false;
            } else {
                // 4. clear POR bit
                resetPOR();
                return true;
            
            }
        }
        return true;
    }
    return false; //device not found
}

void MAX17055::getLearnedParameters(uint16_t& v_rComp0, uint16_t& v_tempCo, uint16_t& v_fullCapRep, uint16_t& v_cycles, uint16_t& v_fullCapNom, uint16_t& v_repCap, uint16_t& v_mixCap, bool& v_learnComp) 
{
    v_rComp0 = readReg16Bit(RComp0);
    v_tempCo = readReg16Bit(TempCo);
    v_fullCapRep = readReg16Bit(FullCapRep);
    v_cycles = readReg16Bit(Cycles);
    v_fullCapNom = readReg16Bit(FullCapNom);
    v_repCap = readReg16Bit(RepCap);
    v_mixCap = readReg16Bit(MixCap);
    v_learnComp = getLearnCfg();
}

void MAX17055::restoreLearnedParameters(uint16_t v_rComp0, uint16_t v_tempCo, uint16_t v_fullCapRep, uint16_t v_cycles, uint16_t v_fullCapNom, uint16_t v_repCap, uint16_t v_mixCap, bool v_learnCfg)
{
    WriteAndVerifyRegister(RComp0, v_rComp0);
    WriteAndVerifyRegister(TempCo, v_tempCo);
    WriteAndVerifyRegister(FullCapNom, v_fullCapNom);

    _wait(350);

    WriteAndVerifyRegister(FullCapRep, v_fullCapRep);

    //Write dQacc to 200% of Capacity and dPacc to 200%  
    uint16_t dQAcc = (FullCapNom / 16);
    writeReg16Bit(DPAcc, 0x0C80);
    writeReg16Bit(DQAcc, dQAcc); 

    _wait(350);

    WriteAndVerifyRegister(Cycles, v_cycles);
    //Restore Learn CFG
    if (v_learnCfg) {
        setLearnComp();
    }
    WriteAndVerifyRegister(RepCap, v_repCap);
    WriteAndVerifyRegister(MixCap, v_mixCap);
    
}

bool MAX17055::getPOR() 
{
    return readReg16Bit(Status)&0x0002;
}

void MAX17055::resetPOR() 
{
    writeReg16Bit(Status,readReg16Bit(Status)&0xFFFD); //reset POR Status
}

bool MAX17055::checkInit() 
{
    float repSOC = getSOC();
    float repCap = getRepCapacity();
    float mixCap = readReg16Bit(MixCap) * capacity_multiplier_mAH;
    float deltaCap = 0.0;
    if (mixCap > 0 && repCap > 0) {
        deltaCap = fabsf( ((mixCap - repCap) / repCap ) * 100);

    } else {
        deltaCap = 100.0;

    }
    
    //DEBUG - TO BE REMOVED
    Serial.println("===== INIT TEST =====");
    Serial.printf("%s        : %f\n","deltaCap",deltaCap);

    if ( (repSOC == 1.0 | repSOC == 99.0) && (deltaCap > 50 ) ) {
        //Serial.println("INIT ERROR");    
        return false;
    } else {
        //Serial.println("NO INIT ERROR FOUND");
        return true;
    }


}

void MAX17055::setCapacity(uint16_t batteryCapacity)
{
	//calcuation based on AN6358 page 13 figure 1.3 for Capacity, but reversed to get the register value 
	//writeReg16Bit(DesignCap, batteryCapacity*2);	
    WriteAndVerifyRegister(DesignCap, batteryCapacity*2);
}	

void MAX17055::setEmptyVoltage(uint16_t vEmpty, uint16_t vRecovery){
    uint16_t regVal = (vEmpty << 7) & 0xFF80;

    // vRecovery has a resolution of 40mV in the Reg
    regVal = regVal | ((vRecovery >> 2) & 0x007F);

	WriteAndVerifyRegister(VEmpty, regVal);
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

  WriteAndVerifyRegister(ModelCfg, val);
}

uint16_t MAX17055::getModelCfg(){
	return readReg16Bit(ModelCfg);
}

uint16_t MAX17055::getCycles(){
	return readReg16Bit(Cycles);
}

bool MAX17055::getLearnCfg(){
    // Learn Stage defaults to 0h, making the voltage fuel gauge dominate. Learn Stage then advances 
    // to 7h over the course of two full cell cycles to make the coulomb counter dominate.
	return ( (readReg16Bit(LearnCfg) & 0x70) == 112) ? true : false;
}

void MAX17055::setLearnComp(){
	WriteAndVerifyRegister(LearnCfg,readReg16Bit(LearnCfg) | 0x70);
}

bool MAX17055::getSOCAlert(){
    // dSOCi (State of Charge 1% Change Alert)
	return readReg16Bit(Status) & 0x80;
}

void MAX17055::resetSOCAlert() {
    writeReg16Bit(Status,readReg16Bit(Status)&0xFF7F); 
}

void MAX17055::setEmptySOCHold(float percentage){
    uint16_t socHold = readReg16Bit(SOCHold) & 0xFFE0;
    uint8_t emptySOCHold = (uint8_t) floor(percentage * 2) & 0x1F; 

    writeReg16Bit(SOCHold, socHold | emptySOCHold);
}

void MAX17055::setOCV_Low_Lim(uint16_t mVoltage){
    uint16_t val_ScOcvLim = readReg16Bit(ScOcvLim) & 0x7F;
    uint16_t regValue = ( (mVoltage - 2560) / 5) << 7 & 0xFF80;

    writeReg16Bit(ScOcvLim, val_ScOcvLim | regValue);
}

void MAX17055::setOCV_Delta(uint16_t mVoltage){
    uint16_t val_ScOcvLim = readReg16Bit(ScOcvLim) & 0xFF80;
    uint16_t regValue = (mVoltage / 2.5f);
    regValue = regValue & 0x7F;
    
    writeReg16Bit(ScOcvLim, val_ScOcvLim | regValue);
}

float MAX17055::getEmptySOCHold(){
    uint8_t emptySOCHold = readReg16Bit(SOCHold) & 0x001F;
    return (float) emptySOCHold / 2.0f;
}

float MAX17055::getCapacity()
{
   	// uint16_t capacity_raw = readReg16Bit(RepCap);
   	uint16_t capacity_raw = readReg16Bit(DesignCap);
	return (capacity_raw * capacity_multiplier_mAH);
}

float MAX17055::getRepCapacity()
{
   	uint16_t capacity_raw = readReg16Bit(RepCap);
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

uint8_t MAX17055::getUserSOC()
{
   	uint16_t SOC_raw = readReg16Bit(RepSOC);
	return (int)round( (SOC_raw * percentage_multiplier) * 1000.0 ) / 1000.0;
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
  _wire->beginTransmission(I2CAddress);
  _wire->write(reg);
  _wire->write( value       & 0xFF); // value low byte
  _wire->write((value >> 8) & 0xFF); // value high byte
  uint8_t last_status = _wire->endTransmission();
}

uint16_t MAX17055::readReg16Bit(uint8_t reg)
{
  uint16_t value = 0;  
  _wire->beginTransmission(I2CAddress); 
  _wire->write(reg);
  uint8_t last_status = _wire->endTransmission(false);
  
  _wire->requestFrom(I2CAddress, (uint8_t) 2); 
  value  = _wire->read();
  value |= (uint16_t)_wire->read() << 8;      // value low byte
  return value;
}

void MAX17055::WriteAndVerifyRegister(uint8_t RegisterAddress, uint16_t RegisterValueToWrite)
{

	int attempt = 0;
	uint16_t RegisterValueRead;

	do {
		writeReg16Bit(RegisterAddress, RegisterValueToWrite);
		_wait(1);	//1ms
		RegisterValueRead = readReg16Bit(RegisterAddress);
	}
	while (RegisterValueToWrite != RegisterValueRead && attempt++<3);

	if (RegisterValueToWrite != RegisterValueRead)
	{
		Serial.printf("%s: 0x%.4x => 0x%.4x\n", "REG-WRITING ERROR", RegisterAddress, RegisterValueToWrite);
	}
	else
	{
		Serial.printf("%s   : 0x%.4x => 0x%.4x\n", "REG-WRITING OK", RegisterAddress, RegisterValueToWrite);
	}
}


void MAX17055::dumpparameter(String remark)
{

	Serial.println("========DUMP========");
	Serial.print("===");
	Serial.print(remark);
	Serial.println("===");

	// RComp0      = 0x38
	uint16_t tmp = readReg16Bit(RComp0);
	Serial.printf("%s          : 0x%.4x\n", "RComp", tmp);

	// TempCo      = 0x39
	tmp = readReg16Bit(TempCo);
	Serial.printf("%s         : 0x%.4x\n", "TempCo", tmp);

	// FullCapNom  = 0x23
	tmp = readReg16Bit(FullCapNom);
	Serial.printf("%s     : 0x%.4x (%f)\n", "FullCapNom", tmp, tmp *capacity_multiplier_mAH);

	//FullCapRep  = 0x10
	tmp = readReg16Bit(FullCapRep);
	Serial.printf("%s     : 0x%.4x (%f)\n", "FullCapRep", tmp, tmp *capacity_multiplier_mAH);

	//Cycles      = 0x17
	tmp = readReg16Bit(Cycles);
	Serial.printf("%s         : 0x%.4x\n", "Cycles", tmp);

	//MixCap      = 0x0F
	tmp = readReg16Bit(MixCap);
	Serial.printf("%s         : 0x%.4x (%f)\n", "MixCap", tmp, tmp *capacity_multiplier_mAH);

	// AtAvCap Register (DFh)
	tmp = readReg16Bit(0xDF);
	Serial.printf("%s        : 0x%.4x (%f)\n", "AtAvCap", tmp, tmp *capacity_multiplier_mAH);

	//RepCap      = 0x05
	tmp = readReg16Bit(RepCap);
	Serial.printf("%s         : 0x%.4x (%f)\n", "RepCap", tmp, tmp *capacity_multiplier_mAH);

	//MixSOC      = 0x0D
	tmp = readReg16Bit(MixSOC);
	Serial.printf("%s         : 0x%.4x (%f)\n", "MixSOC", tmp, tmp *percentage_multiplier);

	//AtAvSOC Register (DEh)
	tmp = readReg16Bit(0xDE);
	Serial.printf("%s        : 0x%.4x (%f)\n", "AtAvSOC", tmp, tmp *percentage_multiplier);

	//RepSOC      = 0x06
	tmp = readReg16Bit(RepSOC);
	Serial.printf("%s         : 0x%.4x (%f)\n", "RepSOC", tmp, tmp *percentage_multiplier);

	//DPAcc       = 0x46
	tmp = readReg16Bit(DPAcc);
	Serial.printf("%s          : 0x%.4x\n", "DPAcc", tmp);

	//DQAcc       = 0x45
	tmp = readReg16Bit(DQAcc);
	Serial.printf("%s          : 0x%.4x\n", "DQAcc", tmp);

	//LearnCFG       = 0x28
	tmp = readReg16Bit(LearnCfg);
	uint16_t learnCFGBit = tmp & 0x70;
	Serial.printf("%s       : 0x%.4x (0x%.4x)\n", "LearnCFG", learnCFGBit, tmp);

	//DesignCap   = 0x18
	tmp = readReg16Bit(DesignCap);
	Serial.printf("%s      : 0x%.4x (%f)\n", "DesignCap", tmp, tmp *capacity_multiplier_mAH);

	//ModelCfg    = 0xDB
	tmp = readReg16Bit(ModelCfg);
	Serial.printf("%s       : 0x%.4x\n", "ModelCfg", tmp);

	//SOCHold     = 0xD3
	tmp = readReg16Bit(0xD3);
	Serial.printf("%s        : 0x%.4x\n", "SOCHold", tmp);

	//ScOcvLim Register (D1h)
	tmp = readReg16Bit(0xD1);
	Serial.printf("%s       : 0x%.4x\n", "ScOcvLim", tmp);

	//IchgTerm
	tmp = readReg16Bit(IchgTerm);
	Serial.printf("%s       : 0x%.4x (%f)\n", "IchgTerm", tmp, tmp *current_multiplier_mV);

    //Status (00h)
	tmp = readReg16Bit(Status);
	Serial.printf("%s         : 0x%.4x\n", "Status", tmp);

	Serial.println("====================");

}