/*

   author: WJ Ding <wj_ding@hotmail.com>
   This work is licensed under the MIT license, see the file LICENSE for details.

   AD7190 SPI driver.


*/
#include <arduino.h>
#include <SPI.h>
#include "ad7190_spi.h"


AD7190_SPI::AD7190_SPI(char* name)
{
  activeState = false;
  spiClass = NULL;
  spiSS = 0;
  filterRate = AD7190_FILTER_RATE_1023; //默认 1023
  gainValue = AD7190_CONF_GAIN_1;  
  debugLevel = 0;

  size_t n = strlen(name);
  if (n > 15)
    // max len of name is 15
    n = 15;
  memset(deviceName, 0, sizeof(deviceName));
  memcpy(deviceName, name, n);
}

AD7190_SPI::~AD7190_SPI()
{
}
/***************************************************************************//**
   @brief init

   @param spi - spi handle.
          ss  - chip select
          slaveRdy - MISO pin

   @return uint8_t - 0: success; other value: failed
*******************************************************************************/
int8_t AD7190_SPI::init(SPIClass * spi, SPISettings setting, uint8_t spi_bus, uint8_t ss, uint8_t miso, uint8_t mosi)
{
  Serial.print("AD7190 init, spi_bus=");
  Serial.println(spi_bus);
  int8_t status = 0;
  if (!spi) {
    Serial.println("AD7190_SPI init error, spi is null!");
    return -1;
  }
  spiClass = spi;  //保存spi句柄
  spiBus = spi_bus;
  spiSS = ss;
  miso = miso;
  mosi = mosi;
  sRdy = miso;
  spiSetting = setting;

  // Reset device
  reset();
  delay(100);
  unsigned int regVal = getRegisterValue(AD7190_REG_ID, 1);
  if ((regVal & AD7190_ID_MASK) != ID_AD7190) {
    if (debugLevel >= SPI_DEBUG_ERROR) {
      Serial.print(getDeviceName());
      Serial.println(": Not found AD7190 module!");
    }
    status = -1;
    activeState = false;
    return status;
  }
  if (debugLevel >= SPI_DEBUG_INFO) {
    Serial.print(getDeviceName());
    Serial.println(": Find AD7190 module");
  }
  activeState = true;
  return status;
}


/***************************************************************************//**
   @brief Reads the value of a register.

   @param registerAddress - Address of the register.
   @param bytesNumber - Number of bytes that will be read.

   @return unsigned int - Value of the register.
*******************************************************************************/
uint32_t AD7190_SPI::getRegisterValue(byte registerAddress, unsigned char bytesNumber) {

  byte inByte = 0;           // incoming byte from the SPI
  unsigned int result = 0;   // result to return

  unsigned char address = AD7190_COMM_READ | AD7190_COMM_ADDR(registerAddress);

  if (debugLevel >= SPI_DEBUG_VERBOSE) {
    Serial.print("getRegisterValue address=0x");
    Serial.print(address, HEX);
    Serial.print(", bytesNumber=");
    Serial.println(bytesNumber);
  }

  spiClass->beginTransaction(spiSetting);
  // take the chip select low to select the device:
  digitalWrite(spiSS, LOW);
  delayMicroseconds(1);

  // send the device the register you want to read:
  spiClass->transfer(address);

  // send a value of 0 to read the first byte returned:
  result = spiClass->transfer(0x00);
  if (debugLevel >= SPI_DEBUG_VERBOSE) {
    Serial.print(String(bytesNumber) + ": 0x");
    Serial.print(result, HEX);
  }
  // decrement the number of bytes left to read:
  bytesNumber--;

  // if you still have another byte to read:
  while (bytesNumber > 0) {
    // shift the first byte left, then get the second byte:
    result = result << 8;
    inByte = spiClass->transfer(0x00);
    if (debugLevel >= SPI_DEBUG_VERBOSE) {
      Serial.print(" " + String(bytesNumber) + ": 0x");
      Serial.print(inByte, HEX);
    }
    // combine the byte you just got with the previous one:
    result = result | inByte;
    // decrement the number of bytes left to read:
    bytesNumber--;
  }

  if (debugLevel >= SPI_DEBUG_VERBOSE) {
    Serial.println();
  }
  // take the chip select high to de-select:
  digitalWrite(spiSS, HIGH);
  spiClass->endTransaction();
  return (result);
}

///***************************************************************************//**
//   @brief Reads the value of data register.
//
//   @param registerAddress - Address of the register.
//   @param bytesNumber - Number of bytes that will be read.
//
//   @return unsigned int - Value of the register.
//*******************************************************************************/
//uint32_t AD7190_SPI::getDataValue() {
//
//  byte inByte = 0;           // incoming byte from the SPI
//  unsigned int result = 0;   // result to return
//
//  unsigned char address = AD7190_COMM_READ | AD7190_COMM_ADDR(AD7190_REG_DATA);
//
//  // take the chip select low to select the device:
//  digitalWrite(spiSS, LOW);
//  delayMicroseconds(1);
//
//  // send a value of 0 to read the first byte returned:
//
//  // decrement the number of bytes left to read:
//  unsigned char bytesNumber = 3;
//
//  // if you still have another byte to read:
//  while (bytesNumber > 0) {
//    // shift the first byte left, then get the second byte:
//    result = result << 8;
//    inByte = spiClass->transfer(0x00);
//    if (debugLevel >= SPI_DEBUG_VERBOSE) {
//      Serial.print(" " + String(bytesNumber) + ": 0x");
//      Serial.print(inByte, HEX);
//    }
//    // combine the byte you just got with the previous one:
//    result = result | inByte;
//    // decrement the number of bytes left to read:
//    bytesNumber--;
//  }
//  if (debugLevel >= SPI_DEBUG_VERBOSE) {
//    Serial.println();
//  }
//
//  // take the chip select high to de-select:
//  digitalWrite(spiSS, HIGH);
//  return (result);
//}


/***************************************************************************//**
   @brief Writes data into a register.

   @param registerAddress - Address of the register.
   @param registerValue - Data value to write.
   @param bytesNumber - Number of bytes to be written.

   @return none.
*******************************************************************************/
void AD7190_SPI::setRegisterValue(unsigned char registerAddress, uint32_t registerValue, unsigned char bytesNumber)
{
  unsigned char writeCommand[5] = {0, 0, 0, 0, 0};
  unsigned char* dataPointer    = (unsigned char*)&registerValue;
  unsigned char bytesNr         = bytesNumber;

  writeCommand[0] = AD7190_COMM_WRITE | AD7190_COMM_ADDR(registerAddress);

  if (debugLevel >= SPI_DEBUG_VERBOSE) {
    Serial.print("setRegisterValue address=0x");
    Serial.print(writeCommand[0], HEX);
    Serial.print(", registerValue=0x");
    Serial.print(registerValue, HEX);
    Serial.print(", bytesNumber=");
    Serial.println(bytesNumber);
  }

  spiClass->beginTransaction(spiSetting);
  // take the chip select low to select the device:
  digitalWrite(spiSS, LOW);
  delayMicroseconds(1);

  while (bytesNr > 0) {
    writeCommand[bytesNr] = *dataPointer;
    if (debugLevel >= SPI_DEBUG_VERBOSE) {
      Serial.print(bytesNr);
      Serial.print(": 0x");
      Serial.print(writeCommand[bytesNr], HEX);
      Serial.print(" ");
    }
    dataPointer ++;
    bytesNr --;
  }
  spiClass->transfer(writeCommand, bytesNumber + 1);
  // take the chip select high to de-select:
  digitalWrite(spiSS, HIGH);
  spiClass->endTransaction();
  if (debugLevel >= SPI_DEBUG_VERBOSE) {
    Serial.println();
  }
}

int8_t AD7190_SPI::getDebugLevel()
{
  return debugLevel;
}

void AD7190_SPI::setDebugLevel(int8_t level)
{
  debugLevel = level;
}

/***************************************************************************//**
   @brief Resets the device.

   @return none.
*/
void AD7190_SPI::reset()
{
  if (debugLevel >= SPI_DEBUG_INFO) {
    Serial.print(getDeviceName());
    Serial.println(": AD7190 reset");
  }
  unsigned char register_word[6];
  register_word[0] = 0xFF;
  register_word[1] = 0xFF;
  register_word[2] = 0xFF;
  register_word[3] = 0xFF;
  register_word[4] = 0xFF;
  register_word[5] = 0xFF;
 
  spiClass->beginTransaction(spiSetting);
  digitalWrite(spiSS, LOW);
  spiClass->transfer(register_word, sizeof(register_word));
  digitalWrite(spiSS, HIGH);
  spiClass->endTransaction();
}

void AD7190_SPI::setFilterRate(int32_t filter)
{
  if (debugLevel >= SPI_DEBUG_INFO) {
    Serial.print("Set Filter Rate: ");
    Serial.println(filter);
  }
  filterRate = filter;
}

void AD7190_SPI::setGainValue(uint32_t gain)
{
  switch(gain)
  {
    case AD7190_CONF_GAIN_1:
    case AD7190_CONF_GAIN_8:
    case AD7190_CONF_GAIN_16:
    case AD7190_CONF_GAIN_32:
    case AD7190_CONF_GAIN_64:
    case AD7190_CONF_GAIN_128:
      gainValue = gain;
      break;
    default:
      Serial.println("Invalid GainValue: ");
      Serial.print(gain);
  }
  Serial.print("set gain=");
  Serial.println(gain);
}
   
/*
   @brief Set device to idle or power-down
   @param pwrMode - Select idle mode:1,  or power-down mode: 0
   @return none
*/
void AD7190_SPI::setPower(unsigned char pwrMode)
{
  unsigned int oldPwrMode = 0x0;
  unsigned int newPwrMode = 0x0;
  oldPwrMode = getRegisterValue(AD7190_REG_MODE, 3);
  oldPwrMode &= ~(AD7190_MODE_SEL(0x7));   // 清零，关闭旧模式
  newPwrMode = oldPwrMode | AD7190_MODE_SEL((pwrMode * (AD7190_MODE_IDLE)) | (!pwrMode * (AD7190_MODE_PWRDN)));
  setRegisterValue(AD7190_REG_MODE, newPwrMode, 3);
  Serial.print("newPwrMode:");
  Serial.println(newPwrMode, HEX);
}

/***************************************************************************//**
   @brief Selects the channel to be enabled.

   @param channel - Selects a channel.

   @return none.
*******************************************************************************/
void AD7190_SPI::selectChannel(unsigned short channel)
{
  if (debugLevel >= SPI_DEBUG_INFO) {
    Serial.print("selectChannel, ");
    Serial.println(channel);
  }

  unsigned int oldRegValue = 0x0;
  unsigned int newRegValue = 0x0;

  oldRegValue = getRegisterValue(AD7190_REG_CONF, 3);
  oldRegValue &= ~(AD7190_CONF_CHAN(0xFF));   // 清零通道
  newRegValue = oldRegValue | AD7190_CONF_CHAN(channel);
  setRegisterValue(AD7190_REG_CONF, newRegValue, 3);
  waitRdyGoLow();
}

/***************************************************************************//**
   @brief Performs the given calibration to the specified channel.

   @param mode - Calibration type.
   @param channel - Channel to be calibrated.

   @return none.
*******************************************************************************/
void AD7190_SPI::calibrate(unsigned char mode, unsigned char channel)
{
  if (debugLevel >= SPI_DEBUG_INFO) {
    String info = "calibrate: mode=" + String(mode) + ", channel=" + String(channel);
    Serial.println(info.c_str());
  }
  unsigned int oldRegValue = 0x0;
  unsigned int newRegValue = 0x0;

  selectChannel(channel);
  oldRegValue = getRegisterValue(AD7190_REG_MODE, 3);
  oldRegValue &= ~AD7190_MODE_SEL(0x7);
  newRegValue = oldRegValue | AD7190_MODE_SEL(mode);
  setRegisterValue(AD7190_REG_MODE, newRegValue, 3);
  waitRdyGoLow();
}

/***************************************************************************//**
   @brief Waits for RDY pin to go low.

   @return none.
*******************************************************************************/
void AD7190_SPI::waitRdyGoLow(void)
{
  uint32_t timeOutCnt = 0xFFFFF;

#if 1
  uint32_t count  = 0;
  int rdyState = digitalRead(sRdy);
  while (rdyState  && timeOutCnt--)
  {
    rdyState = digitalRead(sRdy);
    count++;
  }
  //Serial.println(count, HEX);
#else
  while (timeOutCnt--)
  {
    ;
  }
#endif

  if (debugLevel >= SPI_DEBUG_VERBOSE) {
    if (timeOutCnt == 0xFFFFFFFF) {
      Serial.println("timeOutCnt down to zero");
    }
  }
}

/***************************************************************************//**
   @brief Selects the polarity of the conversion and the ADC input range.

   @param polarity - Polarity select bit.
                     Example: 0 - bipolar operation is selected.
                              1 - unipolar operation is selected.
  @param range - Gain select bits. These bits are written by the user to select
                 the ADC input range.

   @return none.
*******************************************************************************/
void AD7190_SPI::rangeSetup(unsigned char polarity, unsigned char range)
{
  if (debugLevel >= SPI_DEBUG_VERBOSE) {
    Serial.print("rangeSetup, polarity=");
    Serial.print(polarity);
    Serial.print(", range=");
    Serial.println(range);
  }
  unsigned int oldRegValue = 0x0;
  unsigned int newRegValue = 0x0;

  oldRegValue = getRegisterValue(AD7190_REG_CONF, 3);
  oldRegValue &= ~(AD7190_CONF_UNIPOLAR | AD7190_CONF_GAIN(0x7));
  newRegValue = oldRegValue | (polarity * AD7190_CONF_UNIPOLAR) | AD7190_CONF_GAIN(range) | AD7190_CONF_BUF;
  setRegisterValue(AD7190_REG_CONF, newRegValue, 3);
}


/***************************************************************************//**
   @brief Read data from temperature sensor and converts it to Celsius degrees.

   @return temperature - Celsius degrees.
*******************************************************************************/
uint32_t AD7190_SPI::readTemperature(void)
{

  unsigned char temperature = 0x0;
  uint32_t dataReg = 0x0;

  rangeSetup(0, AD7190_CONF_GAIN_1);
  selectChannel(AD7190_CH_TEMP_SENSOR);
  dataReg = singleConversion();
  dataReg -= 0x800000;
  dataReg /= 2815;   // Kelvin Temperature
  dataReg -= 273;    //Celsius Temperature
  temperature = (unsigned int) dataReg;
  return temperature;
}

/*
   对内部零电平模式进行校准，校准通道为差分AIN1与AIN2， 然后AD7190_rangeSetup设置双极性工作模式，以及AD7190_CONF_GAIN_128增益，
   接着内置满量程的校准，校准完成后选择通道。
   设置连续转换模式、内部4.92MHz时钟，过滤器更新率1023、使能SINC2滤波器
   最后调用AD7190_getRegisterValue连续读取两次数据，确保数据的准确性
*/
void AD7190_SPI::adSetup()
{
  Serial.println("adSetup");

  unsigned int command = 0x0;

  /* calibrates channel AIN1(+) - AIN2(-) */
  calibrate(AD7190_MODE_CAL_INT_ZERO, AD7190_CH_AIN1P_AIN2M);
  calibrate(AD7190_MODE_CAL_INT_ZERO, AD7190_CH_AIN3P_AIN4M);

  /* Select unipolar operation and ADC/ input range to +-Vref/1. */
  rangeSetup(0, gainValue);

  calibrate(AD7190_MODE_CAL_INT_FULL, AD7190_CH_AIN1P_AIN2M);
  calibrate(AD7190_MODE_CAL_INT_FULL, AD7190_CH_AIN3P_AIN4M);

  /*Performs a single conversion */
  selectChannel(AD7190_CH_AIN1P_AIN2M | AD7190_CH_AIN3P_AIN4M);
  //selectChannel(AD7190_CH_AIN1P_AIN2M);
  //selectChannel(AD7190_CH_AIN3P_AIN4M);

  adModeConfig();
  
  waitRdyGoLow();
  getRegisterValue(AD7190_REG_DATA, 4);
  getRegisterValue(AD7190_REG_DATA, 4);

}


// 模式寄存器设置
void AD7190_SPI::adModeConfig()
{
  Serial.println("adModeConfig");
  uint32_t command = 0;
  // AD7190_MODE_CONT: 连续转换模式
  // AD7190_CLK_INT: 内部时钟源
  command = AD7190_MODE_SEL(AD7190_MODE_CONT) | AD7190_MODE_DAT_STA | AD7190_MODE_CLKSRC(AD7190_CLK_INT) | AD7190_MODE_RATE(filterRate) | AD7190_MODE_SINC3;
  setRegisterValue(AD7190_REG_MODE, command, 3);

}

/***************************************************************************//**
   @brief Returns the result of a single conversion.

   @return regData - Result of a single analog-to-digital conversion.
*******************************************************************************/
uint32_t AD7190_SPI::singleConversion(void)
{
  if (debugLevel >= SPI_DEBUG_VERBOSE) {
    Serial.println("singleConversion");
  }
  uint32_t command = 0x0;
  uint32_t regData = 0x0;

  command = AD7190_MODE_SEL(AD7190_MODE_SINGLE) | AD7190_MODE_CLKSRC(AD7190_CLK_INT) | AD7190_MODE_RATE(filterRate);
  setRegisterValue(AD7190_REG_MODE, command, 3);
  waitRdyGoLow();
  regData = getRegisterValue(AD7190_REG_DATA, 3);
  return regData;
}

/***************************************************************************//**
   @brief Returns the average of several conversion results.

   @return samplesAverage - The average of the conversion results.
*******************************************************************************/
uint32_t AD7190_SPI::continuousReadAvg(unsigned char sampleNumber)
{
  uint32_t samplesAverage = 0x0;
  unsigned char count = 0x0;
  uint32_t command = 0x0;

  //command = AD7190_MODE_SEL(AD7190_MODE_CONT) | AD7190_MODE_CLKSRC(AD7190_CLK_INT) | AD7190_MODE_RATE(0x060);
  command = AD7190_MODE_SEL(AD7190_MODE_CONT) | AD7190_MODE_DAT_STA | AD7190_MODE_CLKSRC(AD7190_CLK_INT) | AD7190_MODE_RATE(0x060);

  setRegisterValue(AD7190_REG_MODE, command, 3);
  for (count = 0; count < sampleNumber; count ++)
  {
    waitRdyGoLow();
    samplesAverage += getRegisterValue(AD7190_REG_DATA, 3);
  }
  samplesAverage = samplesAverage / sampleNumber;

  return samplesAverage ;
}

/*
   获取到24位数据后，“>>4”的动作，samplesAverage += (WEIGHT_getRegisterValue(WEIGHT_REG_DATA, 3)>>4);
   取20位数据，最大的范围也是20位数据
   注意：AD7190_MODE_DAT_STA没有设置时才能使用

*/
uint32_t AD7190_SPI::weightReadAvg(unsigned char sampleNumber)
{
#if 1
  uint32_t v = 0x0;
  unsigned int samplesAverage = 0x0;
  unsigned char count = 0x0;
  for (count = 0; count < sampleNumber; count ++)
  {
    waitRdyGoLow();
    v = getRegisterValue(AD7190_REG_DATA, 3);
    samplesAverage += (v >> 4);
  }
  samplesAverage = samplesAverage / sampleNumber;
  return samplesAverage ;
#else
  //    unsigned int samplesValue = 0x0;
  //    waitRdyGoLow();
  //    samplesValue = (getRegisterValue(AD7190_REG_DATA, 3)>>4);
  //    return samplesValue;
  unsigned int samplesValue = 0x0;
  waitRdyGoLow();
  samplesValue = (getDataValue() >> 4);
  return samplesValue;

#endif
}

// 4Bytes数据： 最后1 Byte时状态寄存器值
uint32_t AD7190_SPI::adcReadWithSatus()
{
  uint32_t value = 0x0;
  waitRdyGoLow();
  value = getRegisterValue(AD7190_REG_DATA, 4);
  return value ;
}


boolean AD7190_SPI::isActive()
{
  return activeState;
}

unsigned char AD7190_SPI::readStatus()
{
  unsigned int status = getRegisterValue(AD7190_REG_STAT, 1);
  return (unsigned char)(status & 0xFF);
}

char * AD7190_SPI::getDeviceName()
{
  return deviceName;
}

uint8_t AD7190_SPI::getSpiBus()
{
  return spiBus;
}
