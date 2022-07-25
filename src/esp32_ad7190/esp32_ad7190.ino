#include <WiFi.h>
#include <SPI.h>
#include <Preferences.h>

#include "arduino_secrets.h"
#include "ad7190_spi.h"
#include "data_trans.h"

#define VER 0.9.4
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif


// SPI Interface
#define HSPI_ENABLED   // HSPI
//#define VSPI_ENABLED   // VSPI

// AD7190 filter param
#define HSPI_FILTER_VALUE   AD7190_FILTER_RATE_32
#define VSPI_FILTER_VALUE   AD7190_FILTER_RATE_32

// AD7190 gain
#define HSPI_GAIN_VALUE     AD7190_CONF_GAIN_128
#define VSPI_GAIN_VALUE     AD7190_CONF_GAIN_128

// sample delay (ms)
#define SAMPLE_HSPI_DELAY_INTERVAL     50
#define SAMPLE_VSPI_DELAY_INTERVAL     50

// calibration
//#define CALIBRATION_ENABLED


// SPI Mode
#define AD7190_SPI_MODE   SPI_MODE3


// send data to server
//#define SEND_DATA_ENABLED         1


#define MAX_QUEUE_LENGTH          30
#define MAX_MESSAGE_BYTES         64

#define DEUBG_AD_DATA   1
#define DEBUG_VERBOSE   1

#ifdef HSPI_ENABLED
boolean hspiEnabled = 1;
#else
boolean hspiEnabled = 0;
#endif

#ifdef VSPI_ENABLED
boolean vspiEnabled = 1;
#else
boolean vspiEnabled = 0;
#endif


// HSPI
#define HSPI_MISO   12
#define HSPI_MOSI   13
#define HSPI_SCLK   14
#define HSPI_SS     15

// VSPI
#define VSPI_MISO   19
#define VSPI_MOSI   23
#define VSPI_SCLK   18
#define VSPI_SS     5




void(* resetFunc) (void) = 0;
void task_process_data( void *pvParameters );
void task_hspi_data( void *pvParameters );
void task_vspi_data( void *pvParameters );

//-------------------------------------------------------------------
// below params need calibration
uint32_t hspiWeightZero = 539240;
uint32_t hspiWeightProportion = 41510;
uint32_t vspiWeightZero = 539240;
uint32_t vspiWeightProportion = 41510;

boolean calibrateMode = false;
Preferences prefs;
//-------------------------------------------------------------------
WiFiClient wifiClient;
static uint8_t wifi_index = 0;

const IPAddress serverIP(192, 168, 3, 37); // Server IP
uint16_t serverPort = 8000;                // Server Port

int32_t socket_error_num = 0;

//------------------------------------------------------------------
// SPI
SPIClass* hspi = NULL;
SPIClass* vspi = NULL;

static const int spiClk = 10000000; // 10 MHz
static SPISettings hspiSetting(spiClk, MSBFIRST, AD7190_SPI_MODE);
static SPISettings vspiSetting(spiClk, MSBFIRST, AD7190_SPI_MODE);

AD7190_SPI* ad7190_hspi = NULL;
AD7190_SPI* ad7190_vspi = NULL;

static QueueHandle_t spiQueue = NULL;

//-----------------------------------------------------------------
unsigned long last_ts = 0;

uint32_t sampleCount = 0;
uint32_t errorHspiCount = 0;
uint32_t confHspiCount = 0;
uint32_t errorVspiCount = 0;
uint32_t confVspiCount = 0;

void spi_init()
{
  unsigned char status = 1;

  if (hspiEnabled) {
    Serial.println("HSPI initializing...");
    hspi = new SPIClass(HSPI);

    //initialise hspi with default pins
    //SCLK = 14, MISO = 12, MOSI = 13, SS = 15

    hspi->begin();
    hspi->setClockDivider(SPI_CLOCK_DIV8);// SPI_CLOCK_DIV2
    hspi->setDataMode(AD7190_SPI_MODE);
    hspi->setBitOrder(MSBFIRST);

    //set up slave select pins as outputs as the Arduino API
    //doesn't handle automatically pulling SS low
    pinMode(HSPI_SS, OUTPUT); //HSPI SS
    Serial.println("HSPI initialize success.");
    delay(500);
  }

  if (vspiEnabled) {
    Serial.println("VSPI initializing...");
    vspi = new SPIClass(VSPI);

    //initialise hspi with default pins
    //SCLK = 18, MISO = 19, MOSI = 23, SS = 5

    vspi->begin();
    vspi->setClockDivider(SPI_CLOCK_DIV4);// SPI_CLOCK_DIV4: 4MHz, SPI_CLOCK_DIV16: 1MHz
    vspi->setDataMode(AD7190_SPI_MODE);
    vspi->setBitOrder(MSBFIRST);

    //set up slave select pins as outputs as the Arduino API
    //doesn't handle automatically pulling SS low
    pinMode(VSPI_SS, OUTPUT); //HSPI SS
    Serial.println("VSPI initialize success.");
    delay(500);
  }

}

wifi_info* get_next_wifi()
{
  uint8_t n = sizeof(wifi_list) / sizeof(wifi_info);
  wifi_info* wi = &wifi_list[wifi_index];
  Serial.print("Get WIFI info from config array: ");
  Serial.println(wifi_index);
  ++wifi_index;
  if (wifi_index < 0 || wifi_index >= n) {
    wifi_index = 0;
  }
  return wi;
}

// WIFI init
int8_t wifi_init()
{
  wifi_info* wi = get_next_wifi();

  Serial.println("WIFI initializing...");
  Serial.print("SSID: ");
  Serial.print(wi->ssid);
  Serial.print("  Password: ");
  Serial.println(wi->password);

  size_t retry_count = 0;
  WiFi.mode(WIFI_STA);
  WiFi.begin(wi->ssid,  wi->password);
  while (WiFi.status() != WL_CONNECTED) {   //未连接上
    Serial.print("Attempting to connect to Network: ");
    Serial.println(wi->ssid);
    delay(1000);
    retry_count++;
    if (retry_count > 5)
      return -1;
  }

  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  Serial.println("WIFI initialize success.");
  // set_clock();
  return 0;
}

void connect_server()
{
  Serial.print("Connecting socket server: IP=");
  Serial.print(serverIP);
  Serial.print(", Port=");
  Serial.println(serverPort);

  wifiClient.setTimeout(5);  // seconds
  if (wifiClient.connect(serverIP, serverPort)) {
    Serial.println("Connect socket success");

  } else {
    Serial.println("Connect socket failed.");
  }
}

void disconnect_server()
{
  Serial.println("Stop socket client");
  wifiClient.stop(); //关闭客户端
}


int8_t ad7190_hspi_init(int retryTime)
{
  Serial.println("ad7190_hspi_init");

  if (!ad7190_hspi) {
    ad7190_hspi = new AD7190_SPI("H");
    if (ad7190_hspi == NULL) {
      Serial.print("ad7190_hspi_init failed!");
      return -1;
    }
  }
  ad7190_hspi->setDebugLevel(SPI_DEBUG_ERROR);
  //ad7190_hspi->setDebugLevel(SPI_DEBUG_VERBOSE);
  int8_t ad_ret = ad7190_hspi->init(hspi, hspiSetting, HSPI, HSPI_SS, HSPI_MISO, HSPI_MOSI);
  while (ad_ret != 0 && retryTime--) {
    delay(1000);
    ad_ret = ad7190_hspi->init(hspi, hspiSetting, HSPI, HSPI_SS, HSPI_MISO, HSPI_MOSI);
    Serial.println("AD7190 hspi initalizing...");
  }
  if (ad_ret != 0) {
    Serial.println("AD7190 hspi init failed!");
    return ad_ret;
  }
  Serial.println("AD7190 hspi init success.");
  delay(10);
  uint32_t t = ad7190_hspi->readTemperature();
  Serial.print("Temperature: ");
  Serial.println(t);

  //delay(30*1000);
  ad7190_hspi->setFilterRate(HSPI_FILTER_VALUE);  // 设置滤波器字, 具体定义见ad7190_spi.h
  ad7190_hspi->setGainValue(HSPI_GAIN_VALUE);
  ad7190_hspi->weightConfig();
  delay(2);
  ad7190_hspi_lite_config();
  errorHspiCount = 0;
  confHspiCount = 0;
  return 0;
}
// 配置AD7190 HSPI数据采集参数
void ad7190_hspi_lite_config()
{
#ifdef DEBUG_VERBOSE
  Serial.println("ad7190_hspi_lite_config");
#endif
  ad7190_hspi->setFilterRate(HSPI_FILTER_VALUE);
  ad7190_hspi->setGainValue(HSPI_GAIN_VALUE);
  ad7190_hspi->weightConfig();
  ++confHspiCount;
}

int8_t ad7190_vspi_init(int retryTime)
{
  Serial.println("ad7190_vspi_init");
  if (!ad7190_vspi) {
    ad7190_vspi = new AD7190_SPI("V");
    if (ad7190_vspi == NULL) {
      Serial.print("ad7190_vspi_init failed!");
      return -1;
    }
  }
  ad7190_vspi->setDebugLevel(SPI_DEBUG_ERROR);
  int8_t ad_ret = ad7190_vspi->init(vspi, vspiSetting, VSPI, VSPI_SS, VSPI_MISO, VSPI_MOSI);
  while (ad_ret != 0 && retryTime > 0) {
    delay(1000);
    ad_ret = ad7190_vspi->init(vspi, vspiSetting, VSPI, VSPI_SS, VSPI_MISO, VSPI_MOSI);
    Serial.println("AD7190 vspi initalizing...");
    --retryTime;
  }
  if (ad_ret != 0) {
    Serial.println("AD7190 vspi init failed!");
    return ad_ret;
  }
  Serial.println("AD7190 vspi init success.");
  delay(10);
  uint32_t t = ad7190_vspi->readTemperature();
  Serial.print("Temperature: ");
  Serial.println(t);

  ad7190_vspi->setFilterRate(VSPI_FILTER_VALUE);  // 设置滤波器字, 具体定义见ad7190_spi.h
  ad7190_vspi->setGainValue(VSPI_GAIN_VALUE);
  ad7190_vspi->weightConfig();
  delay(2);
  errorVspiCount = 0;
  confVspiCount = 0;
  return 0;
}
// 配置AD7190 VSPI数据采集参数
void ad7190_vspi_lite_config()
{
#ifdef DEBUG_VERBOSE
  Serial.println("ad7190_vspi_lite_config");
#endif
  ad7190_vspi->setFilterRate(VSPI_FILTER_VALUE);
  ad7190_vspi->setGainValue(VSPI_GAIN_VALUE);
  ad7190_vspi->weightLiteConfig();
  ++confVspiCount;
}

void task_init()
{
  spiQueue = xQueueCreate( MAX_QUEUE_LENGTH, MAX_MESSAGE_BYTES);
  if (spiQueue == 0) {
    Serial.println("Error creating the spiQueue");
  }

  xTaskCreatePinnedToCore(
    task_process_data,
    "task_process_data",   
    2048,  // This stack size can be checked & adjusted by reading the Stack Highwater
    NULL,
    2,  // 任务优先级, with 3 (configMAX_PRIORITIES - 1) 是最高的，0是最低的.
    NULL,
    1);  // Run task at Core 1, xPortGetCoreID()

  xTaskCreatePinnedToCore(
    task_hspi_data,
    "task_hspi_data",   
    3072,  // This stack size can be checked & adjusted by reading the Stack Highwater
    NULL,
    2,  // 任务优先级, with 3 (configMAX_PRIORITIES - 1) 是最高的，0是最低的.
    NULL,
    0);  // Run task at Core 0, xPortGetCoreID()

  xTaskCreatePinnedToCore(
    task_vspi_data,
    "task_vspi_data",   
    3072,  // This stack size can be checked & adjusted by reading the Stack Highwater
    NULL,
    2,  // 任务优先级, with 3 (configMAX_PRIORITIES - 1) 是最高的，0是最低的.
    NULL,
    0);  // Run task at Core 0, xPortGetCoreID()
}

/******************************************************************************/
//  初始化
//--------------------------------------------------------------------------------//
void setup() {
  Serial.begin(115200);
  Serial.print("setup() function running on core: ");
  Serial.println(xPortGetCoreID());
  spi_init();

  if (hspiEnabled) {
    ad7190_hspi_init(5);
  }

  if (vspiEnabled) {
    ad7190_vspi_init(5);
  }

  int8_t wifi_ret = wifi_init();
  delay(5000);
  while (wifi_ret != 0) {
    delay(1000);
    wifi_ret = wifi_init();
  }
  delay(100);

  // 连接上位机服务器
#if SEND_DATA_ENABLED
  connect_server();
  delay(100);
#endif

  task_init();
}

void put_message(AD7190_SPI * ad7190_ptr, float w)
{
  char ad_data[MAX_MESSAGE_BYTES];
  memset(ad_data, 0, sizeof(ad_data));
  sprintf(ad_data, "{\"%s\", %.2f, %d}\n", ad7190_ptr->getDeviceName(), w, millis());
  xQueueSend(spiQueue, &ad_data,  ( TickType_t ) 100);

}

boolean check_spi_error()
{
  if (errorHspiCount > 100) {
    Serial.println("Error: errorHspiCount >100, init ad7190");
    ad7190_hspi_init(0);
    return false;
  }
  if (confHspiCount > 5) {
    Serial.println("Error: confHspiCount , init ad7190");
    ad7190_hspi_init(0);
    return false;
  }
  if (errorVspiCount > 100) {
    Serial.println("Error: errorVspiCount >100, init ad7190");
    ad7190_vspi_init(0);
    return false;
  }
  if (confVspiCount > 5) {
    Serial.println("Error: confVspiCount , init ad7190");
    ad7190_vspi_init(0);
    return false;
  }
  return true;
}
boolean check_ad_status(AD7190_SPI * ad7190_ptr, uint8_t adStatus)
{

  if (ad7190_ptr->getSpiBus() == HSPI) {
    if (adStatus == 0xFF) {
      // 0xFF
#if DEBUG_VERBOSE
      Serial.print("state:");
      Serial.print(adStatus, HEX);
      Serial.println(", call init");
#endif
      ++errorHspiCount;
      ad7190_hspi_init(1);
      //ad7190_hspi_lite_config();
      errorHspiCount = 0;
      delay(1);
      return false;
    } else if ((adStatus & 0x80) == 0x80) {
      // 0x80
#if DEBUG_VERBOSE
      //      Serial.print("state:");
      //      Serial.print(adStatus, HEX);
      //      Serial.println();
#endif
      delay(1);
      return false;
    }
  }

  if (ad7190_ptr->getSpiBus() == VSPI) {
    if (adStatus == 0xFF) {
      // 0xFF
#if DEBUG_VERBOSE
      Serial.print("state:");
      Serial.print(adStatus, HEX);
      Serial.println(", call init");
#endif
      ++errorVspiCount;
      ad7190_vspi_init(1);
      //ad7190_vspi_lite_config();
      errorVspiCount = 0;
      return false;
    } else if ((adStatus & 0x80) == 0x80) {
      // 0x80
#if DEBUG_VERBOSE
      //      Serial.print("state:");
      //      Serial.print(adStatus, HEX);
      //      Serial.println();
#endif
      delay(1);
      return false;
    }
  }
  return true;
}

boolean check_ad_weight_count(AD7190_SPI * ad7190_ptr, uint32_t weight_count)
{
  if (ad7190_ptr->getSpiBus() == HSPI) {
    if (weight_count == 0) {
#if DEBUG_VERBOSE
      Serial.print("hspi weight_count is 0, lite config, ");
      Serial.print(errorHspiCount);
      Serial.print(", ");
      Serial.println(confHspiCount);
#endif
      ad7190_hspi_lite_config();
      ++errorHspiCount;
      delay(1);
      return false;
    }
    if (weight_count == 0xFFFFF) {
#if DEBUG_VERBOSE
      Serial.println("hspi weight_count is 0xFFFFF, init config");
#endif
      ad7190_hspi_init(0);
      delay(1);
      return false;
    }


    if (weight_count < 0x70000 || weight_count > 0xF0000) {
#if DEBUG_VERBOSE
      Serial.print("hspi weight_count is overflow: 0x");
      Serial.println(weight_count, HEX);
#endif
      ad7190_hspi_lite_config();
      ++errorHspiCount;
      return false;
    }

  }

  if (ad7190_ptr->getSpiBus() == VSPI) {
    if (weight_count == 0) {
#if DEBUG_VERBOSE
      Serial.print("vspi weight_count is 0, lite config, ");
      Serial.print(errorVspiCount);
      Serial.print(", ");
      Serial.println(confVspiCount);
#endif
      ad7190_vspi_lite_config();
      ++errorVspiCount;
      delay(1);
      return false;
    }

    if (weight_count == 0xFFFFF) {
#if DEBUG_VERBOSE
      Serial.println("vspi weight_count is 0xFFFFF, init config");
#endif
      ad7190_vspi_init(0);
      delay(1);
      return false;
    }


    if (weight_count < 0x70000 || weight_count > 0xF0000) {
#if DEBUG_VERBOSE
      Serial.print("vspi weight_count is overflow: 0x");
      Serial.println(weight_count, HEX);
#endif
      ad7190_vspi_lite_config();
      ++errorVspiCount;
      return false;
    }
  }
  return true;
}


boolean spi_ad7190_data(AD7190_SPI * ad7190_ptr)
{
  if (!ad7190_ptr->isActive() && ad7190_ptr->getSpiBus() == HSPI) {
    ++errorHspiCount;
    if (errorHspiCount % 100 == 0) {
      Serial.print("No activate SPI device: ");
      Serial.println(ad7190_ptr->getDeviceName());
      ad7190_hspi_init(0);
      return false;
    }
  }
  if (!ad7190_ptr->isActive() && ad7190_ptr->getSpiBus() == VSPI) {
    ++errorVspiCount;
    if (errorVspiCount % 100 == 0) {
      Serial.print("No activate SPI device: ");
      Serial.println(ad7190_ptr->getDeviceName());
      ad7190_vspi_init(0);
      return false;
    }
  }

  if (!check_spi_error()) {
    //Serial.println("Error: check_spi_error");
    return false;
  }

  uint8_t adStatus = ad7190_ptr->readStatus() & 0xFF;
  //  if (!check_ad_status(ad7190_ptr, adStatus)) {
  //    return false;
  //  }

  if ((adStatus & 0x80) == 0x80) {
    // data not ready
    ad7190_ptr->waitRdyGoLow();
  }
  uint32_t weight_count = ad7190_ptr->weightReadAvg(1);
  if (!check_ad_weight_count(ad7190_ptr, weight_count)) {
    //Serial.println("Error: check_ad_weight_count");
    return false;
  }

  int32_t data_temp;
  float weight;
  if (ad7190_ptr->getSpiBus() == HSPI) {
    data_temp = weight_count - hspiWeightZero;
    weight = data_temp * 1000.0 / hspiWeightProportion;
    confHspiCount = 0;
  } else if (ad7190_ptr->getSpiBus() == VSPI) {
    data_temp = weight_count - vspiWeightZero;
    weight = data_temp * 1000.0 / vspiWeightProportion;
    confVspiCount = 0;
  }


#if DEUBG_AD_DATA
  if (sampleCount % 20 == 0) {
    Serial.print(ad7190_ptr->getDeviceName());
    Serial.print(": ts: ");
    Serial.print(millis());
    Serial.print(", AD: 0x");
    Serial.print(weight_count, HEX);
    Serial.print(", status: 0x");
    Serial.print(adStatus, HEX);
    Serial.print(", Weight Value: ");
    Serial.print(weight);
    Serial.print("(g)");
    Serial.println();
    //Serial.println(sampleCount);
  }
#endif


#if SEND_DATA_ENABLED
  put_message(ad7190_ptr, weight);
#endif //SEND_DATA_ENABLED

  return true;

}


//xTask : 负责发送SPI队列的数据
void task_process_data( void *pvParameters )
{
  Serial.println("task_process_data start");
  char data[MAX_MESSAGE_BYTES];

  while (1) {
    xQueueReceive(spiQueue, &data, portMAX_DELAY);
    send_to_server(data);
  }
}

//xTask : HSPI Data
void task_hspi_data( void *pvParameters )
{
  Serial.println("task_hspi_ta start");
  char data[MAX_MESSAGE_BYTES];

  while (1) {
    if (calibrateMode) {
      delay(10);
      continue;
    }
    if (hspiEnabled && spi_ad7190_data(ad7190_hspi)) {
      ++sampleCount;
      if (sampleCount % 1000 == 0) {
        //Serial.println("AD7190 sample in progress...");
        Serial.println(sampleCount);
      }
    }
    delay(SAMPLE_HSPI_DELAY_INTERVAL);
  }
}

void task_vspi_data( void *pvParameters )
{
  Serial.println("task_vspi_data start");
  char data[MAX_MESSAGE_BYTES];

  while (1) {
    if (calibrateMode) {
      delay(10);
      continue;
    }
    if (vspiEnabled && spi_ad7190_data(ad7190_vspi)) {
      ++sampleCount;
      if (sampleCount % 1000 == 0) {
        //Serial.println("AD7190 sample in progress...");
        Serial.println(sampleCount);
      }
    }
    delay(SAMPLE_VSPI_DELAY_INTERVAL);
  }
}
/******************************************************************************/
//
/******************************************************************************/
void loop() {

  if (WiFi.status() != WL_CONNECTED) {
    wifi_init();
  }

  unsigned long ts = millis();
  if ((ts - last_ts) >= 10) {
    last_ts = ts;

  }

  // Serial interactive command
#ifdef CALIBRATION_ENABLED
  process_cmd();
#endif

  delay(1);
}
