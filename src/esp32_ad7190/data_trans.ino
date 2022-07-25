#include <Arduino.h>

#define NTP1  "ntp1.aliyun.com"
#define NTP2  "ntp2.aliyun.com"
#define NTP3  "ntp3.aliyun.com"



struct tm timeinfo;




void set_clock()
{
  configTime(8 * 3600, 0, NTP1, NTP2, NTP3);
  delay(1000);
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
  }

  Serial.println(&timeinfo, "%F %T %A"); // 格式化输出:2021-10-24 23:00:44 Sunday
  //Serial.print(asctime(&timeinfo));    //默认打印格式：Mon Oct 25 11:13:29 2021
}


char* get_time(char* curTime)
{
  struct tm timeInfo; //声明一个结构体
  if (!getLocalTime(&timeInfo)) //一定要加这个条件判断，否则内存溢出
  {
    Serial.println("Failed to obtain time");
    return "";
  }
  sprintf(curTime, "%4d-%02d-%02d %02d:%02d:%02d", (timeInfo.tm_year + 1900), (timeInfo.tm_mon + 1), timeInfo.tm_mday,
          timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
  return curTime;
}





// Load config from nvs
void load_ad7190_config()
{
  boolean flag = prefs.begin("ad7190");
  if (flag == false) {
    Serial.print("Preferences open error: ad7190");
    prefs.end();
    return;
  }

  int8_t ver = prefs.getChar("ver", 0);
  Serial.print("Load config: ver=");
  Serial.println(ver);

  hspiWeightZero = prefs.getLong("hspi_zero", 0);
  Serial.print("Load HSPI zero_data=");
  Serial.println(hspiWeightZero);

  hspiWeightProportion = prefs.getLong("hspi_prop", 0);
  Serial.print("Load HSPI proportion=");
  Serial.println(hspiWeightProportion);

  vspiWeightZero = prefs.getLong("vspi_zero", 0);
  Serial.print("Load VSPI zero_data=");
  Serial.println(vspiWeightZero);

  vspiWeightProportion = prefs.getLong("vspi_prop", 0);
  Serial.print("Load VSPI proportion=");
  Serial.println(vspiWeightProportion);

  //  Serial.print("Preferences free entries: ");
  //  Serial.println(prefs.freeEntries());

  prefs.end();
}

void save_ad7190_config(const char* key, int32_t value)
{
  Serial.print("save_ad7190_config: ");
  Serial.print(key);
  Serial.print(", ");
  Serial.println(value);

  boolean flag = prefs.begin("ad7190", false);
  if (flag == false) {
    Serial.println("Preferences open error!");
    prefs.end();
    return;
  }
  prefs.putChar("ver", 1);

  size_t ret = prefs.putLong(key, value);
  if (ret == 0) {
    Serial.println("save_ad7190_config Error!");
  }
  prefs.end();
}

void send_to_server(char* ad_data)
{
  if (!ad_data) {
    Serial.println("Error: send_to_server, ad_data is NULL");
    return;
  }
  if (wifiClient.connected())
  {
    int n = strlen(ad_data);
    int ret = wifiClient.write(ad_data, n);
    if (ret <= 0) {
      Serial.print("Send data to socket, len=");
      Serial.println(ret);
    }

    while (wifiClient.available())
    {
      String line = wifiClient.readStringUntil('\n'); //读取数据到换行符
//      Serial.print("read data：");
//      Serial.println(line);
//      remote_command(line.c_str());
      //wifiClient.write(line.c_str()); //将收到的数据回发
    }
  }
  else
  {
    Serial.println("Socket not connected, retry...");
    connect_server();
  }
}

void remote_command(const char * cmd)
{
  Serial.print("remote_command: ");
  Serial.println(cmd);
  if ( strcmp("reset", cmd)) {
    Serial.println("fake do reset from remote");
  }
}
