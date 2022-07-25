
#include "data_trans.h"

/*
 *  需要在当前目录下生成一个arduino_secrets.h的文件，用于保存WIFI SSID和密码
 */
#define SECRET_SSID ""
#define SECRET_PASS ""

//
//struct wifi_info {
//  char ssid[32];
//  char password[32];
//};


static struct wifi_info wifi_list[] = {
  { "test1", "123456" },
  { "test2", "654321" },
  { "test3", "888888" }   
};
