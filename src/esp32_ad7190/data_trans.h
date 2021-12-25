#ifndef __DATA_TRANS_H__
#define __DATA_TRANS_H__

struct wifi_info {
  char ssid[32];
  char password[32];
};



void set_clock();
char* get_time(char* curTime);


void load_ad7190_config();
void save_ad7190_config(const char* key, int32_t value);

void send_to_server(char* ad_data);

#endif // __DATA_TRANS_H__
