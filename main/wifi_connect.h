#ifndef WIFI_CONNECT_H
#define WIFI_CONNECT_H

// CHANGE THESE TO YOUR NETWORK CREDENTIALS
// Ideally, use idf.py menuconfig -> Example Configuration, but this is quicker for a prototype.
#define WIFI_SSID      "MY_ROBOT_WIFI"
#define WIFI_PASS      "robot123"

void wifi_init_sta(void);

#endif
