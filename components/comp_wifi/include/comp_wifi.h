#ifndef _USER_WIFI_H
#define _USER_WIFI_H

typedef enum {
    WIFI_CLOSE = 0x0,     /*!< Disable GPIO pull-up resistor */
    WIFI_OPEN = 0x1,      /*!< Enable GPIO pull-up resistor */
} mode_wifi;


//void ap_tcp_server_wifi(void);
void wifi_init_softap(void);

#endif