#ifndef _USER_GPIO_H_
#define _USER_GPIO_H_

//gpio input
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_IO_0     4
#define AP_WAKE_BT_R        37   /*主控开关wifi*/
#define GPIO_INPUT_PIN_SEL  (1ULL<<AP_WAKE_BT_R)
//gpio output
#define BT_WAKE_AP_R 18  /*本体连接成功*/
#define WL_WAKE_AP_R 5   /*手机连接成功*/
//#define BT_RST_N_R   36  /*wifi statu*/  //这个是输入脚
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<BT_WAKE_AP_R) | (1ULL<<WL_WAKE_AP_R))

void gpio_init(void);

#endif