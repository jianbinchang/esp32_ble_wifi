#ifndef _USER_SDIO_H_
#define _USER_SDIO_H_

#include "stdint.h"

void sdio_init(void);
int send_data_to_sdio(unsigned char *buf, unsigned int len);

#endif