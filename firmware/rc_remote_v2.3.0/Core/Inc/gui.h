#ifndef __GUI_H_
#define __GUI_H_

#include "common.h"


#define LIMIT_MAX1_TRIG 0x01
#define LIMIT_MIN1_TRIG 0x02
#define LIMIT_MAX2_TRIG 0x04
#define LIMIT_MIN2_TRIG 0x08
#define LIMIT_MAX3_TRIG 0x10
#define LIMIT_MIN3_TRIG 0x20


void gui_init(void);
void show_tx_vcc(u8 adval, u8 force);
void show_rx_vcc(u8 adval, u8 force);
void show_rx_current(u8 adval, u8 force);
void show_rx_limit(u8 limit, u8 force);

#endif
