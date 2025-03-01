#include "gui.h"
#include "OLED_I2C.h"

#define LCDBUFSIZE 8
u8 lcdbuf[LCDBUFSIZE] = {0};
#define CLR_LCDBUF memset(lcdbuf, 0, LCDBUFSIZE)

void gui_init(void)
{

    OLED_Init();
    OLED_Fill(0x00); // all black
    // while(1)
    {
        //OLED_ShowStr(0, 0, "TX:3.6V  ", 2);
        //OLED_ShowStr(0, 2, "RX:8.4V 200mA", 2);
        // OLED_ShowStr(0, 4, "ABCDEFGHIJKLMN", 2);
        // OLED_ShowStr(0, 6, "ABCDEFGHIJKLMN", 2);
    }

    // memset(lcdbuf, 0, 32);
    // snprintf(lcdbuf, 32, "RX:%.1fV %4dmA    ", ackpayload[0] / 10.0, ackpayload[1] * 10);
    // OLED_ShowStr(0, 2, lcdbuf, 2);
}

void show_tx_vcc(u8 adval, u8 force)
{
    static u8 advallast = 255;
    if (abs(adval - advallast) > 2)
    {
        advallast = adval;
        float vcc = adval;
        vcc = vcc / 256 * 33.3 * 2;
        CLR_LCDBUF;
        snprintf(lcdbuf, LCDBUFSIZE, "TX:%.1fV   ", vcc / 10);
        OLED_ShowStr(0, 0, lcdbuf, 2);
    }
}

/**
 * @brief 
 * 
 * @param adval 2024年7月9日 16:57:28 为了兼容更多设备，遥控器只显示，不计算。单位0.1v
 * @param force 
 */
void show_rx_vcc(u8 adval, u8 force)
{
    static u8 advallast = 255;
    if (abs(adval - advallast) > 2)
    {
        advallast = adval;

        float vcc = adval;
        vcc /= 10;

        CLR_LCDBUF;
        snprintf(lcdbuf, LCDBUFSIZE, "RX:%.1fV ", vcc);
        OLED_ShowStr(0, 4, lcdbuf, 2);
    }
} 

void show_rx_current(u8 adval, u8 force)
{
    static u8 advallast = 255;
    if (abs(adval - advallast) > 2)
    {
        advallast = adval;
        float current = adval;
        current = current * 3.33 * 1000 / 256 / 0.05 / 20;

        
        CLR_LCDBUF;
        snprintf(lcdbuf, LCDBUFSIZE, "%4dmA", (int)current);
        OLED_ShowStr(72, 4, lcdbuf, 2);
    }
}



void show_rx_limit(u8 limit, u8 force)
{
    static u8 limitlast = 255;

    if (limit != limitlast || force)
    {
        limitlast = limit;
        CLR_LCDBUF;
        if (limit & LIMIT_MAX1_TRIG)
        {
            strcat(lcdbuf, "u");
        }
        else if (limit & LIMIT_MIN1_TRIG)
        {
            strcat(lcdbuf, "n");
        }
        else
        {
            strcat(lcdbuf, "-");
        }

        if (limit & LIMIT_MAX2_TRIG)
        {
            strcat(lcdbuf, "u");
        }
        else if (limit & LIMIT_MIN2_TRIG)
        {
            strcat(lcdbuf, "n");
        }
        else
        {
            strcat(lcdbuf, "-");
        }
        if (limit & LIMIT_MAX3_TRIG)
        {
            strcat(lcdbuf, "u");
        }
        else if (limit & LIMIT_MIN3_TRIG)
        {
            strcat(lcdbuf, "n");
        }
        else
        {
            strcat(lcdbuf, "-");
        }

        OLED_ShowStr(72, 6, lcdbuf, 2);
    }
}
