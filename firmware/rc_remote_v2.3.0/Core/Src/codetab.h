/************************************************************************************
*  Copyright (c), 2015, HelTec Automatic Technology co.,LTD.
*            All rights reserved.
*
* Http:    www.heltec.cn
* Email:   cn.heltec@gmail.com
* WebShop: heltec.taobao.com
*
* File name: OLED.h
* Project  : OLED
* Processor: STC89C52
* Compiler : Keil C51 Compiler
* 
* Author : Aaron Lee
* Version: 1.00
* Date   : 2014.3.24
* Email  : hello14blog@gmail.com
* Modification: none
* 
* Description:
* 1. 128*64点整OLED模块功能演示程序的字表，仅适用heltec.taobao.com所售产品;
* 2. 字表由打包资料中的“取字软件”计算得出;
* 3. 取字方式 -- 共阴、列行式、逆向输出
*
* Others: none;
*
* Function List: none;
*
* History: none;
*
*************************************************************************************/

/***************************16*16的点阵字体取模方式：共阴——列行式——逆向输出*********/
extern unsigned char F16x16[];


/************************************6*8的点阵************************************/
extern const unsigned char F6x8[][6];
/****************************************8*16的点阵************************************/
extern const unsigned char F8X16[];  

extern unsigned char BMP1[];
