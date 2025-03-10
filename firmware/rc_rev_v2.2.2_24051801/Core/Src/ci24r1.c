// Copyright 2021 IOsetting <iosetting(at)outlook.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ci24r1.h"

// uint8_t cbuf[2],

// uint8_t *xbuf_data = xbuf + 1;

void for_delay_us(uint32_t nus)
{
    uint32_t Delay = nus * 168 / 4;
    do
    {
        __NOP();
    } while (Delay--);
}

// 1 on 0 off
void CI24R1_PowerCtrl(uint8_t en)
{
    if(en)
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
}

void CI24R1_WriteByte(uint8_t value)
{
    uint8_t i = 0;
    CI24R1_CLK_LOW();
    //for_delay_us(10);
    CI24R1_DATA_OUT();
    //for_delay_us(10);
    for (i = 0; i < 8; i++)
    {
        CI24R1_CLK_LOW();
        //for_delay_us(10);
        if (value & 0x80)
        {
            CI24R1_DATA_HIGH();
        }
        else
        {
            CI24R1_DATA_LOW();
        }
        //for_delay_us(10);
        CI24R1_CLK_HIGH();
        value = value << 1;
    }
    CI24R1_CLK_LOW();
}

uint8_t CI24R1_ReadByte(void)
{
    uint8_t i = 0, RxData;

    CI24R1_DATA_IN();
    //for_delay_us(10);
    CI24R1_CLK_LOW();
    //for_delay_us(10);
    for (i = 0; i < 8; i++)
    {
        RxData = RxData << 1;
        CI24R1_CLK_HIGH();
        //for_delay_us(10);
        if (CI24R1_DATA_READ())
        {
            RxData |= 0x01;
        }
        else
        {
            RxData &= 0xfe;
        }
        //for_delay_us(10);
        CI24R1_CLK_LOW();
    }
    CI24R1_CLK_LOW();
    return RxData;
}

void CI24R1_WriteReg(uint8_t reg, uint8_t value)
{
    CI24R1_NSS_LOW();
    CI24R1_WriteByte(reg);
    CI24R1_WriteByte(value);
    CI24R1_NSS_HIGH();
}

uint8_t CI24R1_ReadReg(uint8_t reg)
{
    uint8_t reg_val;
    CI24R1_NSS_LOW();
    CI24R1_WriteByte(reg);
    reg_val = CI24R1_ReadByte();
    CI24R1_NSS_HIGH();
    return reg_val;
}

void CI24R1_WriteCmd(uint8_t cmd)
{
    CI24R1_NSS_LOW();
    CI24R1_WriteByte(cmd);
    CI24R1_NSS_HIGH();
}

void CI24R1_WriteFromBuf(uint8_t reg, const uint8_t *pBuf, uint8_t len)
{
    uint8_t ctr;
    CI24R1_NSS_LOW();
    CI24R1_WriteByte(reg);
    for (ctr = 0; ctr < len; ctr++)
    {
        CI24R1_WriteByte(*pBuf++);
    }
    CI24R1_NSS_HIGH();
}

void CI24R1_ReadToBuf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t ctr;
    CI24R1_NSS_LOW();
    CI24R1_WriteByte(reg);
    for (ctr = 0; ctr < len; ctr++)
    {
        pBuf[ctr] = CI24R1_ReadByte();
    }
    CI24R1_NSS_HIGH();
}

void CI24R1_SetTxMode(void)
{
    uint8_t value;
    value = CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_CONFIG);
    value &= 0xFE;
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_CONFIG, value);
}

void CI24R1_SetRxMode(void)
{
    uint8_t value;
    value = CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_CONFIG);
    value |= 0x01;
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_CONFIG, value);
}

uint8_t CI24R1_SPI_Test(void)
{
    uint8_t i, *ptr = (uint8_t *)CI24R1_TEST_ADDR;
		uint8_t xbuf[5];
	
    CI24R1_CE_LOW();
    CI24R1_WriteReg(CI24R1_CMD_SELSPI, CI24R1_CMD_NOP);
    CI24R1_WriteFromBuf(CI24R1_CMD_W_REGISTER | CI24R1_REG_TX_ADDR, ptr, 5);
    CI24R1_ReadToBuf(CI24R1_CMD_R_REGISTER | CI24R1_REG_TX_ADDR, xbuf, 5);
    for (i = 0; i < 5; i++)
    {
        // UART1_TxHex(*(xbuf + i));
        if (*(xbuf + i) != *ptr++)
            return HAL_ERROR;
    }
    return HAL_OK;
}

void CI24R1_SetTxAddress(uint8_t *address)
{
    CI24R1_WriteFromBuf(CI24R1_CMD_W_REGISTER | CI24R1_REG_TX_ADDR, address, 5);
    CI24R1_WriteFromBuf(CI24R1_CMD_W_REGISTER | CI24R1_REG_RX_ADDR_P0, address, 5);
}

void CI24R1_SetRxAddress(uint8_t *address)
{
    CI24R1_WriteFromBuf(CI24R1_CMD_W_REGISTER | CI24R1_REG_TX_ADDR, address, 5);
    CI24R1_WriteFromBuf(CI24R1_CMD_W_REGISTER | CI24R1_REG_RX_ADDR_P0, address, 5);
}

void CI24R1_SetChannel(uint8_t channel)
{
    if (channel > 125)
        channel = 125;
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_RF_CH, channel);
}
#if 0 
void CI24R1_Init(void)
{
    CI24R1_CE_LOW();
#if (CI24R1_PLOAD_WIDTH == 0)
    // Enable dynamic payload length on pipe 0 and pipe 1
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_DYNPD, 0x03);
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_FEATURE, 0x07);
#else
    // Fixed payload length
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_DYNPD, 0x00);
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_FEATURE, 0x03);
    // Length of pipe 0
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_RX_PW_P0, CI24R1_PLOAD_WIDTH);
    // Length of pipe 1
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_RX_PW_P1, CI24R1_PLOAD_WIDTH);
#endif
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_CONFIG, 0x0E);
    // Enable auto ack all pipes
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_EN_AA, 0x3F);
    // Enable all pipes
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_EN_RXADDR, 0x3F);
    // Address width, 0x1:3bytes, 0x02:4bytes, 0x3:5bytes
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_SETUP_AW, 0x03);
    // Resend 500us and 3 times. interval: 250us * ([0, 15] + 1), retries: [0, 15]
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_SETUP_RETR, (0x01 << 4) | 0x03);
    // RF Data Rate 250K 11db
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_RF_SETUP, CI24R1_RF_SETUP_2M | CI24R1_RF_SETUP_11DB);
    CI24R1_CE_HIGH();
}
#else
void CI24R1_Init(void)
{
    CI24R1_CE_LOW();
#if (CI24R1_PLOAD_WIDTH == 0)
    // Enable dynamic payload length on pipe 0 and pipe 1
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_DYNPD, 0x01);
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_FEATURE, 0x07);
#else
    // Fixed payload length
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_DYNPD, 0x00);
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_FEATURE, 0x03);
    // Length of pipe 0
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_RX_PW_P0, CI24R1_PLOAD_WIDTH);
    // Length of pipe 1
    // CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_RX_PW_P1, CI24R1_PLOAD_WIDTH);
#endif
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_CONFIG, 0x0E);
    // Enable auto ack all pipes
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_EN_AA, 0x01);
    // Enable all pipes
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_EN_RXADDR, 0x01);
    // Address width, 0x1:3bytes, 0x02:4bytes, 0x3:5bytes
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_SETUP_AW, 0x03);
    // Resend 500us and 3 times. interval: 250us * ([0, 15] + 1), retries: [0, 15]
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_SETUP_RETR, 0xfa);
    // RF Data Rate 250K 11db
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_RF_SETUP, 0x0e);
    CI24R1_CE_HIGH();
}
#endif

uint8_t CI24R1_Tx(uint8_t *ucPayload, uint8_t length)
{
    uint8_t status;
#if (CI24R1_PLOAD_WIDTH == 0)
    CI24R1_WriteFromBuf(CI24R1_CMD_W_TX_PAYLOAD, ucPayload, length);
#else
    CI24R1_WriteFromBuf(CI24R1_CMD_W_TX_PAYLOAD, ucPayload, CI24R1_PLOAD_WIDTH);
#endif
    CI24R1_CE_HIGH();
    CI24R1_WriteCmd(CI24R1_CMD_SELIRQ);
    CI24R1_DATA_IN();
    while (CI24R1_DATA_READ()); //发送的while一般不会阻塞
    CI24R1_DATA_OUT();
    CI24R1_WriteCmd(CI24R1_CMD_SELSPI);
    status = CI24R1_ReadStatus();
    if (status & CI24R1_FLAG_MAX_RT)
    {
        CI24R1_WriteReg(CI24R1_CMD_FLUSH_TX, CI24R1_CMD_NOP);
    }
    // Clear status flags
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_STATUS, status);
    return status;
}

uint8_t CI24R1_WriteACKPload(uint8_t *ucPayload, uint8_t length)
{
    CI24R1_WriteFromBuf(CI24R1_CMD_W_ACK_PAYLOAD, ucPayload, length);
    return 0;
}

uint8_t CI24R1_ReadACKPload(uint8_t *rxbuf, uint8_t *rxbuflen)
{
    uint8_t rxplWidth;
    rxplWidth = CI24R1_ReadReg(CI24R1_CMD_R_RX_PL_WID);
    CI24R1_ReadToBuf(CI24R1_CMD_R_RX_PAYLOAD, rxbuf, rxplWidth);
    *rxbuflen = rxplWidth;
	return 0;
}

uint8_t CI24R1_Rx(uint8_t *rxbuf, uint8_t *rxbuflen)
{
    uint8_t status, rxplWidth;
    CI24R1_WriteReg(CI24R1_CMD_FLUSH_RX, CI24R1_CMD_NOP);
    CI24R1_WriteReg(CI24R1_CMD_SELIRQ, CI24R1_CMD_NOP);
    CI24R1_DATA_IN();
    while (CI24R1_DATA_READ());
    CI24R1_DATA_OUT();
    CI24R1_WriteReg(CI24R1_CMD_SELSPI, CI24R1_CMD_NOP);
    status = CI24R1_ReadStatus();
    // UART1_TxChar('>');
    // UART1_TxHex(status);
    if (status & CI24R1_FLAG_RX_READY)
    {
#if CI24R1_PLOAD_WIDTH == 0
        rxplWidth = CI24R1_ReadReg(CI24R1_CMD_R_RX_PL_WID);
#else
        rxplWidth = CI24R1_PLOAD_WIDTH;
#endif
        // Read RX to buffer
        CI24R1_ReadToBuf(CI24R1_CMD_R_RX_PAYLOAD, rxbuf, rxplWidth);
        *rxbuflen = rxplWidth;
        // Clear status flags
        CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_STATUS, status);
#if 0
				UART1_TxChar('>');
        for (i = 0; i < rxplWidth; i++)
        {
            UART1_TxHex(*(xbuf_data + i));
        }
#endif
    }
    return status;
}

uint8_t CI24R1_ReadStatus(void)
{
    return CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_STATUS);
}

void CI24R1_Switch1F_AF(uint8_t af)
{
    uint8_t val;

    val = CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_EN_AA);
    val &= 0x3F;
    val |= (af & 0x03) << 6;
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_EN_AA, val);

    val = CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_EN_RXADDR);
    val &= 0x3F;
    val |= (af & 0x0C) << 4;
    CI24R1_WriteReg(CI24R1_CMD_W_REGISTER | CI24R1_REG_EN_RXADDR, val);
}

uint8_t CI24R1_PrintStatus(void)
{
#if 0
    uint8_t i, status;

    UART1_TxString("[Config]");
    UART1_TxHex(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_CONFIG));

    UART1_TxString("  [EN_AA]");
    UART1_TxHex(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_EN_AA));

    UART1_TxString("  [EN_RxAddr]");
    UART1_TxHex(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_EN_RXADDR));

    UART1_TxString("  [AddrWidth]");
    UART1_TxHex(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_SETUP_AW));

    UART1_TxString("  [Retry]");
    UART1_TxHex(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_SETUP_RETR));

    UART1_TxString("\r\n[RF_Channel]");
    UART1_TxHex(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RF_CH));

    UART1_TxString("  [RF_Setup]");
    UART1_TxHex(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RF_SETUP));

    UART1_TxString("  [Observe_Tx]");
    UART1_TxHex(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_OBSERVE_TX));

    UART1_TxString("  [RSSI]");
    UART1_TxHex(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RSSI));

    UART1_TxString("\r\n[TxAddr]  ");
    CI24R1_ReadToBuf(CI24R1_CMD_R_REGISTER | CI24R1_REG_TX_ADDR, xbuf_data, 5);
    for (i = 0; i < 5; i++) {
        UART1_TxHex(*(xbuf_data + i));
    }

    UART1_TxString("\r\n[RxAddrP0]");
    CI24R1_ReadToBuf(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_ADDR_P0, xbuf_data, 5);
    for (i = 0; i < 5; i++) {
        UART1_TxHex(*(xbuf_data + i));
    }
    UART1_TxString(" [RxAddrP1]");
    CI24R1_ReadToBuf(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_ADDR_P1, xbuf_data, 5);
    for (i = 0; i < 5; i++) {
        UART1_TxHex(*(xbuf_data + i));
    }
    UART1_TxString(" [RxAddrP2]");
    UART1_TxHex(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_ADDR_P2));
    UART1_TxString(" [RxAddrP3]");
    UART1_TxHex(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_ADDR_P3));
    UART1_TxString(" [RxAddrP4]");
    UART1_TxHex(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_ADDR_P4));
    UART1_TxString(" [RxAddrP5]");
    UART1_TxHex(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_ADDR_P5AF));

    UART1_TxString("\r\n[0F_CRC]");
    CI24R1_Switch1F_AF(CI24R1_EN_RXADDR_CRC);
    UART1_TxHex(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_ADDR_P5AF));

    UART1_TxString(" [0F_OSC_C]");
    CI24R1_Switch1F_AF(CI24R1_EN_RXADDR_OSC_C);
    UART1_TxHex(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_ADDR_P5AF));

    UART1_TxString(" [0F_BT]");
    CI24R1_Switch1F_AF(CI24R1_EN_RXADDR_BT);
    UART1_TxHex(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_ADDR_P5AF));

    UART1_TxString(" [0F_BT_CRC_L/M/H]");
    CI24R1_Switch1F_AF(CI24R1_EN_RXADDR_BT_CRC_L);
    UART1_TxHex(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_ADDR_P5AF));
    CI24R1_Switch1F_AF(CI24R1_EN_RXADDR_BT_CRC_M);
    UART1_TxHex(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_ADDR_P5AF));
    CI24R1_Switch1F_AF(CI24R1_EN_RXADDR_BT_CRC_H);
    UART1_TxHex(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_ADDR_P5AF));

    UART1_TxString("\r\n[RX_PW_P0]");
    UART1_TxHex(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_PW_P0));
    UART1_TxString(" [RX_PW_P1]");
    UART1_TxHex(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_PW_P1));
    UART1_TxString(" [RX_PW_P2]");
    UART1_TxHex(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_PW_P2));
    UART1_TxString(" [RX_PW_P3]");
    UART1_TxHex(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_PW_P3));
    UART1_TxString(" [RX_PW_P4]");
    UART1_TxHex(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_PW_P4));
    UART1_TxString(" [RX_PW_P5]");
    UART1_TxHex(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_RX_PW_P5));

    UART1_TxString("\r\n[FIFO_Status]");
    UART1_TxHex(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_FIFO_STATUS));

    UART1_TxString("  [DynPloadWidth]");
    UART1_TxHex(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_DYNPD));

    UART1_TxString("  [Feature]");
    UART1_TxHex(CI24R1_ReadReg(CI24R1_CMD_R_REGISTER | CI24R1_REG_FEATURE));

    status = CI24R1_ReadStatus();
    UART1_TxString("\r\n[Status]");
    UART1_TxHex(status);
    UART1_TxString("\r\n\r\n");
    return status;
#else
    return 0;
#endif
}
