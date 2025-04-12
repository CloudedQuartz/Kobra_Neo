/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once
#include "stdint.h"
#ifndef LCD_READ_ID
  #define LCD_READ_ID 0x04   // Read display identification information (0xD3 on ILI9341)
#endif
#ifndef LCD_READ_ID4
  #define LCD_READ_ID4 0xD3   // Read display identification information (0xD3 on ILI9341)
#endif

#define DATASIZE_8BIT    0//SPI_DATASIZE_8BIT
#define DATASIZE_16BIT   1//SPI_DATASIZE_16BIT
#define TFT_IO_DRIVER TFT_SPI
#define DMA_MAX_WORDS  0xFFFF


#define DMA_MINC_ENABLE   1
#define DMA_MINC_DISABLE  0

#ifndef __LCD_INIT_H
#define __LCD_INIT_H
#include "stdint.h"

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;
//#define WHITE         	 0xFFFF
//#define BLACK         	 0x0000	  
//#define BLUE           	 0x001F  
//#define BRED             0XF81F
//#define GRED 			       0XFFE0
//#define GBLUE			       0X07FF
//#define RED           	 0xF800
//#define MAGENTA       	 0xF81F
//#define GREEN         	 0x07E0
//#define CYAN          	 0x7FFF
//#define YELLOW        	 0xFFE0
//#define BROWN 			     0XBC40 //��ɫ
//#define BRRED 			     0XFC07 //�غ�ɫ
//#define GRAY  			     0X8430 //��ɫ
//#define DARKBLUE      	 0X01CF	//����ɫ
//#define LIGHTBLUE      	 0X7D7C	//ǳ��ɫ  
//#define GRAYBLUE       	 0X5458 //����ɫ
//#define LIGHTGREEN     	 0X841F //ǳ��ɫ
//#define LGRAY 			     0XC618 //ǳ��ɫ(PANNEL),���屳��ɫ
//#define LGRAYBLUE        0XA651 //ǳ����ɫ(�м����ɫ)
//#define LBBLUE           0X2B12 //ǳ����ɫ(ѡ����Ŀ�ķ�ɫ)

#define USE_HORIZONTAL 2  //���ú�������������ʾ 0��1Ϊ���� 2��3Ϊ����

//#define moni_spi
#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 320
#define LCD_H 480

#else
#define LCD_W 320
#define LCD_H 240
#endif


//-----------------LCD�˿ڶ���---------------- 
#define LCD_GPIO_PORRC               ((M4_PORT_TypeDef *)M4_PORT + 0x2A)
#define LCD_GPIO_POSRC               ((M4_PORT_TypeDef *)M4_PORT + 0x28)


#define SCLK          			   Pin05
#define MOSI					   	Pin04

#define  BL_port                   PortC
#define  BL_pin                    Pin00

#define  BL_ON                    (PORT_SetBits(BL_port, BL_pin))   


#define tft_CS_PORT                     (PortB)
#define tft_CS_PIN                      (Pin01)
#define tft_CS_H()                      (PORT_SetBits(tft_CS_PORT, tft_CS_PIN))
#define tft_CS_L()                      (PORT_ResetBits(tft_CS_PORT, tft_CS_PIN))


/* Rs Port/Pin definition */
#define tft_RS_PORT                       (PortB)
#define tft_RS_PIN                        (Pin00)
#define tft_RS_H()                        (PORT_SetBits(tft_RS_PORT, tft_RS_PIN))
#define tft_RS_L()                        (PORT_ResetBits(tft_RS_PORT, tft_RS_PIN))

/* Rs Port/Pin definition */
#define tft_RES_PORT                       (PortB)
#define tft_RES_PIN                        (Pin04)
#define tft_RES_H()                       (PORT_SetBits(tft_RS_PORT, tft_RES_PIN))
#define tft_RES_L()                       (PORT_ResetBits(tft_RS_PORT, tft_RES_PIN))

/* SPI_SCK Port/Pin definition */
#define SPI_SCK_PORT                    (PortC)
#define SPI_SCK_PIN                     (Pin05)
#define SPI_SCK_FUNC                    (Func_Spi2_Sck)

#define tft_SCK_H()                      (PORT_SetBits(SPI_SCK_PORT, SPI_SCK_PIN))
#define tft_SCK_L()                      (PORT_ResetBits(SPI_SCK_PORT, SPI_SCK_PIN))

/* SPI_MOSI Port/Pin definition */
#define SPI_MOSI_PORT                   (PortC)
#define SPI_MOSI_PIN                    (Pin04)
#define SPI_MOSI_FUNC                   (Func_Spi2_Mosi)

#define tft_MOSI_H()                      (PORT_SetBits(SPI_MOSI_PORT, SPI_MOSI_PIN))
#define tft_MOSI_L()                      (PORT_ResetBits(SPI_MOSI_PORT, SPI_MOSI_PIN))

///* SPI_MISO Port/Pin definition */
//#define SPI_MISO_PORT                   (PortE)
//#define SPI_MISO_PIN                    (Pin03)
//#define SPI_MISO_FUNC                   (Func_Spi3_Miso)

/* SPI unit and clock definition */
#define SPI_UNIT                        (M4_SPI2)
#define SPI_UNIT_CLOCK                  (PWC_FCG1_PERIPH_SPI2)

/* Choose SPI master or slave mode */
#define SPI_MASTER_MODE
//#define SPI_SLAVE_MODE
void delay_ms(u16 count);

void LCD_GPIO_Init(void);//��ʼ��GPIO
void LCD_Writ_Bus(u8 dat);//ģ��SPIʱ��
void LCD_WR_DATA8(u8 dat);//д��һ���ֽ�
void LCD_WR_DATA(u16 dat);//д�������ֽ�
void LCD_WR_REG(u8 dat);//д��һ��ָ��
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2);//�������꺯��
void LCD_Init(void);//LCD��ʼ��
void LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color);


void LCD_ShowIntNum(u16 x,u16 y,u16 num,u8 len,u16 fc,u16 bc,u8 sizey);
void Show_logo_secreen(void);
#endif


class TFT_SPI {
private:
//  static SPI_HandleTypeDef SPIx;
//  static DMA_HandleTypeDef DMAtx;
  static uint8_t data_type;
  static uint32_t ReadID(uint16_t Reg);
  static void Transmit(uint16_t Data);
  static void TransmitDMA(uint32_t MemoryIncrease, uint16_t *Data, uint16_t Count);

public:
  static void init();
  static uint32_t getID();
  static bool isBusy();
  static void abort();

  static void dataTransferBegin(uint16_t DataWidth);
  static void dataTransferEnd() { }//WRITE(TFT_CS_PIN, HIGH); };
  static void dataTransferAbort();

  static void writeData(uint16_t Data) {
    if(data_type==DATASIZE_16BIT)
    LCD_WR_DATA(Data);
    else
    LCD_WR_DATA8(Data&0xff);
    }//Transmit(Data); }
  static void writeReg(uint16_t Reg)   { LCD_WR_REG(Reg&0xff);}//WRITE(TFT_A0_PIN, LOW); Transmit(Reg); WRITE(TFT_A0_PIN, HIGH); }

  static void writeSequence(uint16_t *Data, uint16_t Count) {
    uint16_t i;
    for(i=0;i<Count;i++)
    {
      LCD_WR_DATA(Data[i]);
    }
    }// TransmitDMA(DMA_MINC_ENABLE, Data, Count); }

  static void writeMultiple(uint16_t Color, uint32_t Count) {
        uint16_t i;
    for(i=0;i<Count;i++)
    {
      LCD_WR_DATA(Color);
    }
  }
  static void writeSequence_DMA(uint16_t *Data, uint16_t Count) { TransmitDMA(DMA_MINC_ENABLE, Data, Count); }
  static void writeMultiple_DMA(uint16_t Color, uint16_t Count) { static uint16_t Data; Data = Color; TransmitDMA(DMA_MINC_DISABLE, &Data, Count); }


    /*
    static uint16_t Data; Data = Color;
    while (Count > 0) {
      TransmitDMA(DMA_MINC_DISABLE, &Data, Count > 0xFFFF ? 0xFFFF : Count);
      Count = Count > 0xFFFF ? Count - 0xFFFF : 0;
    }
  }
    */
};


