#include "sys.h"


#ifndef _SPI_H_
#define _SPI_H_

//�����Գ���ʹ�õ���Ӳ��SPI�ӿ�����
//����SPIʱ���ź��Լ�SPI����д�ź����Ų��ɸ��ģ��������Ŷ����Ը���
//SPI��ʱ�����Ŷ���̶�ΪPA5
//SPI�Ķ��������Ŷ���̶�ΪPA6
//SPI��д�������Ŷ���̶�ΪPA7
 
u8 SPI_WriteByte(SPI_TypeDef* SPIx,u8 Byte,u8 cmd);
void SPI1_Init(void);
void SPI_SetSpeed(SPI_TypeDef* SPIx,u8 SpeedSet);

#endif

