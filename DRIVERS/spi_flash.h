#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H
#include "stm32f10x.h"
//���������������ݰ��ĸ�ʽ
struct alrm_inf
{
	u8  	info_type;//��Ϣ����
	u8  	pos_chn;//ͨ����
	u32  	pos_dat;//λ������
};
typedef struct alrm_inf alrm_data;//������Ϣ
//w25Q128һ��page256 bytes
#define SPI_FLASH_PageSize      256
#define SPI_FLASH_PerWritePageSize      SPI_FLASH_PageSize

	
void SPI_FLASH_Init(void);
void SPI_FLASH_SectorErase(u32 SectorAddr);
void SPI_FLASH_BulkErase(void);
void SPI_FLASH_PageWrite(u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite);
void SPI_FLASH_BufferWrite(u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite);
void SPI_FLASH_BufferRead(u8* pBuffer, u32 ReadAddr, u16 NumByteToRead);
u32 SPI_FLASH_ReadID(void);
u32 SPI_FLASH_ReadDeviceID(void);
void SPI_FLASH_StartReadSequence(u32 ReadAddr);
void SPI_Flash_PowerDown(void);
void SPI_Flash_WAKEUP(void);


u8 SPI_FLASH_ReadByte(void);
u8 SPI_FLASH_SendByte(u8 byte);
u16 SPI_FLASH_SendHalfWord(u16 HalfWord);
void SPI_FLASH_WriteEnable(void);
void SPI_FLASH_WaitForWriteEnd(void);

u8 DMA1_Spi2_RX(u8 *buffer,u32 len);
u8 DMA1_Spi2_TX(u8 *buffer,u32 len);

#endif /* __SPI_FLASH_H */

