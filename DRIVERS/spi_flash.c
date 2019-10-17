/**********************************************************************************
 * 文件名  ：spi_flash.c
 * 描述    ：spi 底层应用函数库         

 * 硬件连接 ----------------------------
 *         | PA4-SPI1-NSS  : W25X128-CS  |
 *         | PA5-SPI1-SCK  : W25X128-CLK |
 *         | PA6-SPI1-MISO : W25X128-DO  |
 *         | PA7-SPI1-MOSI : W25X128-DIO |
 *          ----------------------------

**********************************************************************************/
#include "spi_flash.h"



/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
#define W25X_WriteEnable		      0x06 
#define W25X_WriteDisable		      0x04 
#define W25X_ReadStatusReg		    0x05 
#define W25X_WriteStatusReg		    0x01 
#define W25X_ReadData			        0x03 
#define W25X_FastReadData		      0x0B 
#define W25X_FastReadDual		      0x3B 
#define W25X_PageProgram		      0x02 
#define W25X_BlockErase			      0xD8 
#define W25X_SectorErase		      0x20 
#define W25X_ChipErase			      0xC7 
#define W25X_PowerDown			      0xB9 
#define W25X_ReleasePowerDown	    0xAB 
#define W25X_DeviceID			        0xAB 
#define W25X_ManufactDeviceID   	0x90 
#define W25X_JedecDeviceID		    0x9F 

#define WIP_Flag                  0x01  /* Write In Progress (WIP) flag */

#define Dummy_Byte                0xFF


#define SPI2_MOSI_PIN      	GPIO_Pin_15
#define SPI2_MISO_PIN      	GPIO_Pin_14
#define SPI2_SCLK_PIN     	GPIO_Pin_13
#define SPI2_NSS_PIN      	GPIO_Pin_12

#define FLASH_nRST     	GPIO_Pin_9


#define SPI2_GPIO       						GPIOB
#define SPI2_GPIO_PeriphClock       RCC_APB2Periph_GPIOB
#define SPI2_PeriphClock						RCC_APB1Periph_SPI2

#define Flash_SPI SPI2
#define Flash_SPI_GPIO       						GPIOB
#define Flash_SPI_GPIO_PeriphClock       RCC_APB2Periph_GPIOB
#define Flash_SPI_PeriphClock						RCC_APB1Periph_SPI2

#define SPI_FLASH_CS_LOW()        GPIO_ResetBits(Flash_SPI_GPIO, SPI2_NSS_PIN)
#define SPI_FLASH_CS_HIGH()      GPIO_SetBits(Flash_SPI_GPIO, SPI2_NSS_PIN)
///################################################################################
//* Function Name  : SPI_FLASH_Init
//* Description    : Initializes the peripherals used by the SPI FLASH driver.
//* Input          : None
//* Output         : None
//* Return         : None
//################################################################################
u8 DMA1_Spi2_Init(void);
void SPI_FLASH_Init(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  // Enable SPI1 and GPIO clocks 
	//SPI_FLASH_SPI_CS_GPIO, SPI_FLASH_SPI_MOSI_GPIO, 
	//SPI_FLASH_SPI_MISO_GPIO, SPI_FLASH_SPI_DETECT_GPIO 
	//and SPI_FLASH_SPI_SCK_GPIO Periph clock enable
  RCC_APB2PeriphClockCmd(Flash_SPI_GPIO_PeriphClock, ENABLE);
  RCC_APB1PeriphClockCmd(Flash_SPI_PeriphClock, ENABLE);//SPI_FLASH_SPI Periph clock enable 
  // Configure SPI_FLASH_SPI pins: SCK 
  GPIO_InitStructure.GPIO_Pin = SPI2_SCLK_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(Flash_SPI_GPIO, &GPIO_InitStructure);
  // Configure SPI_FLASH_SPI pins: MISO
  GPIO_InitStructure.GPIO_Pin = SPI2_MISO_PIN;
  GPIO_Init(Flash_SPI_GPIO, &GPIO_InitStructure);
  //Configure SPI_FLASH_SPI pins: MOSI
  GPIO_InitStructure.GPIO_Pin = SPI2_MOSI_PIN;
  GPIO_Init(Flash_SPI_GPIO, &GPIO_InitStructure);
  //Configure SPI_FLASH_SPI_CS_PIN pin: SPI_FLASH Card CS pin
  GPIO_InitStructure.GPIO_Pin = SPI2_NSS_PIN|FLASH_nRST;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_InitStructure.GPIO_Pin = ;//PB9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(Flash_SPI_GPIO, &GPIO_InitStructure);
	GPIO_SetBits(Flash_SPI_GPIO, FLASH_nRST);//set high
  // Deselect the FLASH: Chip Select high
  SPI_FLASH_CS_HIGH();
  // SPI1 configuration 
  // W25X16: data input on the DIO pin is sampled on the rising edge of the CLK. 
  // Data on the DO and DIO pins are clocked out on the falling edge of CLK.
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;//SPI_NSS_Hard;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(Flash_SPI, &SPI_InitStructure);
	//SPI_NSSInternalSoftwareConfig(Flash_SPI,SPI_NSSInternalSoft_Set);
	SPI_SSOutputCmd(Flash_SPI, ENABLE);
	DMA1_Spi2_Init();
  SPI_Cmd(Flash_SPI, ENABLE);			//Enable SPI1
}
u8 DMA1_Spi2_Init(void)
{
	 DMA_InitTypeDef DMA1_Init;
	
		NVIC_InitTypeDef NVIC_InitSPI2TX,NVIC_InitSPI2RX;
	/* Enable the USART1 Interrupt */
		NVIC_InitSPI2TX.NVIC_IRQChannel = DMA1_Channel5_IRQn;
		NVIC_InitSPI2TX.NVIC_IRQChannelPreemptionPriority = 2;
		NVIC_InitSPI2TX.NVIC_IRQChannelSubPriority = 2;
		NVIC_InitSPI2TX.NVIC_IRQChannelCmd = ENABLE;
	//	NVIC_Init(&NVIC_InitSPI2TX);

		NVIC_InitSPI2RX.NVIC_IRQChannel = DMA1_Channel4_IRQn;
		NVIC_InitSPI2RX.NVIC_IRQChannelPreemptionPriority = 2;
		NVIC_InitSPI2RX.NVIC_IRQChannelSubPriority = 2;
		NVIC_InitSPI2RX.NVIC_IRQChannelCmd = ENABLE;
	//	NVIC_Init(&NVIC_InitSPI2RX);
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
		//DMA_SPI_RX  SPI->RAM的数据传输
		DMA_DeInit(DMA1_Channel4);	
		DMA1_Init.DMA_PeripheralBaseAddr=(u32)&SPI2->DR;//启动传输前装入实际RAM地址
		DMA1_Init.DMA_DIR=DMA_DIR_PeripheralSRC;
		DMA1_Init.DMA_BufferSize=0;
		DMA1_Init.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
		DMA1_Init.DMA_MemoryInc=DMA_MemoryInc_Enable;
		DMA1_Init.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;
		DMA1_Init.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;
		DMA1_Init.DMA_Mode=DMA_Mode_Normal;
		DMA1_Init.DMA_Priority=DMA_Priority_High; 
		DMA1_Init.DMA_M2M=DMA_M2M_Disable;
		DMA_Init(DMA1_Channel4,&DMA1_Init); //对DMA通道4进行初始化
		//DMA_SPI_TX  RAM->SPI的数据传输
		DMA_DeInit(DMA1_Channel5);
		DMA1_Init.DMA_PeripheralBaseAddr=(u32)&SPI2->DR;//启动传输前装入实际RAM地址
		DMA1_Init.DMA_DIR=DMA_DIR_PeripheralDST; 
		DMA1_Init.DMA_BufferSize=0;
		DMA1_Init.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
		DMA1_Init.DMA_MemoryInc=DMA_MemoryInc_Enable;
		DMA1_Init.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;
		DMA1_Init.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;
		DMA1_Init.DMA_Mode=DMA_Mode_Normal;
		DMA1_Init.DMA_Priority=DMA_Priority_Low; 
		DMA1_Init.DMA_M2M=DMA_M2M_Disable;
		DMA_Init(DMA1_Channel5,&DMA1_Init); //对DMA通道5进行初始化
		SPI_I2S_DMACmd(SPI2,SPI_I2S_DMAReq_Rx,ENABLE); //使能SPI的DMA接收请求
		SPI_I2S_DMACmd(SPI2,SPI_I2S_DMAReq_Tx,ENABLE); //使能SPI的DMA发送请求
	 return 0;
}
//从sd卡读取一个数据包的内容时候启动DMA传输
//buf:数据缓存区
//len:要读取的数据长度.
u8 DMA1_Spi2_RX(u8 *buffer,u32 len)
{
    u8 temp=0xff;
    	
		DMA1_Channel4->CNDTR=len;                 //设置传输的数据长度
		DMA1_Channel4->CMAR=(uint32_t)buffer;     //设置内存缓冲区地址
		
		/*SPI作为主机进行数据接收时必须要主动产生时钟，因此此处必须有DMA通道5的配合*/
		DMA1_Channel5->CNDTR=len; 
		DMA1_Channel5->CMAR=(uint32_t)&temp;      //temp=0xff
		DMA1_Channel5->CCR&=~DMA_MemoryInc_Enable;//内存地址非自增
		
		DMA_Cmd(DMA1_Channel4,ENABLE);            //首先启动DMA通道4
		DMA_Cmd(DMA1_Channel5,ENABLE);            //再启动DMA通道5
		while(!DMA_GetFlagStatus(DMA1_FLAG_TC4)); //等待DMA通道4接收数据完成
		DMA_ClearFlag(DMA1_FLAG_TC4); 
		DMA_ClearFlag(DMA1_FLAG_TC5);             //清除DMA通道4与5的传输完成标志
		DMA_Cmd(DMA1_Channel4,DISABLE);
		DMA_Cmd(DMA1_Channel5,DISABLE);           //使DMA通道4与5停止工作
		
		DMA1_Channel5->CCR|=DMA_MemoryInc_Enable; //将DMA通道5恢复为内存地址自增方式
	  return 0;
}

//buf:数据缓存区
u8 DMA1_Spi2_TX(u8 *buffer,u32 len)
{
		DMA1_Channel5->CNDTR=len;                 	//设置要传输的数据长度
		DMA1_Channel5->CMAR=(uint32_t)buffer;     	//设置RAM缓冲区地址
		DMA_Cmd(DMA1_Channel5,ENABLE);            	//启动DMA传输 RAM->SPI
		while(!DMA_GetFlagStatus(DMA1_FLAG_TC5)); //等待DMA通道5传输完成
		DMA_ClearFlag(DMA1_FLAG_TC5);             //清除通道5传输完成状态标记
		DMA_Cmd(DMA1_Channel5,DISABLE);           //使DMA通道5停止工作
	  return 0;
}
void DMA1_Channel5_IRQHandler(void)//SPI2/I2S2_TX
{

}
void DMA1_Channel4_IRQHandler(void)//SPI2/I2S2_RX
{

}
/*******************************************************************************
* Function Name  : SPI_FLASH_SectorErase
* Description    : Erases the specified FLASH sector.
* Input          : SectorAddr: address of the sector to erase.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_SectorErase(u32 SectorAddr)
{
  /* Send write enable instruction */
  SPI_FLASH_WriteEnable();
  SPI_FLASH_WaitForWriteEnd();
  /* Sector Erase */
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();
  /* Send Sector Erase instruction */
  SPI_FLASH_SendByte(W25X_SectorErase);
  /* Send SectorAddr high nibble address byte */
  SPI_FLASH_SendByte((SectorAddr & 0xFF0000) >> 16);
  /* Send SectorAddr medium nibble address byte */
  SPI_FLASH_SendByte((SectorAddr & 0xFF00) >> 8);
  /* Send SectorAddr low nibble address byte */
  SPI_FLASH_SendByte(SectorAddr & 0xFF);
  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();
  /* Wait the end of Flash writing */
  SPI_FLASH_WaitForWriteEnd();
}

/*******************************************************************************
* Function Name  : SPI_FLASH_BulkErase
* Description    : Erases the entire FLASH.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_BulkErase(void)
{
  /* Send write enable instruction */
  SPI_FLASH_WriteEnable();
  /* Bulk Erase */
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();
  /* Send Bulk Erase instruction  */
  SPI_FLASH_SendByte(W25X_ChipErase);
  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();

  /* Wait the end of Flash writing */
  SPI_FLASH_WaitForWriteEnd();
}

/*******************************************************************************
* Function Name  : SPI_FLASH_PageWrite
* Description    : Writes more than one byte to the FLASH with a single WRITE
*                  cycle(Page WRITE sequence). The number of byte can't exceed
*                  the FLASH page size.
* Input          : - pBuffer : pointer to the buffer  containing the data to be
*                    written to the FLASH.
*                  - WriteAddr : FLASH's internal address to write to.
*                  - NumByteToWrite : number of bytes to write to the FLASH,
*                    must be equal or less than "SPI_FLASH_PageSize" value.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_PageWrite(u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite)
{
  /* Enable the write access to the FLASH */
  SPI_FLASH_WriteEnable();

  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();
  /* Send "Write to Memory " instruction */
  SPI_FLASH_SendByte(W25X_PageProgram);
  /* Send WriteAddr high nibble address byte to write to */
  SPI_FLASH_SendByte((WriteAddr & 0xFF0000) >> 16);
  /* Send WriteAddr medium nibble address byte to write to */
  SPI_FLASH_SendByte((WriteAddr & 0xFF00) >> 8);
  /* Send WriteAddr low nibble address byte to write to */
  SPI_FLASH_SendByte(WriteAddr & 0xFF);

  if(NumByteToWrite > SPI_FLASH_PerWritePageSize)
  {
     NumByteToWrite = SPI_FLASH_PerWritePageSize;
     //printf("\n\r Err: SPI_FLASH_PageWrite too large!");
  }

  /* while there is data to be written on the FLASH */
  while (NumByteToWrite--)
  {
    /* Send the current byte */
    SPI_FLASH_SendByte(*pBuffer);
    /* Point on the next byte to be written */
    pBuffer++;
  }

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();

  /* Wait the end of Flash writing */
  SPI_FLASH_WaitForWriteEnd();
}

/*******************************************************************************
* Function Name  : SPI_FLASH_BufferWrite
* Description    : Writes block of data to the FLASH. In this function, the
*                  number of WRITE cycles are reduced, using Page WRITE sequence.
* Input          : - pBuffer : pointer to the buffer  containing the data to be
*                    written to the FLASH.
*                  - WriteAddr : FLASH's internal address to write to.
*                  - NumByteToWrite : number of bytes to write to the FLASH.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_BufferWrite(u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite)
{
  u8 NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

  Addr = WriteAddr % SPI_FLASH_PageSize;
  count = SPI_FLASH_PageSize - Addr;
  NumOfPage =  NumByteToWrite / SPI_FLASH_PageSize;
  NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;

  if (Addr == 0) /* WriteAddr is SPI_FLASH_PageSize aligned  */
  {
    if (NumOfPage == 0) /* NumByteToWrite < SPI_FLASH_PageSize */
    {
      SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumByteToWrite);
    }
    else /* NumByteToWrite > SPI_FLASH_PageSize */
    {
      while (NumOfPage--)
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, SPI_FLASH_PageSize);
        WriteAddr +=  SPI_FLASH_PageSize;
        pBuffer += SPI_FLASH_PageSize;
      }
      SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumOfSingle);
    }
  }
  else /* WriteAddr is not SPI_FLASH_PageSize aligned  */
  {
    if (NumOfPage == 0) /* NumByteToWrite < SPI_FLASH_PageSize */
    {
      if (NumOfSingle > count) /* (NumByteToWrite + WriteAddr) > SPI_FLASH_PageSize */
      {
        temp = NumOfSingle - count;

        SPI_FLASH_PageWrite(pBuffer, WriteAddr, count);
        WriteAddr +=  count;
        pBuffer += count;

        SPI_FLASH_PageWrite(pBuffer, WriteAddr, temp);
      }
      else
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumByteToWrite);
      }
    }
    else /* NumByteToWrite > SPI_FLASH_PageSize */
    {
      NumByteToWrite -= count;
      NumOfPage =  NumByteToWrite / SPI_FLASH_PageSize;
      NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;

      SPI_FLASH_PageWrite(pBuffer, WriteAddr, count);
      WriteAddr +=  count;
      pBuffer += count;

      while (NumOfPage--)
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, SPI_FLASH_PageSize);
        WriteAddr +=  SPI_FLASH_PageSize;
        pBuffer += SPI_FLASH_PageSize;
      }

      if (NumOfSingle != 0)
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumOfSingle);
      }
    }
  }
}

/*******************************************************************************
* Function Name  : SPI_FLASH_BufferRead
* Description    : Reads a block of data from the FLASH.
* Input          : - pBuffer : pointer to the buffer that receives the data read
*                    from the FLASH.
*                  - ReadAddr : FLASH's internal address to read from.
*                  - NumByteToRead : number of bytes to read from the FLASH.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_BufferRead(u8* pBuffer, u32 ReadAddr, u16 NumByteToRead)
{
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();
  /* Send "Read from Memory " instruction */
  SPI_FLASH_SendByte(W25X_ReadData);
  /* Send ReadAddr high nibble address byte to read from */
  SPI_FLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
  /* Send ReadAddr medium nibble address byte to read from */
  SPI_FLASH_SendByte((ReadAddr& 0xFF00) >> 8);
  /* Send ReadAddr low nibble address byte to read from */
  SPI_FLASH_SendByte(ReadAddr & 0xFF);
//  while (NumByteToRead--) /* while there is data to be read */
//  {
//    /* Read a byte from the FLASH */
//    *pBuffer = SPI_FLASH_SendByte(Dummy_Byte);
//    /* Point to the next location where the byte read will be saved */
//    pBuffer++;
//  }
		DMA1_Spi2_RX(pBuffer,NumByteToRead);
  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();
}

/*******************************************************************************
* Function Name  : SPI_FLASH_ReadID
* Description    : Reads FLASH identification.
* Input          : None
* Output         : None
* Return         : FLASH identification
*******************************************************************************/
u32 SPI_FLASH_ReadID(void)
{
  u32 Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;

  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /* Send "RDID " instruction */
  SPI_FLASH_SendByte(W25X_JedecDeviceID);

  /* Read a byte from the FLASH */
  Temp0 = SPI_FLASH_SendByte(Dummy_Byte);

  /* Read a byte from the FLASH */
  Temp1 = SPI_FLASH_SendByte(Dummy_Byte);

  /* Read a byte from the FLASH */
  Temp2 = SPI_FLASH_SendByte(Dummy_Byte);

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();

  Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;

  return Temp;
}
/*******************************************************************************
* Function Name  : SPI_FLASH_ReadID
* Description    : Reads FLASH identification.
* Input          : None
* Output         : None
* Return         : FLASH identification
*******************************************************************************/
u32 SPI_FLASH_ReadDeviceID(void)
{
  u32 Temp = 0;

  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /* Send "RDID " instruction */
  SPI_FLASH_SendByte(W25X_DeviceID);
  SPI_FLASH_SendByte(Dummy_Byte);
  SPI_FLASH_SendByte(Dummy_Byte);
  SPI_FLASH_SendByte(Dummy_Byte);
  
  /* Read a byte from the FLASH */
  Temp = SPI_FLASH_SendByte(Dummy_Byte);

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();

  return Temp;
}
/*******************************************************************************
* Function Name  : SPI_FLASH_StartReadSequence
* Description    : Initiates a read data byte (READ) sequence from the Flash.
*                  This is done by driving the /CS line low to select the device,
*                  then the READ instruction is transmitted followed by 3 bytes
*                  address. This function exit and keep the /CS line low, so the
*                  Flash still being selected. With this technique the whole
*                  content of the Flash is read with a single READ instruction.
* Input          : - ReadAddr : FLASH's internal address to read from.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_StartReadSequence(u32 ReadAddr)
{
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /* Send "Read from Memory " instruction */
  SPI_FLASH_SendByte(W25X_ReadData);

  /* Send the 24-bit address of the address to read from -----------------------*/
  /* Send ReadAddr high nibble address byte */
  SPI_FLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
  /* Send ReadAddr medium nibble address byte */
  SPI_FLASH_SendByte((ReadAddr& 0xFF00) >> 8);
  /* Send ReadAddr low nibble address byte */
  SPI_FLASH_SendByte(ReadAddr & 0xFF);
}

/*******************************************************************************
* Function Name  : SPI_FLASH_ReadByte
* Description    : Reads a byte from the SPI Flash.
*                  This function must be used only if the Start_Read_Sequence
*                  function has been previously called.
* Input          : None
* Output         : None
* Return         : Byte Read from the SPI Flash.
*******************************************************************************/
u8 SPI_FLASH_ReadByte(void)
{
  return (SPI_FLASH_SendByte(Dummy_Byte));
}

/*******************************************************************************
* Function Name  : SPI_FLASH_SendByte
* Description    : Sends a byte through the SPI interface and return the byte
*                  received from the SPI bus.
* Input          : byte : byte to send.
* Output         : None
* Return         : The value of the received byte.
*******************************************************************************/
u8 SPI_FLASH_SendByte(u8 byte)
{
  //Loop while DR register in not emplty 
  while (SPI_I2S_GetFlagStatus(Flash_SPI, SPI_I2S_FLAG_TXE)==RESET);
  // Send byte through the SPI peripheral 
  SPI_I2S_SendData(Flash_SPI, byte);
  //Wait to receive a byte 
  while (SPI_I2S_GetFlagStatus(Flash_SPI, SPI_I2S_FLAG_RXNE)==RESET);
  // Return the byte read from the SPI bus 
  return SPI_I2S_ReceiveData(Flash_SPI);
}

/*******************************************************************************
* Function Name  : SPI_FLASH_SendHalfWord
* Description    : Sends a Half Word through the SPI interface and return the
*                  Half Word received from the SPI bus.
* Input          : Half Word : Half Word to send.
* Output         : None
* Return         : The value of the received Half Word.
*******************************************************************************/
u16 SPI_FLASH_SendHalfWord(u16 HalfWord)
{
  // Loop while DR register in not emplty
  while (SPI_I2S_GetFlagStatus(Flash_SPI, SPI_I2S_FLAG_TXE)==RESET);
  // Send Half Word through the SPI peripheral 
  SPI_I2S_SendData(Flash_SPI, HalfWord);  // Wait to receive a Half Word
  while (SPI_I2S_GetFlagStatus(Flash_SPI, SPI_I2S_FLAG_RXNE)==RESET);
  // Return the Half Word read from the SPI bus
  return SPI_I2S_ReceiveData(Flash_SPI);
}

/*******************************************************************************
* Function Name  : SPI_FLASH_WriteEnable
* Description    : Enables the write access to the FLASH.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_WriteEnable(void)
{
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();
  /* Send "Write Enable" instruction */
  SPI_FLASH_SendByte(W25X_WriteEnable);
  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();
}
/*******************************************************************************
* Function Name  : SPI_FLASH_WaitForWriteEnd
* Description    : Polls the status of the Write In Progress (WIP) flag in the
*                  FLASH's status  register  and  loop  until write  opertaion
*                  has completed.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_WaitForWriteEnd(void)
{
  u8 FLASH_Status = 0;

  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /* Send "Read Status Register" instruction */
  SPI_FLASH_SendByte(W25X_ReadStatusReg);

  /* Loop as long as the memory is busy with a write cycle */
  do
  {
    /* Send a dummy byte to generate the clock needed by the FLASH
    and put the value of the status register in FLASH_Status variable */
    FLASH_Status = SPI_FLASH_SendByte(Dummy_Byte);	 
  }
  while ((FLASH_Status & WIP_Flag) == SET); /* Write in progress */

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();
}


//进入掉电模式
void SPI_Flash_PowerDown(void)   
{ 
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /* Send "Power Down" instruction */
  SPI_FLASH_SendByte(W25X_PowerDown);

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();
}   

//唤醒
void SPI_Flash_WAKEUP(void)   
{
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /* Send "Power Down" instruction */
  SPI_FLASH_SendByte(W25X_ReleasePowerDown);

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();                   //等待TRES1
}   



/******************************END OF FILE*****************************/
