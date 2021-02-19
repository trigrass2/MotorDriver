
/* header of standard C - libraries
---------------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "abs_encoder.h"
#include "string.h"


typedef struct {
  uint8_t     SendReqID;     // ADDR : 0x00 (RD=Send ID, WR=Req. Data Write (Req. ID)
  uint8_t     CD;            // ADDR : 0x01
  uint8_t     DF0;           // ADDR : 0x02
  uint8_t     DF1;           // ADDR : 0x03
  uint8_t     DF2;           // ADDR : 0x04
  uint8_t     DF3;           // ADDR : 0x05
  uint8_t     DF4;           // ADDR : 0x06
  uint8_t     DF5;           // ADDR : 0x07
  uint8_t     DF6;           // ADDR : 0x08
  uint8_t     DF7;           // ADDR : 0x09
  uint8_t     SF;            // ADDR : 0x0A
  uint8_t     encCRC;        // ADDR : 0x0B
  uint8_t     Alram;         // ADDR : 0x0C
  uint8_t     rsv0;             // ADDR : 0x0D
  uint8_t     rsv1;             // ADDR : 0x0E
  uint8_t     rsv2;             // ADDR : 0x0F  
  uint8_t     SSI_Start;        // ADDR : 0x10  ( 1 : SSI clk start, 0 : SSI clk. completed/IDLE)
  uint8_t     SSI_Clock;        // ADDR : 0x11  ( default = 0x14 (1MHz),  clk = 50 nsec x value)
  uint8_t     SSI_Resolution;   // ADDR : 0x12  ( default = 0x18 (24 bit))
  uint8_t     EncoderType;      // ADDR : 0x13  ( default = 0 (SSI Encoder), 1 (Tamagawa)
  uint32_t    SSI_RxData;       // ADDR : 0x14 ~ 0x17
}ENC_REGISTER_T;

volatile ENC_REGISTER_T *pEnc;    
ENC_TYPE _encType = ENC_SSI_POSITAL;
//int16_t _encMultiTurnValue = 0;
//int16_t _encSingleTurnValue = 0;

int8_t _encBusyRQ = 1;
int8_t _encBusyRX = 1;
int8_t _encTimeout = 0;
int8_t _encInt = 1;

STEncDataDesc _abs_encoder;

// TAMAGAWA Abs. Encoder API *******************************************************
UART_HandleTypeDef *_huart_abs_enc = NULL;
uint8_t _abs_enc_dma_buf[11];
uint8_t ctrl_field = 0;

int8_t init_TAMAGAWA_abs_encoder(UART_HandleTypeDef *huart)
{
  _huart_abs_enc = huart;
  _abs_encoder.alarm.byte = 0xFF;
  return reset_TAMAGAWA_abs_encoder();
}

int8_t reset_TAMAGAWA_abs_encoder( )
{
  if(_huart_abs_enc == NULL)
    return -1;

  ctrl_field = 0xBA;
  memset(&_abs_encoder, 0, sizeof(_abs_encoder));
  memset(&_abs_enc_dma_buf, 0, sizeof(_abs_enc_dma_buf[11]));
  
  // resetting ABS. Enc.
  __HAL_UART_FLUSH_DRREGISTER(_huart_abs_enc);
  TAMAGAWA_TX_ENABLE;
  if(HAL_UART_Transmit(_huart_abs_enc, &ctrl_field, 1, 1) != HAL_OK)
  {
	TAMAGAWA_TX_DISABLE;	  
	return -1;
  }
  
  TAMAGAWA_TX_DISABLE;
  HAL_Delay(1);
  return 1;
}

int8_t request_data_TAMAGAWA_abs_encoder( )
{
  if(_huart_abs_enc == NULL)
    return -1;
  
  memset(&_abs_enc_dma_buf, 0, sizeof(_abs_enc_dma_buf[11]));
  ctrl_field = 0x1A;
  
  // start read ABS. Enc. data
  __HAL_UART_FLUSH_DRREGISTER(_huart_abs_enc);
  TAMAGAWA_TX_ENABLE;
  HAL_UART_Transmit_IT(_huart_abs_enc, &ctrl_field, 1);

  return 1;
}

uint8_t get_data_TAMAGAWA_abs_encoder(unsigned long *single, long *multi, long *pulse)
{
  *single = _abs_encoder.singleTurn;
  *multi = _abs_encoder.multiTurn;
  *pulse = _abs_encoder.pulse;
  
  return _abs_encoder.alarm.byte;
}



// FPGA Encoder API *******************************************************

uint8_t init_Encoder(ENC_TYPE type)
{   
    // set start address of ENCODER 
  pEnc = (ENC_REGISTER_T *)(__IO uint8_t *)(SRAM_BANK1_NE4_ADDR);
  _encType = type;
  reset_Encoder();
  HAL_Delay(10);
  if(_encType == ENC_SSI_POSITAL)
  {
    pEnc->EncoderType = 0;
    set_EncoderSSI(40, 24);     // clock: 2 MHz, resolution: 24 bit
  }
  else if(_encType == ENC_TAMAGAWA)
  {
    pEnc->EncoderType = 1;
  }
  return(0);
}

uint8_t reset_Encoder(void)
{   
    GPIOD->BSRR = (uint32_t)GPIO_PIN_11 << 16U;
    HAL_Delay(10);
    GPIOD->BSRR = (uint32_t)GPIO_PIN_11;
    return(0);
}

uint8_t set_EncoderSSI(uint8_t clock, uint8_t resolution)
{   
    pEnc->SSI_Clock = clock;            // clock x 50 nsec
    pEnc->SSI_Resolution = resolution;  // resolution bit
    return(0);
}

    int8_t abs[3];
    int8_t abm[3];
    int8_t enID=0;
    int8_t alarm = 0;
    
int32_t read_Encoder(void)
{   
  int32_t readValue = 0;
  
  _encBusyRQ = DEF_ENC_RQ_BUSY;
  _encBusyRX = DEF_ENC_RX_BUSY;
  _encTimeout = DEF_ENC_TIMEOUT;  
  _encInt = DEF_ENC_INT;
    
  if(_encType == ENC_SSI_POSITAL)
  {
    pEnc->SSI_Start = 1;
    while(pEnc->SSI_Start == 1)
    {
      if(pEnc->SSI_Start == 0)
        break;
      if(DEF_ENC_TIMEOUT == 1)
        return 0;
    }
    readValue = (pEnc->SSI_RxData & 0x00FFFFFF);
    _encMultiTurnValue = readValue/4096;
    _encSingleTurnValue = readValue%4096;
    return readValue;
  }
  else if(_encType == ENC_TAMAGAWA)
  {
    pEnc->EncoderType = 1;
    HAL_Delay(10);
    //uint8_t id = pEnc->SendReqID;
    if((_encBusyRX == GPIO_PIN_SET) && (_encInt == GPIO_PIN_RESET))
    {
      abs[0] = pEnc->DF0;
      abs[1] = pEnc->DF1;
      abs[2] = pEnc->DF2;
      enID = pEnc->DF3;
      abm[0] = pEnc->DF4;
      abm[1] = pEnc->DF5;
      abm[2] = pEnc->DF6;
      alarm = pEnc->DF7;
    }   

    pEnc->EncoderType = 1;
    if((_encBusyRQ == GPIO_PIN_SET) && (_encBusyRX == GPIO_PIN_SET) && ( _encInt == GPIO_PIN_SET))
    {
      pEnc->SendReqID = 0x03;
    }
    HAL_Delay(10);
    if((_encBusyRX == GPIO_PIN_SET) && (_encInt == GPIO_PIN_RESET))
    {
      abs[0] = pEnc->DF0;
      abs[1] = pEnc->DF1;
      abs[2] = pEnc->DF2;
      enID = pEnc->DF3;
      abm[0] = pEnc->DF4;
      abm[1] = pEnc->DF5;
      abm[2] = pEnc->DF6;
      alarm = pEnc->DF7;
    }

  }      
  return(0);
}



void SmartAbsEncSendCFData(unsigned char bCode)
{
/*
	int iTimeOut = 20000;

	ScibRegs.SCIFFTX.bit.TXFFINTCLR=1;	// Clear SCI Interrupt flag
	// using SCIB port
//	if(ScibRegs.SCIFFTX.bit.TXFFIENA != 1)
//	{
	ScibRegs.SCIFFRX.bit.RXFIFORESET = 0;
	GpioDataRegs.GPASET.bit.GPIO21 = 1;
//		ScibRegs.SCIFFTX.bit.TXFFIENA = 1;
	ScibRegs.SCITXBUF = bCode;
	m_sAbsEnc.bCF = bCode;
	ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;
//	}
//	else
//	{
//		ScibRegs.SCIFFTX.bit.TXFFIENA = 0;
//	}

	while(1)
	{
		if(ScibRegs.SCIFFTX.bit.TXFFINT == 1)
			break;
		iTimeOut--;
		if(iTimeOut < 0)
			return;
	}
	delay_us_isr(3);
	GpioDataRegs.GPACLEAR.bit.GPIO21 = 1;
*/
}


void ReadSmartAbsEncSingleTurnData(unsigned char *pData, unsigned long *plSingleCnt)
{
  /*
	UData32 uTmp;

	m_sAbsEnc.sStatus.byte = *(pData+1);
	uTmp.bData.b0 =  *(pData+2);
	uTmp.bData.b1 =  *(pData+3);
	uTmp.bData.b2 =  *(pData+4);
	uTmp.bData.b3 = 0x00;
	m_sAbsEnc.lSingleTurn = uTmp.lData;
	*plSingleCnt = m_sAbsEnc.lSingleTurn;
  */
}


void ReadSmartAbsEncData(unsigned char *pData, unsigned long *plSingleCnt, unsigned long *plMultiCnt)
{
  /*
	UData32 uTmp;

//	ReadSmartAbsEncSingleTurnData(pData, piSingleCnt);
	m_sAbsEnc.sStatus.byte = *(pData+1);
	uTmp.bData.b0 =  *(pData+2);
	uTmp.bData.b1 =  *(pData+3);
	uTmp.bData.b2 =  *(pData+4);
	uTmp.bData.b3 = 0x00;
	m_sAbsEnc.lSingleTurn = uTmp.lData;
	*plSingleCnt = m_sAbsEnc.lSingleTurn;

	m_sAbsEnc.bEncID = *(pData+5);

	uTmp.bData.b0 =  *(pData+6);
	uTmp.bData.b1 =  *(pData+7);
	uTmp.bData.b2 =  *(pData+8);
	uTmp.bData.b3 = 0x00;
	m_sAbsEnc.lMultiTurn = uTmp.lData;
	*plMultiCnt = m_sAbsEnc.lMultiTurn;
	
	m_sAbsEnc.sAlarm.byte = *(pData+9);
  */
}
/*
#ifdef FLASH_MODE
	#pragma CODE_SECTION(SmartAbsEncCRC8, "ramfuncs");
#endif
unsigned char SmartAbsEncCRC8( unsigned char *source , int number) 
{   
   int i, j ;   
   unsigned char crc, ch;   
   crc = 0;
  
   for ( i = 0 ; i < number ; i++ ) {   
      ch = *(source + i);   
  
      for ( j = 0 ; j < 8 ; j++ )    
      {   
          if ( crc & 0x80 )    
          {   
              crc<<=1;   
              if ( ch & 0x80 )    
              {   
                   crc = crc | 0x01;   
              }    
              else    
              {   
                   crc =crc & 0xfe;   
              }    
              crc = crc ^ 0x107;   
          }    
          else    
          {   
              crc<<=1;   
              if ( ch & 0x80 )    
              {   
                   crc = crc | 0x01;   
              }    
              else    
              {   
                   crc = crc & 0xfe;   
              }   
          }   
          ch<<=1;   
      }   
   }    
  
   ch = 0;   
   for ( j = 0 ; j < 8 ; j++ )    
   {   
      if ( crc & 0x80 )    
      {   
          crc<<=1;   
          if ( ch & 0x80 )    
          {   
             crc = crc | 0x01;   
          }    
          else    
          {   
             crc =crc & 0xfe;   
          }    
          crc = crc ^ 0x107;   
      }    
      else    
      {   
          crc<<=1;   
          if ( ch & 0x80 )    
          {   
             crc = crc | 0x01;   
          }    
          else    
          {   
             crc = crc & 0xfe;   
          }   
       }   
       ch<<=1;   
    }   

    return crc; 
}
*/

/*______________________________________________________________________EOF_*/
