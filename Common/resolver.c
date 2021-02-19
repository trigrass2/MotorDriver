
/* header of standard C - libraries
---------------------------------------------------------------------------*/
#include <string.h>
#include "stm32f4xx_hal.h"

#include "math.h"
#include "resolver.h"  
#include "binary_values_bit.h"
#include "spi.h"

#define BYTE_H(x)  ((x >> 8) & 0x00FF)
#define BYTE_L(x)  (x & 0x00FF)

uint8_t AMC1210_DATA_READ = 1;
uint8_t AMC1210_DATA_WRITE = 0;
uint8_t AMC1210_ISR_REG_ADDR = 0x00;
uint8_t AMC1210_CTRL_PARAM_REG_ADDR[4] = {0x01, 0x07, 0x0D, 0x13};
uint8_t AMC1210_SINC_FILTER_REG_ADDR[4] = {0x02, 0x08, 0x0E, 0x14};
uint8_t AMC1210_INTEGRATOR_PARAM_REG_ADDR[4] = {0x03, 0x09, 0x0F, 0x15};
uint8_t AMC1210_HIGH_LEVEL_THRESHOLD_REG_ADDR[4] = {0x04, 0x0A, 0x10, 0x16};
uint8_t AMC1210_LOW_LEVEL_THRESHOLD_REG_ADDR[4] = {0x05, 0x0B, 0x11, 0x17};
uint8_t AMC1210_COMP_FILTER_PARAM_REG_ADDR[4] = {0x06, 0x0C, 0x12, 0x18};
uint8_t AMC1210_CTRL_REG_ADDR = 0x19;
uint8_t AMC1210_PATTERN_REG_ADDR = 0x1A;
uint8_t AMC1210_CLK_DIV_REG_ADDR = 0x1B;
uint8_t AMC1210_STATUS_REG_ADDR = 0x1C;
uint8_t AMC1210_DATA_REG_ADDR[4] = {0x1D, 0x1F, 0x21, 0x23};
uint8_t AMC1210_TIME_REG_ADDR[4] = {0x1E, 0x20, 0x22, 0x24};

/*
// -a 0.9 -T 1024
uint16_t AMC1210_SIG_PATTERN[64] = {
  0xCCAA, 0x64C9, 0x8C52, 0x9186, 0x1485, 0x1121, 0x2110, 0x8812, 0x0220, 0x1080,
  0x2080, 0x0804, 0x0008, 0x0040, 0x0004, 0x0001, 0x0000, 0x0800, 0x0400, 0x1000,
  0x8010, 0x0804, 0x0808, 0x4044, 0x0902, 0x4221, 0x2228, 0x30A2, 0x8A49, 0x464A,
  0x94B1, 0xA565, 0x5967, 0x2D5A, 0xD6B5, 0xD6DB, 0x6EBB, 0xB776, 0xF77B, 0xBEEF,
  0xDDFD, 0xEFF7, 0xEFFB, 0xFBFF, 0xF7FD, 0xFFFF, 0xBFFF, 0xBFFF, 0xFEFF, 0xFFBF,
  0xFFDF, 0xFF7F, 0xEFFD, 0xFEFF, 0x7DFD, 0xF7DE, 0xFBDD, 0xF5F9, 0xF9F3, 0xE7AE,
  0xBADB, 0x5AD9, 0xCD66, 0xACCB  
};
*/
// -a 0.9 -T 1000 (0x03E7 + 1)
uint16_t AMC1210_SIG_PATTERN[64] = {
  0x0000, 0x00CC, 0xAA58, 0xC952, 0x5251, 0x48A2, 0x4488, 0x9090, 0x4814, 0x0240,
  0x4202, 0x0201, 0x0080, 0x0800, 0x2000, 0x8000, 0x0800, 0x0100, 0x0010, 0x0020,
  0x0020, 0x0801, 0x0080, 0x8101, 0x0820, 0x8422, 0x0910, 0x9122, 0x2894, 0x3149,
  0x498C, 0x6633, 0x2AAA, 0xB335, 0xAAD9, 0xB9CF, 0x3AED, 0xB775, 0xEEDE, 0xEF77,
  0xDDFD, 0xDFBF, 0xBEFF, 0xBFDF, 0xFBFF, 0xBFFF, 0xDFFF, 0xDFFF, 0xFEFF, 0xFFDF,
  0xFFF7, 0xFF7F, 0xFF7E, 0xFFEF, 0xBFEF, 0x7EFB, 0xEEFB, 0xDBED, 0xEDDD, 0xB76D,
  0xB9E6, 0xE739, 0xCD66, 0xACCB
};


SPI_HandleTypeDef *_hspi = NULL;


uint32_t _motor_control_loop_freq = 8000; // 8kHz, carrier freq.
uint32_t _input_clock = 24000000; // 24 MHz

// uint8_t _motor_control_loop_freq = 20000; // 20kHz, carrier freq.
// uint8_t _input_clock = 24000000; // 24 MHz
// CDiv = f_clk / (f_carrier * N_PAT) = 24000000 / (20000 * 1000)

AMC1210_RSV_VAR _resolver_var;
  
int8_t resolver_config_spi_write_data(uint8_t addr, uint8_t dataH, uint8_t dataL)
{
  AMC1210_WRITE_DATA spi_write;
  AMC1210_WRITE_DATA spi_read;    
  memset(&spi_write.byte, 0, sizeof(spi_write.value));
  memset(&spi_read.byte, 0, sizeof(spi_read.value));
  
  spi_write.value.RW = AMC1210_DATA_WRITE;
  spi_write.value.address = addr;
  spi_write.value.byteH = dataH;
  spi_write.value.byteL = dataL;
    
  RSLVR_SPI_CS_ENABLE;
  if(HAL_SPI_TransmitReceive(_hspi, spi_write.byte, spi_read.byte, 3, 100) != HAL_OK)
  {
    RSLVR_SPI_CS_DISABLE;
    return -1;
  }
  RSLVR_SPI_CS_DISABLE;  
  return 1;
}

int8_t resolver_config_spi_read_data(uint8_t addr, uint8_t *data)
{
  AMC1210_WRITE_DATA spi_write;
  AMC1210_WRITE_DATA spi_read;    
  memset(&spi_write.byte, 0, sizeof(spi_write.value));
  memset(&spi_read.byte, 0, sizeof(spi_read.value));
  
  spi_write.value.RW = AMC1210_DATA_READ;
  spi_write.value.address = addr;
  
  RSLVR_SPI_CS_ENABLE;
  if(HAL_SPI_TransmitReceive(_hspi, spi_write.byte, spi_read.byte, 3, 100) != HAL_OK)
  {
    RSLVR_SPI_CS_DISABLE;
    return -1;
  }
  RSLVR_SPI_CS_DISABLE;    
  memcpy(data, &spi_read.byte[1], 2);
  return 1;
}

int8_t init_resolver(SPI_HandleTypeDef *hspi)
{
  uint8_t index = 0;
  AMC1210_WRITE_DATA spi_write;
  AMC1210_WRITE_DATA spi_read;  

  RSLVR_SPI_CS_DISABLE;
  _hspi = hspi;
  memset(&spi_write.value, 0, sizeof(spi_write.value));
  memset(&spi_read.value, 0, sizeof(spi_read.value));
  memset(&_resolver_var, 0, sizeof(_resolver_var));
  _resolver_var.init_ok = -1;
	
	//RSLVR_RESET_LOW;	HAL_Delay(1);
	//RSLVR_RESET_HIGH;	HAL_Delay(1);
	
  _resolver_var.spi_state = AMC1210_SPI_RSV_DATA_READ_OK;
  _resolver_var.spi_busy = AMC1210_SPI_READY;
	
  for(index = 0; index < 2; index++)
  {
    // control parameter
		resolver_config_spi_write_data(AMC1210_CTRL_PARAM_REG_ADDR[index], 0, b00000000);
    
    // sinc. filter
		// ByteH = b0000 1111:
    // ByteL = b0111 1100: 255 (SOSR = 256)
		// ByteL = b0011 1111:
    resolver_config_spi_write_data(AMC1210_SINC_FILTER_REG_ADDR[index], b00001111, b01111100);		
    
    // integrator parameter
		// ByteH = b0000 0111:
    // ByteL = b0011 1111:	// Oversampling mode
		// ByteL = b1011 1111:	// Sample-and-hold mode
    resolver_config_spi_write_data(AMC1210_INTEGRATOR_PARAM_REG_ADDR[index], b00000111, b00111111);
		
    /*   
    // high level thershold
		resolver_config_spi_write_data(AMC1210_HIGH_LEVEL_THRESHOLD_REG_ADDR[index], b00000011, b11111111);
		resolver_config_spi_write_data(AMC1210_LOW_LEVEL_THRESHOLD_REG_ADDR[index], b00000011, b11111111);
    
    // Caomparator filter parameter
		resolver_config_spi_write_data(AMC1210_COMP_FILTER_PARAM_REG_ADDR[index], b00000000, b00000000);		
    */
    HAL_Delay(5);
  }  


  // Control reg.
  // ByteH = b0000 0011:  // bit13: MIE interrupt pin and flag are bloked (interrupt pin INT always inactive)
  // ByteH = b0010 0011:  // bit13: MIE interrupt pin and flag are not bloked and can be set/reset
  // ByteL = b1110 0111: ONTROL.value.PC0_9 = 0x3E7;
  resolver_config_spi_write_data(AMC1210_CTRL_REG_ADDR, b00000011, b11100111);
  
  for(index = 0; index < 64; index++)
  {
    // ByteH = Pattern H:
    // ByteL = Pattern L:
    resolver_config_spi_write_data(AMC1210_PATTERN_REG_ADDR, ((AMC1210_SIG_PATTERN[index] >> 8 ) & 0xFF), (AMC1210_SIG_PATTERN[index] & 0xFF));    
  }

  // Clock divider reg.
	// Motor Control Loop Freq = carrier freq.
  // SD0_3 = f_clk / (f_carrier * PC0_9)
  // ByteH = b0001 1111:
  // ByteL = b0000 1111:
  resolver_config_spi_write_data(AMC1210_CLK_DIV_REG_ADDR, b00011111, b00000000);  
  //resolver_config_spi_read_data(AMC1210_CLK_DIV_REG_ADDR, spi_read.byte);  
/*
  spi_write.value.byteH = b00011110;  
  spi_write.value.byteL = b00000011;
*/


//  _resolver_var.spi_state = AMC1210_SPI_RSV_DATA_READ_OK;
//  _resolver_var.spi_busy = AMC1210_SPI_READY;

  _resolver_var.init_ok = 1;
  return _resolver_var.init_ok;
}


uint8_t _spi_DMA_write_byte[5];
uint8_t _spi_DMA_read_byte[5];

void resolver_read_data()
{
    if(_resolver_var.spi_state == AMC1210_SPI_RSV_READY_SIN_DATA)
      resolver_read_sin();
    else if(_resolver_var.spi_state == AMC1210_SPI_RSV_READY_COS_DATA)    
      resolver_read_cos();
}

int8_t resolver_read_sin()
{
  memset(_spi_DMA_write_byte, 0, 5);
  memset(_spi_DMA_read_byte, 0, 5);
  
  if(_resolver_var.init_ok < 0)
	  return -1;
  
  if(_resolver_var.spi_busy == AMC1210_SPI_BUSY)
    return -1;
  
  _resolver_var.spi_busy = AMC1210_SPI_BUSY;
  
  _spi_DMA_write_byte[0] = 0x80 | AMC1210_DATA_REG_ADDR[0];
  RSLVR_SPI_CS_ENABLE;
  HAL_SPI_TransmitReceive_DMA(_hspi, _spi_DMA_write_byte, _spi_DMA_read_byte, 5);
	_resolver_var.spi_state = AMC1210_SPI_RSV_READY_SIN_DATA;
    
  return 1;
}

int8_t resolver_read_cos()
{
  memset(_spi_DMA_write_byte, 0, 5);
  memset(_spi_DMA_read_byte, 0, 5);
  
	if(_resolver_var.init_ok < 0)
	  return -1;

  if(_resolver_var.spi_busy == AMC1210_SPI_BUSY)
    return -1;
  
  _resolver_var.spi_busy = AMC1210_SPI_BUSY;  
  
  _spi_DMA_write_byte[0] = 0x80 | AMC1210_DATA_REG_ADDR[1];
  RSLVR_SPI_CS_ENABLE;
  HAL_SPI_TransmitReceive_DMA(_hspi, _spi_DMA_write_byte, _spi_DMA_read_byte, 5);
  
  return 1;
}

int8_t resolver_read_DMA_callback( )
{
  float temp = 0.0f;
  
  RSLVR_SPI_CS_DISABLE;
  if(_resolver_var.spi_state == AMC1210_SPI_RSV_READY_SIN_DATA)
  {
    _resolver_var.sin = _spi_DMA_read_byte[1]*16777216 + _spi_DMA_read_byte[2]*65536+ _spi_DMA_read_byte[3]*256+ _spi_DMA_read_byte[4];
    _resolver_var.spi_state = AMC1210_SPI_RSV_READY_COS_DATA;  
    _resolver_var.spi_busy = AMC1210_SPI_READY;
		resolver_read_cos();
    return _resolver_var.spi_state;
  }
  else if(_resolver_var.spi_state == AMC1210_SPI_RSV_READY_COS_DATA)
  {
    _resolver_var.cos = _spi_DMA_read_byte[1]*16777216 + _spi_DMA_read_byte[2]*65536+ _spi_DMA_read_byte[3]*256+ _spi_DMA_read_byte[4]; 
    _resolver_var.theta = atan2f((float)_resolver_var.sin, (float)_resolver_var.cos);
    temp = _resolver_var.theta*570.295779513f;//180.0f/3.141592f*10.0f;  
		_resolver_var.single_turn = (int32_t)(temp);
	
	/*
	if((_resolver_var.single_turn <= 0) && (_resolver_var.single_turn_prv > 0)) {
		_resolver_var.multi_turn++;
	}
	else if((_resolver_var.single_turn >= 0) && (_resolver_var.single_turn_prv < 0)) {
		_resolver_var.multi_turn--;
	}
	
	*/
	if(_resolver_var.single_turn_prv < _resolver_var.single_turn)
	{
		if(_resolver_var.direction == -1)
		{
			if(fabs(_resolver_var.single_turn - _resolver_var.single_turn_prv) > 3550)
				_resolver_var.multi_turn--;
			else
				_resolver_var.direction = 1;
		}
		else
			_resolver_var.direction = 1;
	}
	else if(_resolver_var.single_turn_prv > _resolver_var.single_turn)
	{
		if(_resolver_var.direction == 1)
		{
			if(fabs(_resolver_var.single_turn - _resolver_var.single_turn_prv) > 3550)
				_resolver_var.multi_turn++;
			else
				_resolver_var.direction = -1;		
		}
		else
			_resolver_var.direction = -1;		
	}



		_resolver_var.pulse = _resolver_var.multi_turn*3600 + _resolver_var.single_turn + 1800;
		_resolver_var.single_turn_prv = _resolver_var.single_turn;
	
    _resolver_var.spi_state = AMC1210_SPI_RSV_DATA_READ_OK;  
    _resolver_var.spi_busy = AMC1210_SPI_READY;   
	
	GPIOA->BSRR = (uint32_t)GPIO_PIN_2 << 16U;
	return _resolver_var.spi_state;
  }
  return -1;
}
