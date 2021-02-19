
#ifndef RESOLVER_H
#define RESOLVER_H	1

#include "stm32f4xx.h"

#if defined(STM32F407IGHx)
  #define RSLVR_SPI_CS_PORT           GPIOB
  #define RSLVR_SPI_CS_PIN            GPIO_PIN_12
	#define RSLVR_RESET_PORT						GPIOI
	#define RSLVR_RESET_PIN							GPIO_PIN_5
	#define RSLVR_RESET_LOW							GPIOI->BSRR = (uint32_t)GPIO_PIN_5 << 16U;
	#define RSLVR_RESET_HIGH						GPIOI->BSRR = GPIO_PIN_5
	#define RSLVR_ACK_PORT							GPIOI
	#define RSLVR_ACK_PIN								GPIO_PIN_6
	#define RSLVR_ACK										(((GPIOI->IDR & GPIO_PIN_6) != (uint32_t)GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#endif

#define RSLVR_CH1  0
#define RSLVR_CH2  1
#define RSLVR_CH3  2
#define RSLVR_CH4  3

#define RSLVR_SPI_CS_DISABLE    (GPIOB->BSRR = GPIO_PIN_12)
#define RSLVR_SPI_CS_ENABLE     (GPIOB->BSRR = (uint32_t)GPIO_PIN_12 << 16U)

typedef struct
{
  // High-level interrupt flag for Filter x
  // 0: Comparator Filter x output is below the high limit threshold
  // 1: Comparator Filter x output is equal to or above the high level threshold, if enabled  
  uint32_t IFH1:1;
  uint32_t IFL1:1;
  uint32_t IFH2:1;
  uint32_t IFL2:1;
  uint32_t IFH3:1;
  uint32_t IFL3:1;
  uint32_t IFH4:1; 
  // Low-level interrupt flag for Filter x
  // 0: Comparator Filter x output is above the low limit threshold
  // 1: Comparator Filter x output is equal to or below the low level threshold, if enabled  
  uint32_t IFL4:1; 
  // Modulator failure flag for Filter x.
  // 0: Modulator is operating normally for Filter x
  // 1: Modulator failure for Filter x
  uint32_t MF1:1; 
  uint32_t MF2:1; 
  uint32_t MF3:1; 
  uint32_t MF4:1;
  // Acknowledge flag for Filter x.
  // 0: No new data available for Filter x
  // 1: New data available for Filter x
  uint32_t AF1:1; 
  uint32_t AF2:1; 
  uint32_t AF3:1; 
  uint32_t AF4:1; 
  
  uint32_t Rsv:16;
} AMC1210_ISR;  // address 0x00

typedef union
{
  AMC1210_ISR value;
  uint8_t byte[4];
}AMC1210_ISR_REG;

typedef struct
{
  // Delta-Sigma Modulator mode.
  // 00: The clock speed is equal to the data rate from the modulator
  // 01: The clock rate is half of the data rate from the modulator
  // 10: The data from the modulator is Manchester decoded
  // 11: The clock rate is twice the data rate of the modulator  
  uint32_t MOD0_1:2;  
  // Time measure mode.
  // 0: The time is measured from the last filter update to the last rising edge of the selected
  // 1: The time is measured between two rising edges of the selected sample-and-hold signal  
  uint32_t TM:1;
  // Sample-and-hold select.
  // 0: Signal SH1 is chosen as sample-and-hold signal
  // 1: Signal SH2 is chosen as sample-and-hold signal  
  uint32_t SHS:1;
  // Input clock direction.
  // 0: Pin CLKx is an input
  // 1: Pin CLKx is an output. The outgoing clock comes from the modulator clock divider.
  uint32_t CD:1;
  // Manchester status
  uint32_t MS0_10:11;  
  uint32_t Rsv:16;
}AMC1210_CTRL_PARAM;  // address 0x01, 0x07, 0x0D, 0x13

typedef union
{
  AMC1210_CTRL_PARAM value;
  uint8_t byte[4];
}AMC1210_CTRL_PARAM_REG;

typedef struct
{
// Oversampling ratio. The actual rate is SOSR+1
  uint32_t SOSR0_7:8;  
  // Filer enable
  uint32_t FEN:1;   
  // Acknowledge enable
  uint32_t AE:1;    
  // Sinc filter structure
  // 00: Sinc filter runs with a sincfast structure
  // 01: Sinc filter runs with a Sinc 1 structure
  // 10: Sinc filter runs with a Sinc 2 structure
  // 11: Sinc filter runs with a Sinc 3 structure
  uint32_t SST0_1:2;   
  
  uint32_t Rsv:20;  
}AMC1210_SINC_FILTER;   // address 0x02, 0x08, 0x0E, 0x14

typedef union
{
  AMC1210_SINC_FILTER value;
  uint8_t byte[4];
}AMC1210_SINC_FILTER_REG;

typedef struct
{
  // Oversampling ratio. The actual rate is IOSR + 1. 
  // These bits set the oversampling ratio of the integrator. 0x03 represents an oversampling ratio of 4.
  uint32_t IOSR0_6:7;  
  // Integrator mode.     
  // 0: The oversampling mode updates the data output of the integrator.
  // 1: The selected sample-and-hold signal updates the data output of the integrator
  uint32_t IMOD:1;  
  // Integrator enable.   
  // 0: The data from the sinc filter output is stored in the register map.
  // 1: The data from the integrator is stored in the register map
  uint32_t IEN:1;   
  // Demodulation enable. 
  // 0: The demodulation for resolver applications is disabled.
  // 1: The demodulation for resolver applications is enabled
  uint32_t DEN:1;   
  // Data representation. 
  // 0: The data is stored in 16-bit two's complement.
  // 1: The data is stored in 32-bit two's complement
  uint32_t DR:1;    
  // Shift control. These bits indicate by how many bits the 16-bit window is shifted up when 16-bit data representation is chosen.
  uint32_t SH0_4:5;    

  uint32_t Rsv:16;  
}AMC1210_INTEGRATOR_PARAM;   // address 0x03, 0x09, 0x0F, 0x15

typedef union
{
  AMC1210_INTEGRATOR_PARAM value;
  uint8_t byte[4];
}AMC1210_INTEGRATOR_PARAM_REG;


typedef struct
{
  uint32_t HLT0:1;  // Unsigned high level threshold for the comparator filter output.
  uint32_t HLT1:1;
  uint32_t HLT2:1;
  uint32_t HLT3:1;
  uint32_t HLT4:1;
  uint32_t HLT5:1;
  uint32_t HLT6:1;
  uint32_t HLT7:1;
  uint32_t HLT8:1;
  uint32_t HLT9:1;
  uint32_t HLT10:1;
  uint32_t HLT11:1;
  uint32_t HLT12:1;
  uint32_t HLT13:1;
  uint32_t HLT14:1;
  
  uint32_t Rsv:17;    // Unused. Alwalys read '0'  
}AMC1210_HIGH_LEVEL_THRESHOLD;  // address 0x04, 0x0A, 0x10, 0x16

typedef union
{
  AMC1210_HIGH_LEVEL_THRESHOLD value;
  uint8_t byte[4];
}AMC1210_HIGH_LEVEL_THRESHOLD_REG;

typedef struct
{
  uint32_t LLT0:1;  // Unsigned low level threshold for the comparator filter output.
  uint32_t LLT1:1;
  uint32_t LLT2:1;
  uint32_t LLT3:1;
  uint32_t LLT4:1;
  uint32_t LLT5:1;
  uint32_t LLT6:1;
  uint32_t LLT7:1;
  uint32_t LLT8:1;
  uint32_t LLT9:1;
  uint32_t LLT10:1;
  uint32_t LLT11:1;
  uint32_t LLT12:1;
  uint32_t LLT13:1;
  uint32_t LLT14:1;
  
  uint32_t Rsv:17;    // Unused. Alwalys read '0'    
}AMC1210_LOW_LEVEL_THRESHOLD;  // address 0x05, 0x0B, 0x11, 0x17

typedef union
{
  AMC1210_LOW_LEVEL_THRESHOLD value;
  uint8_t byte[4];
}AMC1210_LOW_LEVEL_THRESHOLD_REG;


typedef struct
{
  // Oversampling ratio. The actual rate is COSR + 1.
  // These bits set the oversampling ratio of the filter. 0xFF represents an oversampling ratio of 256.
  uint32_t COSR0_4:5;
  // High-level interrupt enable.
  // 0: The high-level interrupt flag as well as the output INT is disabled for this particular flag
  // 1: The high-level interrupt flag is enabled
  uint32_t IEH:1;
  // Low-level interrupt enable.
  // 0: The low-level interrupt flag as well as the output INT is disabled for this particular flag
  // 1: The low-level interrupt flag is enabled
  uint32_t IEL:1;
  // Comparator filter structure.
  // 00: Comparator filter runs with a sincfast structure
  // 01: Comparator filter runs with a Sinc 1 structure
  // 10: Comparator filter runs with a Sinc 2 structure
  // 11: Comparator filter runs with a Sinc 3 structure
  uint32_t CS0_1:2;
  // Modulator failure interrupt enable.
  // 0: The modulator failure flag as well as the output INT is disabled for this particular flag
  // 1: The modulator failure flag is enabled
  uint32_t MFIE:1;

  uint32_t Rsv:22;   // Unused. Alwalys read '0'

}AMC1210_COMP_FILTER_PARAM; // address 0x06, 0x0C, 0x12, 0x18

typedef union
{
  AMC1210_COMP_FILTER_PARAM value;
  uint8_t byte[4];
}AMC1210_COMP_FILTER_PARAM_REG;


typedef struct
{
  // Pattern count. Defines the length of the shift register for the signal generator
  uint32_t PC0_9:10;
  uint32_t Rsv1:1;   // Unused. Always read '0'.  
  // Master interrupt enable.
  // 0: Interrupt pin and interrupt flags are blocked (interrupt pin INT always inactive).
  // 1: Interrupt pin and interrupt flags are not blocked and can be set and reset (if individually enabled).
  uint32_t MIE:1;
  // Interrupt polarity for pin INT.
  // 0: An interrupt is signaled with a positive transition on the pin INT
  // 1: An interrupt is signaled with a negative transition on the pin INT
  uint32_t IP:1;
  // Acknowledge polarity for pin ACK.
  // 0: New data is signaled with a '1' on the pin ACK
  // 1: New data is signaled with a '0' on the pin ACK
  uint32_t AP:1;

  uint32_t Rsv:16;
}AMC1210_CTRL;  // address 0x19

typedef union
{
  AMC1210_CTRL value;
  uint8_t byte[4];
}AMC1210_CTRL_REG;


typedef struct
{
  // Shift register pattern. Write-only reg.
  uint32_t SP0:1;
  uint32_t SP1:1;
  uint32_t SP2:1;
  uint32_t SP3:1;
  uint32_t SP4:1;
  uint32_t SP5:1;
  uint32_t SP6:1;
  uint32_t SP7:1;
  uint32_t SP8:1;
  uint32_t SP9:1;
  uint32_t SP10:1;
  uint32_t SP11:1;
  uint32_t SP12:1;
  uint32_t SP13:1;
  uint32_t SP14:1;
  uint32_t SP15:1;
  
  uint32_t Rsv:16;
}AMC1210_PATTERN;   // address 0x1A

typedef union
{
  AMC1210_PATTERN value;
  uint16_t word[2];
  uint8_t byte[4];
}AMC1210_PATTERN_REG;


typedef struct
{
  // Signal generator clock divider.
  // 0000: Clock divider is off, outgoing clock equals incoming clock
  // 0001: Outgoing clock is divided by 2
  // 0010: Outgoing clock is divided by 3
  // 0011: Outgoing clock is divided by 4
  // 0100: Outgoing clock is divided by 5
  // 0101: Outgoing clock is divided by 6
  // 0110: Outgoing clock is divided by 7
  // 0111: Outgoing clock is divided by 8
  // 1000: Outgoing clock is divided by 9
  // 1001: Outgoing clock is divided by 10
  // 1010: Outgoing clock is divided by 11
  // 1011: Outgoing clock is divided by 12
  // 1100: Outgoing clock is divided by 13
  // 1101: Outgoing clock is divided by 14
  // 1110: Outgoing clock is divided by 15
  // 1111: Outgoing clock is divided by 16  
  uint32_t SD0_3:4;
  // Modulator clock divider. The coding is equal to the first eight codes in SD; see below.
  uint32_t MD0_2:3;
  // Signal generator Control Select (necessary for Phase Calibration and Demodulation on the selected channel).
  // 00: The phase calibration is performed on filter module 1
  // 01: The phase calibration is performed on filter module 2.
  // 10: The phase calibration is performed on filter module 3.
  // 11: The phase calibration is performed on filter module 4.
  uint32_t SCS0_1:2;
  // Start of phase correction. Writing a '1' to this bit starts the phase calibration. Reading this bit shows the phase calibration status:
  // 1: The phase calibration is performing
  // 0: No phase calibration is performing
  uint32_t PCAL:1;
  // Signal Generator enable.
  // 0: Signal generator is disabled
  // 1: Signal generator is enabled
  uint32_t SGE:1;
  // Master Filter Enable. Functionally AND'ed with bit FEN in the Sinc Filter Parameter Register.
  // 0: Sinc filter units of all filter modules are disabled.
  // 1: Sinc filter units can be enabled if bit FEN is '1'.
  uint32_t MFE:1;
  // Signal Generator High-Current Output.
  // 0: The high current option for pins PWM1 and PWM2 is disabled
  // 1: The PWM1 and PWM2 outputs are in High Current Mode
  uint32_t HBE:1;
  
  uint32_t Rsv:19;   // Unused. Always read '0'.
}AMC1210_CLK_DIV;   // address 0x1B

typedef union
{
  AMC1210_CLK_DIV value;
  uint8_t byte[4];
}AMC1210_CLK_DIV_REG;


typedef struct
{
    // Integrator overflow for filter module x.
  // 0: No overflow has occurred
  // 1: An overflow occurred in the integrator unit in filter module x
  uint32_t IO1:1;
   // Time counter overflow for filter module x.
  // 0: No overflow has occurred
  // 1: An overflow occurred in the time measurement unit in filter module x
  uint32_t TO1:1;
  uint32_t IO2:1;
  uint32_t TO2:1;
  uint32_t IO3:1;
  uint32_t TO3:1;
  uint32_t IO4:1;
  uint32_t TO4:1; 
  // Manchester failure status for filter module x.
  // 0: The automatic Manchester encoder calibration has worked properly since last read access
  // 1: The automatic Manchester encoder has detected problems since last read access
  uint32_t MAF1:1;
  uint32_t MAF2:1;
  uint32_t MAF3:1;
  uint32_t MAF4:1;
  // Manchester locked status for filter module x.
  // 0: The automatic Manchester encoder calibration is working properly
  // 1: The automatic Manchester encoder calibration has not been able to perform a successful calibration
  uint32_t MAL1:1;
  uint32_t MAL2:1;
  uint32_t MAL3:1;
  uint32_t MAL4:1;

  uint32_t Rsv:16;   // Unused. Always read '0'.
}AMC1210_STATUS;    // address 0x1C

typedef union
{
  AMC1210_STATUS value;
  uint8_t byte[4];
}AMC1210_STATUS_REG;

typedef struct
{
  // Data from the sinc filter or the integrator filter in 16-bit formatting.
  int16_t data;
}AMC1210_DATA_16BIT_REG;    // address 0x1D, 0x1F, 0x21, 0x23


typedef struct
{
  // Data from the sinc filter or the integrator filter in 32-bit formatting.
  int32_t data;
}AMC1210_DATA_32BIT_REG;    // address 0x1D, 0x1F, 0x21, 0x23


typedef struct
{
  // Data from the time measure unit.
  int16_t data;
}AMC1210_TIME_REG;  // address 0x1E, 0x20, 0x22, 0x24


typedef struct
{
  AMC1210_ISR_REG                   INTERRUPT_CTRL;     // address 0x00
  AMC1210_CTRL_REG                  CONTROL;            // address 0x19
  AMC1210_PATTERN_REG               PATTERN;            // address 0x1A
  AMC1210_CLK_DIV_REG               CLK_DIVIDER;        // address 0x1B
  AMC1210_STATUS_REG                STATUS;             // address 0x1C
}AMC1210_REG;  

typedef struct
{
  AMC1210_CTRL_PARAM_REG            CONTROL_PARAM;      // address 0x01, 0x07, 0x0D, 0x13
  AMC1210_SINC_FILTER_REG           SINC_FILTER;        // address 0x02, 0x08, 0x0E, 0x14
  AMC1210_INTEGRATOR_PARAM_REG      INTEGRATOR_PARAM;   // address 0x03, 0x09, 0x0F, 0x15
  AMC1210_HIGH_LEVEL_THRESHOLD_REG  HIGH_LVL_THRESHOLD; // address 0x04, 0x0A, 0x10, 0x16
  AMC1210_LOW_LEVEL_THRESHOLD_REG   LOW_LVL_THRESHOLD;  // address 0x05, 0x0B, 0x11, 0x17
  AMC1210_COMP_FILTER_PARAM_REG     COMP_FILTER_PARAM;  // address 0x06, 0x0C, 0x12, 0x18
  AMC1210_DATA_16BIT_REG            DATA_16BIT;         // address 0x1D, 0x1F, 0x21, 0x23
  AMC1210_DATA_32BIT_REG            DATA_32BIT;         // address 0x1D, 0x1F, 0x21, 0x23
  AMC1210_TIME_REG                  TIME;               // address 0x1E, 0x20, 0x22, 0x24
}AMC1210_CH_REG;

typedef struct
{
  // 1 : Read / 0: Write
  uint32_t address:7;
  uint32_t RW:1;
  uint32_t byteH:8;
  uint32_t byteL:8;
  uint32_t Rsv:8;
  uint32_t Rsv1:16;
}AMC1210_WRITE_CONTROL_DATA;

typedef union
{
  AMC1210_WRITE_CONTROL_DATA value;
  uint8_t byte[6];
}AMC1210_WRITE_DATA;


#define AMC1210_SPI_RSV_READY_SIN_DATA  1
#define AMC1210_SPI_RSV_READY_COS_DATA  2
#define AMC1210_SPI_RSV_DATA_READ_OK    3
#define AMC1210_SPI_BUSY    1
#define AMC1210_SPI_READY   0

typedef struct
{
    uint8_t busy;
    uint8_t state;
}AMC1210_RSV_STATE;

typedef struct
{
	int8_t 	init_ok;
	int32_t cos;  
	int32_t sin;
	float 	theta;
	int32_t single_turn;
	int32_t single_turn_prv;
	int32_t multi_turn;
	int32_t pulse;
	int8_t 	direction;
	
	uint8_t spi_busy;
    uint8_t spi_state;
}AMC1210_RSV_VAR;

int8_t init_resolver(SPI_HandleTypeDef *hspi);
int8_t resolver_read_DMA_callback( );
void resolver_read_data();
int8_t resolver_read_sin();
int8_t resolver_read_cos();

extern uint8_t _spi_DMA_write_byte[5];
extern uint8_t _spi_DMA_read_byte[5];


extern AMC1210_RSV_VAR _resolver_var;

#endif /* RESOLVER_H */
