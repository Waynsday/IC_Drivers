/*
 * MT25QLxx.c
 *
 *  Created on: Jun 12, 2024
 *      Author: Wayne Akeboshi
 *      Organization: De La Salle University Manila
 *
 *  Component: MT25QL01GBBB8ESF-0SIT
 *  Capacity: 1 Gigabit
 *  Tested MCU Compatibility: STM32F76xx
 *  Reference: MT25QL01GBBB Datasheet
 *    		 : STMicroelectronics Official Driver for MT25QL512ABB
 *  		 : https://github.com/STMicroelectronics/stm32-mt25ql512abb/blob/main/mt25ql512abb.c
 *
 *  Description:
 *
 *  This generic driver is designed to work with the HAL library of STM32.
 *  This SPI line is configured as follows:
 *  - No Hardware NSS
 *  - 8-bit dataframe width
 *  - Capped at 133MHz clock speed due to the chip
 *  - Configured for maximum two FMs only
 *  - Use code generator of STM32CubeIDE
 *
 *  Driver Notes:
 *  - CS must immediately be held high after the 8th bit of the last data byte of the command.
 *  Otherwise, the command will not be executed.
 *  - 4-byte address necessary for FM > 128Mbit or 16MBytes
 */

#include "main.h"
#include "MT25QLxx.h"

/*********************************************** USER CONFIG ********************************************/
// Add as many CS lines for FMs needed
#define CS1_LOW() 		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);	//Change pin accordingly
#define CS1_HIGH() 		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);		//Change pin accordingly
#define CS2_LOW() 		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);	//Change pin accordingly
#define CS2_HIGH() 		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);		//Change pin accordingly

/********************************************************************************************************/

#define CHECK_BIT(var,pos) (((var)>>(pos)) & 1)									//Check if pos bit in var is set

/********************************************************************************************************
 *																										*
 *																										*
 *											HELPER FUNCTIONS											*
 *																										*
 *																										*
 *********************************************************************************************************/
//Helper function prototypes are not declared in the header file

/*********************************************************************
 * @brief             - Helper functions for controlling CS pins of FM1 and FM2
 * @param[in]         - FM Selection
 * @return            - None
 * @Note              - Add as many flash memories as needed.
 */
void csLOW(MT25QL_FM_NO_t FM)
{
	switch(FM)
	{
		case MT25QL_FM1:
			CS1_LOW();
			break;
		case MT25QL_FM2:
			CS2_LOW();
			break;
		default:
			CS1_LOW();
			break;
	}
}

/*********************************************************************
 * @brief             - Helper functions for controlling CS pins of FM1 and FM2
 * @param[in]         - FM Selection
 * @return            - None
 * @Note              - Add as many flash memories as needed.
 */
void csHIGH(MT25QL_FM_NO_t FM)
{
	switch(FM)
	{
		case MT25QL_FM1:
			CS1_HIGH();
			break;
		case MT25QL_FM2:
			CS2_HIGH();
			break;
		default:
			CS1_HIGH();
			break;
	}
}

/*********************************************************************
 * @brief             - Helper functions for writing and reading to SPI
 * @param[in]         - Pointer to SPI peripheral
 * @param[in]         - Pointer to data to be sent/received
 * @param[in]         - Length of data to be sent/received
 * @return            - None
 * @Note              - None
 */
void SPI_Write(SPI_HandleTypeDef *hspi, uint8_t *data, uint8_t len)
{
	HAL_SPI_Transmit(hspi, data, len, HAL_MAX_DELAY);
}

void SPI_Read(SPI_HandleTypeDef *hspi, uint8_t *data, uint32_t len)
{
	HAL_SPI_Receive(hspi, data, len, HAL_MAX_DELAY);
}

/*********************************************************************
 * @brief             - Helper functions for enabling/disabling write enable latch
 * @param[in]         - Pointer to SPI peripheral
 * @param[in]         - FM Selection
 * @return            - None
 * @Note              - None
 */
void write_enable(SPI_HandleTypeDef *hspi, MT25QL_FM_NO_t FM)
{
	do{
		uint8_t tData = WRITE_ENABLE_CMD;
		csLOW(FM);
		SPI_Write(hspi, &tData, 1);
		csHIGH(FM);
	}while(!(CHECK_BIT(MT25QL_ReadSR(hspi, FM), 1)));

}

void write_disable(SPI_HandleTypeDef *hspi, MT25QL_FM_NO_t FM)
{
	do{
		uint8_t tData = WRITE_DISABLE_CMD;
		csLOW(FM);
		SPI_Write(hspi, &tData, 1);
		csHIGH(FM);
	}while(CHECK_BIT(MT25QL_ReadSR(hspi, FM), 1));
}

/*********************************************************************
 * @brief             - Helper functions for setting LSB first
 * @param[in]         - 16-bit value
 * @return            - Reversed 16-bits
 * @Note              - None
 */
uint16_t reverse_bits(uint16_t byte)
{
	byte = (byte & 0xFF00) >> 8 | (byte & 0xFF) << 8;
	byte = (byte & 0xF0F0) >> 4 | (byte & 0xF0F) << 4;
	byte = (byte & 0xCCCC) >> 2 | (byte & 0x3333) << 2;
	byte = (byte & 0xAAAA) >> 1 | (byte & 0x555) << 1;
	return byte;
}

void wait_for_ready(SPI_HandleTypeDef *hspi, MT25QL_FM_NO_t FM) {
    while (CHECK_BIT(MT25QL_ReadSR(hspi, FM), 0)); // Wait until the write in progress bit is cleared
}

/********************************************************************************************************
 *																										*
 *																										*
 *											COMMAND FUNCTIONS											*
 *																										*
 *																										*
 *********************************************************************************************************/

/*********************************************************************
 * @brief             - Send reset command to selected FM
 * @param[in]         - Pointer to SPI peripheral
 * @param[in]         - FM selected (FM1, FM2)
 * @return            - None
 * @Note              - None
 */
void MT25QL_Reset(SPI_HandleTypeDef *hspi, MT25QL_FM_NO_t FM)
{
	uint8_t tData[2];
	tData[0] = RESET_ENABLE_CMD;
	tData[1] = RESET_MEMORY_CMD;
	csLOW(FM);
	SPI_Write(hspi, tData, 2);
	csHIGH(FM);
}

/*********************************************************************
 * @brief             - Helper functions for writing and reading to SPI
 * @param[in]         - Pointer to SPI peripheral
 * @param[in]         - FM selected (FM1, FM2)
 * @return            - 24-bit JEDEC ID (Manufacturer ID, Device ID, Unique ID)
 * @Note              - See Table 19: Device ID Data
 */
uint32_t MT25QL_ReadID(SPI_HandleTypeDef *hspi, MT25QL_FM_NO_t FM)
{
	uint8_t tData = READ_ID_CMD;
	uint8_t rData[3] = {0};
	csLOW(FM);
	SPI_Write(hspi, &tData, 1);
	SPI_Read(hspi, rData, 3);
	csHIGH(FM);
	return ((rData[0] <<16)|(rData[1]<<8)|rData[2]);
}

/**************			READ REGISTERS 			*****************/

/*********************************************************************
 * @brief             - Read Status Register
 * @param[in]         - Pointer to SPI peripheral
 * @param[in]         - FM selected (FM1, FM2)
 * @return            - 8-bit status register
 * @Note              - See Table 3: Status Register
 */
uint8_t MT25QL_ReadSR(SPI_HandleTypeDef *hspi, MT25QL_FM_NO_t FM)
{
	uint8_t tData = READ_STATUS_REG_CMD;
	uint8_t rData = 0;
	csLOW(FM);
	SPI_Write(hspi, &tData, 1);
	SPI_Read(hspi, &rData, 1);
	csHIGH(FM);
	return rData;
}

/*********************************************************************
 * @brief             - Read Flag Status Register
 * @param[in]         - Pointer to SPI peripheral
 * @param[in]         - FM selected (FM1, FM2)
 * @return            - 8-bit status register
 * @Note              - See Table 5: Flag Status Register
 */
uint8_t MT25QL_ReadFSR(SPI_HandleTypeDef *hspi, MT25QL_FM_NO_t FM)
{
	uint8_t tData = READ_FLAG_STATUS_REG_CMD;
	uint8_t rData = 0;
	csLOW(FM);
	SPI_Write(hspi, &tData, 1);
	SPI_Read(hspi, &rData, 1);
	csHIGH(FM);
	return rData;
}

/*********************************************************************
 * @brief             - Read Nonvolatile Configuration Register
 * @param[in]         - Pointer to SPI peripheral
 * @param[in]         - FM selected (FM1, FM2)
 * @return            - 16-bit status register
 * @Note              - See Table 7: Nonvolatile Configuration Register
 */
uint16_t MT25QL_ReadNONVOL(SPI_HandleTypeDef *hspi, MT25QL_FM_NO_t FM)
{
	uint8_t tData = READ_NONVOL_CFG_REG_CMD;
	uint8_t rData[2] = {0};
	csLOW(FM);
	SPI_Write(hspi, &tData, 1);
	SPI_Read(hspi, rData, 1);
	csHIGH(FM);
	return ((rData[0] <<8) | (rData[1]));
}

/*********************************************************************
 * @brief             - Read Volatile Configuration Register
 * @param[in]         - Pointer to SPI peripheral
 * @param[in]         - FM selected (FM1, FM2)
 * @return            - 8-bit status register
 * @Note              - See Table 8: Volatile Configuration Register
 */
uint8_t MT25QL_ReadVOL(SPI_HandleTypeDef *hspi, MT25QL_FM_NO_t FM)
{
	uint8_t tData = READ_VOL_CFG_REG_CMD;
	uint8_t rData = 0;
	csLOW(FM);
	SPI_Write(hspi, &tData, 1);
	SPI_Read(hspi, &rData, 1);
	csHIGH(FM);
	return rData;
}

/*********************************************************************
 * @brief             - Read Extended Address Register
 * @param[in]         - Pointer to SPI peripheral
 * @param[in]         - FM selected (FM1, FM2)
 * @return            - 8-bit status register
 * @Note              - See Table 6: Extended Status Register
 */
uint8_t MT25QL_ReadXADDR(SPI_HandleTypeDef *hspi, MT25QL_FM_NO_t FM)
{
	uint8_t tData = READ_EXTENDED_ADDR_REG_CMD;
	uint8_t rData = 0;
	csLOW(FM);
	SPI_Write(hspi, &tData, 1);
	SPI_Read(hspi, &rData, 1);
	csHIGH(FM);
	return rData;
}


/**************			WRITE REGISTERS 			*****************/

/*********************************************************************
 * @brief             - Write Status Register
 * @param[in]         - Pointer to SPI peripheral
 * @param[in]         - FM selected (FM1, FM2)
 * @param[in]         - 8-bit new register value
 * @return            - None
 * @Note              - See Table 3: Status Register
 */
void MT25QL_WriteSR(SPI_HandleTypeDef *hspi, MT25QL_FM_NO_t FM, uint8_t reg_val)
{
	uint8_t tData[2];
	tData[0] = WRITE_STATUS_REG_CMD;
	tData[1] = reg_val;
	write_enable(hspi, FM);
	csLOW(FM);
	SPI_Write(hspi, tData, 1);
	csHIGH(FM);
	//write enable reset is automatic
}

/*********************************************************************
 * @brief             - Write Nonvolatile Configuration Register
 * @param[in]         - Pointer to SPI peripheral
 * @param[in]         - FM selected (FM1, FM2)
 * @param[in]         - 16-bit new register value
 * @return            - None
 * @Note              - See Table 7: Nonvolatile Configuration Register
 * 					  - Writing to nonvolatile cfg requires LSB first input
 */
void MT25QL_WriteNONVOL(SPI_HandleTypeDef *hspi, MT25QL_FM_NO_t FM, uint16_t reg_val)
{
	uint8_t tData[3];
	reg_val = reverse_bits(reg_val);
	tData[0] = WRITE_NONVOL_CFG_REG_CMD;
	tData[1] = (reg_val) & 0xFF;
	tData[2] = (reg_val >> 8) & 0xFF;

	write_enable(hspi, FM);
	csLOW(FM);
	SPI_Write(hspi, tData, 3);
	csHIGH(FM);

	//write enable reset is automatic
}

/*********************************************************************
 * @brief             - Clear flag status register
 * @param[in]         - Pointer to SPI peripheral
 * @param[in]         - FM selected (FM1, FM2)
 * @return            - None
 * @Note              - None
 */
void MT25QL_ClearFSR(SPI_HandleTypeDef *hspi, MT25QL_FM_NO_t FM)
{
	uint8_t tData = CLEAR_FLAG_STATUS_REG_CMD;
	csLOW(FM);
	SPI_Write(hspi, &tData, 1);
	csHIGH(FM);
}

/**************			READ FUNCTIONS 3-BYTE 			*****************/

/*********************************************************************
 * @brief             - Read at page & offset
 * @param[in]         - Pointer to SPI peripheral
 * @param[in]         - FM selected (FM1, FM2)
 * @param[in]         - Page of address
 * @param[in]         - Offset from start of page
 * @param[in]         - Size of data to read
 * @param[in]         - Variable to store data
 * @return            - None
 * @Note              - Max clock speed at 54MHz (Single IO STR). See Table 49: Supported Maximum Frequency
 */
void MT25QL_Read(SPI_HandleTypeDef *hspi, MT25QL_FM_NO_t FM, uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData)
{
	uint8_t tData[4];
	uint32_t Addr = (startPage * PAGE_SIZE) + offset;

	tData[0] = READ_CMD; //Read Command
	tData[1] = (Addr >> 16) &0xFF; //MSB Addr
	tData[2] = (Addr >> 8) &0xFF;
	tData[3] = (Addr) &0xFF; //LSB Addr

	csLOW(FM);
	SPI_Write(hspi, tData, 4);
	SPI_Read(hspi, rData, size);
	csHIGH(FM);
}
/*********************************************************************
 * @brief             - Read at address
 * @param[in]         - Pointer to SPI peripheral
 * @param[in]         - FM selected (FM1, FM2)
 * @param[in]         - Read Address
 * @param[in]         - Size of data to read
 * @param[in]         - Variable to store data
 * @return            - None
 * @Note              - Max clock speed at 54MHz (Single IO STR). See Table 49: Supported Maximum Frequency
 */
void MT25QL_ReadAddr(SPI_HandleTypeDef *hspi, MT25QL_FM_NO_t FM, uint32_t address, uint32_t size, uint8_t *rData)
{
	uint8_t tData[4];
	tData[0] = READ_CMD; //Read Command
	tData[1] = (address >> 16) &0xFF; //MSB Addr
	tData[2] = (address >> 8) &0xFF;
	tData[3] = (address) &0xFF; //LSB Addr

	csLOW(FM);
	SPI_Write(hspi, tData, 4);
	SPI_Read(hspi, rData, size);
	csHIGH(FM);
}
/*********************************************************************
 * @brief             - Fast Read at page & offset
 * @param[in]         - Pointer to SPI peripheral
 * @param[in]         - FM selected (FM1, FM2)
 * @param[in]         - Page of address
 * @param[in]         - Offset from start of page
 * @param[in]         - Size of data to read
 * @param[in]         - Variable to store data
 * @return            - None
 * @Note              - Max clock speed at 133MHz (Single IO STR). See Table 49: Supported Maximum Frequency
 */
void MT25QL_FastRead(SPI_HandleTypeDef *hspi, MT25QL_FM_NO_t FM, uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData)
{
	uint8_t tData[5];
	uint32_t Addr = (startPage * PAGE_SIZE) + offset;

	tData[0] = FAST_READ_CMD; //Read Command
	tData[1] = (Addr >> 16) &0xFF; //MSB Addr
	tData[2] = (Addr >> 8) &0xFF;
	tData[3] = (Addr) &0xFF; //LSB Addr
	tData[4] = 0; //Dummy cycle

	csLOW(FM);
	SPI_Write(hspi, tData, 5);
	SPI_Read(hspi, rData, size);
	csHIGH(FM);
}
/*********************************************************************
 * @brief             - Fast Read at address
 * @param[in]         - Pointer to SPI peripheral
 * @param[in]         - FM selected (FM1, FM2)
 * @param[in]         - Read Address
 * @param[in]         - Size of data to read
 * @param[in]         - Variable to store data
 * @return            - None
 * @Note              - Max clock speed at 133MHz (Single IO STR). See Table 49: Supported Maximum Frequency
 */
void MT25QL_FastReadAddr(SPI_HandleTypeDef *hspi, MT25QL_FM_NO_t FM, uint32_t address, uint32_t size, uint8_t *rData)
{
	uint8_t tData[5];
	tData[0] = FAST_READ_CMD; //Read Command
	tData[1] = (address >> 16) &0xFF; //MSB Addr
	tData[2] = (address >> 8) &0xFF;
	tData[3] = (address) &0xFF; //LSB Addr
	tData[4] = 0; //Dummy cycle

	csLOW(FM);
	SPI_Write(hspi, tData, 5);
	SPI_Read(hspi, rData, size);
	csHIGH(FM);
}

/**************			READ FUNCTIONS 4-BYTE 			*****************/

/*********************************************************************
 * @brief             - 4-Byte Read at page & offset
 * @param[in]         - Pointer to SPI peripheral
 * @param[in]         - FM selected (FM1, FM2)
 * @param[in]         - Page of address
 * @param[in]         - Offset from start of page
 * @param[in]         - Size of data to read
 * @param[in]         - Variable to store data
 * @return            - None
 * @Note              - Max clock speed at 54MHz (Single IO STR). See Table 49: Supported Maximum Frequency
 */
void MT25QL_4BRead(SPI_HandleTypeDef *hspi, MT25QL_FM_NO_t FM, uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData)
{
	uint8_t tData[5];
	uint32_t Addr = (startPage * PAGE_SIZE) + offset;

	tData[0] = ADDR_4BYTE_READ_CMD; //Read Command
	tData[1] = (Addr >> 24) &0xFF; //MSB Addr
	tData[2] = (Addr >> 16) &0xFF;
	tData[3] = (Addr >> 8) &0xFF;
	tData[4] = (Addr) &0xFF; //LSB Addr

	csLOW(FM);
	SPI_Write(hspi, tData, 5);
	SPI_Read(hspi, rData, size);
	csHIGH(FM);
}
/*********************************************************************
 * @brief             - 4-Byte Read at address
 * @param[in]         - Pointer to SPI peripheral
 * @param[in]         - FM selected (FM1, FM2)
 * @param[in]         - Read Address
 * @param[in]         - Size of data to read
 * @param[in]         - Variable to store data
 * @return            - None
 * @Note              - Max clock speed at 54MHz (Single IO STR). See Table 49: Supported Maximum Frequency
 */
void MT25QL_4BReadAddr(SPI_HandleTypeDef *hspi, MT25QL_FM_NO_t FM, uint32_t address, uint32_t size, uint8_t *rData)
{
	uint8_t tData[5];
	tData[0] = ADDR_4BYTE_READ_CMD; //Read Command
	tData[1] = (address >> 24) &0xFF; //MSB Addr
	tData[2] = (address >> 16) &0xFF;
	tData[3] = (address >> 8) &0xFF;
	tData[4] = (address) &0xFF; //LSB Addr


	csLOW(FM);
	SPI_Write(hspi, tData, 5);
	SPI_Read(hspi, rData, size);
	csHIGH(FM);
}
/*********************************************************************
 * @brief             - 4-Byte Fast Read at page & offset
 * @param[in]         - Pointer to SPI peripheral
 * @param[in]         - FM selected (FM1, FM2)
 * @param[in]         - Page of address
 * @param[in]         - Offset from start of page
 * @param[in]         - Size of data to read
 * @param[in]         - Variable to store data
 * @return            - None
 * @Note              - Max clock speed at 133MHz (Single IO STR). See Table 49: Supported Maximum Frequency
 */
void MT25QL_4BFastRead(SPI_HandleTypeDef *hspi, MT25QL_FM_NO_t FM, uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData)
{
	uint8_t tData[6];
	uint32_t Addr = (startPage * PAGE_SIZE) + offset;

	tData[0] = ADDR_4BYTE_FAST_READ_CMD; //Read Command
	tData[1] = (Addr >> 24) &0xFF; //MSB Addr
	tData[2] = (Addr >> 16) &0xFF;
	tData[3] = (Addr >> 8) &0xFF;
	tData[4] = (Addr) &0xFF; //LSB Addr
	tData[5] = 0; //Dummy cycle

	csLOW(FM);
	SPI_Write(hspi, tData, 6);
	SPI_Read(hspi, rData, size);
	csHIGH(FM);
}
/*********************************************************************
 * @brief             - 4-Byte Fast Read at page & offset
 * @param[in]         - Pointer to SPI peripheral
 * @param[in]         - FM selected (FM1, FM2)
 * @param[in]         - Read Address
 * @param[in]         - Size of data to read
 * @param[in]         - Variable to store data
 * @return            - None
 * @Note              - Max clock speed at 133MHz (Single IO STR). See Table 49: Supported Maximum Frequency
 */
void MT25QL_4BFastReadAddr(SPI_HandleTypeDef *hspi, MT25QL_FM_NO_t FM, uint32_t address, uint32_t size, uint8_t *rData)
{
	uint8_t tData[6];
	tData[0] = ADDR_4BYTE_FAST_READ_CMD; //Read Command
	tData[1] = (address >> 24) &0xFF; //MSB Addr
	tData[2] = (address >> 16) &0xFF;
	tData[3] = (address >> 8) &0xFF;
	tData[4] = (address) &0xFF; //LSB Addr
	tData[5] = 0; //Dummy cycle

	csLOW(FM);
	SPI_Write(hspi, tData, 6);
	SPI_Read(hspi, rData, size);
	csHIGH(FM);
}

/**************			PROGRAM OPERATIONS 			*****************/

/*********************************************************************
 * @brief             - Page program / Write to memory
 * @param[in]         - Pointer to SPI peripheral
 * @param[in]         - FM selected (FM1, FM2)
 * @param[in]         - Start page
 * @param[in]         - Offset from start page
 * @param[in]         - Size of data to be input
 * @param[in]		  - Data to be input
 * @return            - None
 * @Note              - Limited to 128Mb or 16MB address
 */
void MT25QL_PageProgram(SPI_HandleTypeDef *hspi, MT25QL_FM_NO_t FM, uint32_t page, uint16_t offset, uint32_t size, uint8_t *data)
{
	uint8_t tData[266];
	uint32_t Addr;
	uint16_t bytesToWrite;
	uint32_t dataIdx = 0;

	while(size > 0)
	{
		Addr  = (page * PAGE_SIZE) + offset;
		tData[0] = PAGE_PROG_CMD;
		tData[1] = (Addr >> 16) &0xFF; //MSB Addr Dummy
		tData[2] = (Addr >> 8) &0xFF;
		tData[3] = (Addr) &0xFF; //LSB Addr Dummy

		bytesToWrite = (size < (PAGE_SIZE - offset)) ? size : (PAGE_SIZE - offset); // Determine how many bytes to write
		for (uint16_t i = 0; i < bytesToWrite; i++) {
			tData[4 + i] = data[dataIdx + i]; // Copy data to buffer
		}
		write_enable(hspi, FM);
		csLOW(FM);
		HAL_SPI_Transmit(hspi, tData, 4 + bytesToWrite, HAL_MAX_DELAY);
		csHIGH(FM);

		wait_for_ready(hspi, FM); 	// Update variables for the next iteration

		page++;					//next page
        offset = 0; 			// Offset is only used for the first page
        size -= bytesToWrite;	//subtract sent bytes from size
		dataIdx += bytesToWrite;//go to data index of data and increment by written bytes
		HAL_Delay(1);

		write_disable(hspi, FM);	// Disable write operations
	}
}
/*********************************************************************
 * @brief             - 4-Byte Page program / Write to memory
 * @param[in]         - Pointer to SPI peripheral
 * @param[in]         - FM selected (FM1, FM2)
 * @param[in]         - Start page
 * @param[in]         - Offset from start page
 * @param[in]         - Size of data to be input
 * @param[in]		  - Data to be input
 * @return            - None
 * @Note              - Limited to 128Mb or 16MB address
 */
void MT25QL_4BPageProgram(SPI_HandleTypeDef *hspi, MT25QL_FM_NO_t FM, uint32_t page, uint16_t offset, uint32_t size, uint8_t *data)
{
	uint8_t tData[266];
	uint32_t Addr;
	uint16_t bytesToWrite;
	uint32_t dataIdx = 0;

	while(size > 0)
	{
		Addr  = (page * PAGE_SIZE) + offset;
		tData[0] = ADDR_4BYTE_PAGE_PROG_CMD;
		tData[1] = (Addr >> 24) &0xFF; //MSB Addr Dummy
		tData[2] = (Addr >> 16) &0xFF; //MSB Addr Dummy
		tData[3] = (Addr >> 8) &0xFF;
		tData[4] = (Addr) &0xFF; //LSB Addr Dummy

		bytesToWrite = (size < (PAGE_SIZE - offset)) ? size : (PAGE_SIZE - offset); // Determine how many bytes to write
		for (uint16_t i = 0; i < bytesToWrite; i++) {
			tData[5 + i] = data[dataIdx + i]; // Copy data to buffer
		}
		write_enable(hspi, FM);
		csLOW(FM);
		HAL_SPI_Transmit(hspi, tData, 4 + bytesToWrite, HAL_MAX_DELAY);
		csHIGH(FM);

		wait_for_ready(hspi, FM); 	// Update variables for the next iteration

		page++;					//next page
        offset = 0; 			// Offset is only used for the first page
        size -= bytesToWrite;	//subtract sent bytes from size
		dataIdx += bytesToWrite;//go to data index of data and increment by written bytes
		HAL_Delay(1);

		write_disable(hspi, FM);	// Disable write operations
	}
}

/**************			4-Byte Address Enable/Disable			*****************/
/*********************************************************************
 * @brief             - Enable 4-byte addressing on commands
 * @param[in]         - Pointer to SPI peripheral
 * @param[in]         - FM selected (FM1, FM2)
 * @return            - None
 * @Note              - None
 */
void MT25QL_4BAddrEnable(SPI_HandleTypeDef *hspi, MT25QL_FM_NO_t FM)
{
	uint8_t tData = ENTER_4_BYTE_ADDR_MODE_CMD;
	csLOW(FM);
	SPI_Write(hspi, &tData, 1);
	csHIGH(FM);
}
/*********************************************************************
 * @brief             - Disable 4-byte addressing on commands
 * @param[in]         - Pointer to SPI peripheral
 * @param[in]         - FM selected (FM1, FM2)
 * @return            - None
 * @Note              - None
 */
void MT25QL_4BAddrDisable(SPI_HandleTypeDef *hspi, MT25QL_FM_NO_t FM)
{
	uint8_t tData = EXIT_4_BYTE_ADDR_MODE_CMD;
	csLOW(FM);
	SPI_Write(hspi, &tData, 1);
	csHIGH(FM);
}

/**************			BLOCK ERASE COMMANDS				*****************/
void MT25QL_BlockErase(SPI_HandleTypeDef *hspi, MT25QL_FM_NO_t FM, uint32_t page, MT25QL_Erase_t BlockErase)
{
	uint8_t tData[5];
	uint32_t Addr = page * PAGE_SIZE;

	MT25QL_4BAddrEnable(hspi, FM);

	write_enable(hspi, FM);

	switch(BlockErase)
	{
		case MT25QL_ERASE_CHIP:
			tData[0] = BULK_ERASE_CMD;
			break;

		case MT25QL_ERASE_4K:
			tData[0] = SUBSECTOR_ERASE_4K_CMD;
			break;

		case MT25QL_ERASE_32K:
			tData[0] = SUBSECTOR_ERASE_32K_CMD;
			break;

		case MT25QL_ERASE_64K:
			tData[0] = SECTOR_ERASE_64K_CMD;
			break;
	}

	tData[1] = (Addr >> 24) &0xFF; //MSB Addr
	tData[2] = (Addr >> 16) &0xFF;
	tData[3] = (Addr >> 8) &0xFF;
	tData[4] = (Addr) &0xFF; //LSB Addr
	csLOW(FM);
	SPI_Write(hspi, tData, 5);
	csHIGH(FM);

	wait_for_ready(hspi, FM);
	MT25QL_4BAddrDisable(hspi, FM);
	write_disable(hspi, FM);
}

/********************************************************************************************************
 *																										*
 *																										*
 *										FM Generic Functions											*
 *																										*
 *																										*
 *********************************************************************************************************/
void MT25QL_Init(SPI_HandleTypeDef *hspi, MT25QL_FM_NO_t FM)
{
	//Reset chip
	MT25QL_Reset(hspi, FM);
	//Check if ID match
	uint8_t ID_Match = (MT25QL_ReadID(hspi, FM) & JEDEC_ID);

	//check if 4-byte addressing ENABLED
	do{
		//Reset chip
		MT25QL_Reset(hspi, FM);
		HAL_Delay(5); //ensure pins reach VCC
		//Check FSR, Bit 7 (Program/Erase ready) and Bit 0 (4-byte addressing) must be on.
		if(!(CHECK_BIT(MT25QL_ReadFSR(hspi, FM), 0) & 1)) //if 4-byte addressing is not on in NVC Register
		{
			//Copy current NVC register and set bit 0 to 0 for 4-byte addressing
			uint16_t currNVC = MT25QL_ReadNONVOL(hspi, FM);
			MT25QL_WriteNONVOL(hspi, FM, (currNVC &= ~1));
		}
		wait_for_ready(hspi, FM);
		//Reset for NVC to take effect
		MT25QL_Reset(hspi, FM);
	}while(!(MT25QL_ReadFSR(hspi, FM) & 0x81));

	// For first time initialization only. Comment out as needed.
	MT25QL_BlockErase(hspi, FM, 0, MT25QL_ERASE_CHIP);

	//Clear FSR
	MT25QL_ClearFSR(hspi, FM);

	//temp
	uint8_t SR = MT25QL_ReadSR(hspi, FM);
	uint8_t FSR = MT25QL_ReadFSR(hspi, FM);
	uint16_t NVC = MT25QL_ReadNONVOL(hspi, FM);
}
