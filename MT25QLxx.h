/*
 * MT25QLxx.h
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

#ifndef INC_MT25QLXX_H_
#define INC_MT25QLXX_H_


// Change according to your MT25QL chip. See Table 19: Device ID Data
#define JEDEC_ID			0x20BA21U
#define DUMMY_CLOCKS		0xFF
/*
 * @brief MT25QL01GBBB Size Configuration.
 * All sizes measured in Bytes.
 * Page size Ref: Fig 2 Block Diagram
 * Sector sizes Ref: Memory Map
 */

#define SECTOR_64K			(uint32_t)(64 * 1024) 	/*2048 sectors of 64KB*/
#define SUBSECTOR_32K 		(uint32_t)(32 * 1024)	/*4096 sub-sectors of 32KB*/
#define SUBSECTOR_4K		(uint32_t)(4 * 1024)	/*32768 sub-sectors of 4KB*/

#define FLASH_SIZE			(uin32_t)((1024 * 1024 * 1024)/8)	/*1 Gigabit => 1024 Mbits => 128MB*/
#define PAGE_SIZE			(uint32_t)256			/*524,488 pages of 256 Bytes*/

/*
 * @brief MT25QL01GBBB Timing Configuration
 * All times measured in milliseconds (ms).
 * See Table 50. AC Characteristics and Operating conditions
 */
#define DIE_ERASE_MAX_TIME				460000U		/*512Mb DIE ERASE MAX => 460s*/
#define SECTOR_ERASE_MAX_TIME			1000U		/*SECTOR ERASE MAX => 1s*/
#define SUBSECTOR_32K_ERASE_MAX_TIME	1000U		/*SUBSECTOR32K ERASE MAX => 1s*/
#define SUBSECTOR_4K_ERASE_MAX_TIME		400U		/*SUBSECTOR4K ERASE MAX => 0.4s*/
#define NON_READ_MIN_TIME				1			/*NON-READ DESELECT MIN TIME => 50ns adjusted to 1ms for accuracy*/

#define RESET_MAX_TIME					1000U		/*Software Reset Recovery Time. Min 70 ns*/

/*
 * @brief MT25QL01GBBB 4-byte addressing state
 */
#define ADDR_4B_EN						1			//enabled 4-byte addressing
#define ADDR_4B_DI						0			//default 3-byte addressing

/******************************************************************************
 * @brief MT25QL01GBBB Commands
 * See Table 21: Command Set
  ****************************************************************************/
/***** Software RESET Operations****************/
#define RESET_ENABLE_CMD				0x66 		/*!< Reset Enable                                            */
#define RESET_MEMORY_CMD				0x99		/*!< Reset Memory                                            */

/***** READ ID Operations ****************************************************/
#define READ_ID_CMD                          0x9E   /*!< Read IDentification                                     */
#define READ_ID_CMD2                         0x9F   /*!< Read IDentification                                     */
#define MULTIPLE_IO_READ_ID_CMD              0xAF   /*!< QPI ID Read                                             */
#define READ_SERIAL_FLASH_DISCO_PARAM_CMD    0x5A   /*!< Read Serial Flash Discoverable Parameter                */

/***** READ MEMORY Operations ************************************************/
#define READ_CMD                             0x03U   /*!< Normal Read 3/4 Byte Address                            */
#define FAST_READ_CMD                        0x0BU   /*!< Fast Read 3/4 Byte Address                              */
#define DO_FAST_READ_CMD                   	 0x3BU   /*!< Dual Output Fast Read 3/4 Byte Address                  */
#define DIO_FAST_READ_CMD                    0xBBU   /*!< Dual Input/Output Fast Read 3/4 Byte Address            */
#define QO_FAST_READ_CMD                   	 0x6BU   /*!< Quad Output Fast Read 3/4 Byte Address                  */
#define QIO_FAST_READ_CMD                    0xEBU   /*!< Quad Input/Output Fast Read 3/4 Byte Address            */
#define FAST_READ_DTR_CMD                    0x0DU   /*!< DTR Fast Read 3/4 Byte Address                          */
#define DO_FAST_READ_DTR_CMD                 0x3DU   /*!< DTR Dual Output Fast Read 3/4 Byte Address              */
#define DIO_FAST_READ_DTR_CMD                0xBDU   /*!< DTR Dual Input/Output Fast Read 3/4 Byte Address        */
#define QO_FAST_READ_DTR_CMD                 0x6DU   /*!< DTR Quad Output Fast Read 3/4 Byte Address              */
#define QIO_FAST_READ_DTR_CMD                0xEDU   /*!< DTR Quad Input/Output Fast Read 3/4 Byte Address        */
#define QIO_WORD_READ_CMD                    0xE7U   /*!< Quad Input/Output Word Read 3/4 Byte Address            */

/***** READ MEMORY Operations with 4-Byte Address ****************************/
#define ADDR_4BYTE_READ_CMD                 0x13U   /*!< Normal Read 4 Byte address                              */
#define ADDR_4BYTE_FAST_READ_CMD            0x0CU   /*!< Fast Read 4 Byte address                                */
#define ADDR_4BYTE_1I2O_FAST_READ_CMD       0x3CU   /*!< Dual Output Fast Read 4 Byte address                    */
#define ADDR_4BYTE_2IO_FAST_READ_CMD        0xBCU   /*!< Dual Input/Output Fast Read 4 Byte address              */
#define ADDR_4BYTE_1I4O_FAST_READ_CMD       0x6CU   /*!< Quad Output Fast Read 4 Byte address                    */
#define ADDR_4BYTE_4IO_FAST_READ_CMD        0xECU   /*!< Quad Input/Output Fast Read 4 Byte address              */
#define ADDR_4BYTE_FAST_READ_DTR_CMD        0x0EU   /*!< DTR Fast Read 4 Byte Address                            */
#define ADDR_4BYTE_2IO_FAST_READ_DTR_CMD    0xBEU   /*!< DTR Dual Input/Output Fast Read 4 Byte address          */
#define ADDR_4BYTE_4IO_FAST_READ_DTR_CMD    0xEEU   /*!< DTR Quad Input/Output Fast Read 4 Byte address          */

/***** WRITE Operations ******************************************************/
#define WRITE_ENABLE_CMD                     0x06U   /*!< Write Enable                                            */
#define WRITE_DISABLE_CMD                    0x04U   /*!< Write Disable                                           */

/***** READ REGISTER Operations **********************************************/
#define READ_STATUS_REG_CMD                  0x05U   /*!< Read Status Register                                    */
#define READ_FLAG_STATUS_REG_CMD             0x70U   /*!< Read Flag Status Register                               */
#define READ_NONVOL_CFG_REG_CMD              0xB5U   /*!< Read NonVolatile Configuration Register                 */
#define READ_VOL_CFG_REG_CMD                 0x85U   /*!< Read Volatile Configuration Register                    */
#define READ_ENHANCED_VOL_CFG_REG_CMD        0x65U   /*!< Read Enhanced Volatile Configuration Register           */
#define READ_EXTENDED_ADDR_REG_CMD           0xC8U   /*!< Read Extended Address Register                          */
#define READ_GEN_PURP_REG_CMD                0x96U   /*!< Read Extended Address Register                          */

/***** WRITE REGISTER Operations *********************************************/
#define WRITE_STATUS_REG_CMD                 0x01U   /*!< Write Status Register                                   */
#define WRITE_NONVOL_CFG_REG_CMD             0xB1U   /*!< Write NonVolatile Configuration Register                */
#define WRITE_VOL_CFG_REG_CMD                0x81U   /*!< Write Volatile Configuration Register                   */
#define WRITE_ENHANCED_VOL_CFG_REG_CMD       0x61U   /*!< Write Enhanced Volatile Configuration Register          */
#define WRITE_EXTENDED_ADDR_REG_CMD          0xC5U   /*!< Write Extended Address Register                         */

/***** CLEAR FLAG STATUS REGISTER Operations *********************************/
#define CLEAR_FLAG_STATUS_REG_CMD            0x50U   /*!< Read Flag Status Register                               */

/***** PROGRAM Operations ****************************************************/
#define PAGE_PROG_CMD                        0x02U   /*!< Page Program 3/4 Byte Address                           */
#define DUAL_INPUT_FAST_PROG_CMD             0xA2U   /*!< Dual Input Fast Program 3/4 Byte Address                */
#define EXTENDED_DUAL_INPUT_FAST_PROG_CMD    0xD2U   /*!< Extended Dual Input Fast Program 3/4 Byte Address       */
#define QUAD_INPUT_FAST_PROG_CMD             0x32U   /*!< Quad Input Fast Program 3/4 Byte Address                */
#define EXTENDED_QUAD_INPUT_FAST_PROG_CMD    0x38U   /*!< Extended Quad Input Fast Program 3/4 Byte Address       */

/***** PROGRAM Operations with 4-Byte Address ********************************/
#define ADDR_4BYTE_PAGE_PROG_CMD             0x12U   /*!< Page Program 4 Byte Address                             */
#define ADDR_4BYTE_QUAD_INPUT_FAST_PROG_CMD  0x34U   /*!< Quad Input Fast Program 4 Byte Address                  */
#define ADDR_4BYTE_QUAD_INPUT_EXTENDED_FAST_PROG_CMD 0x3EU /*!< Quad Input Extended Fast Program 4 Byte Address  */

/***** ERASE Operations ******************************************************/
#define SUBSECTOR_ERASE_32K_CMD              0x52U   /*!< SubSector Erase 32KB 3/4 Byte Address                   */
#define SUBSECTOR_ERASE_4K_CMD               0x20U   /*!< SubSector Erase 4KB 3/4 Byte Address                    */
#define SECTOR_ERASE_64K_CMD                 0xD8U   /*!< Sector Erase 64KB 3/4 Byte Address                      */
#define BULK_ERASE_CMD                       0xC4U   /*!< Bulk Erase                                              */

/***** ERASE Operations with 4-Byte Address **********************************/
#define ADDR_4BYTE_SECTOR_ERASE_64K_CMD     0xDCU   /*!< Sector Erase 64KB 4 Byte address                        */
#define ADDR_4BYTE_SUBSECTOR_ERASE_4K_CMD   0x21U   /*!< SubSector Erase 4KB 4 Byte address                      */
#define ADDR_4BYTE_SUBSECTOR_ERASE_32K_CMD  0x5CU   /*!< SubSector Erase 32KB 4 Byte address                     */

/***** SUSPEND/RESUME Operations *********************************************/
#define PROG_ERASE_SUSPEND_CMD               0x75U   /*!< Program/Erase suspend                                   */
#define PROG_ERASE_RESUME_CMD                0x7AU   /*!< Program/Erase resume                                    */

/***** ONE-TIME PROGRAMMABLE (OTP) Operations ********************************/
#define READ_OTP_ARRAY_CMD                   0x48U   /*!< Read OTP Array 3/4 Byte Address                         */
#define PROGRAM_OTP_ARRAY_CMD                0x42U   /*!< Program OTP Array 3/4 Byte Address                      */

/***** 4-BYTE ADDRESS MODE Operations ****************************************/
#define ENTER_4_BYTE_ADDR_MODE_CMD           0xB7U   /*!< Enter 4-Byte mode (3/4 Byte address commands)           */
#define EXIT_4_BYTE_ADDR_MODE_CMD            0xE9U   /*!< Exit 4-Byte mode (3/4 Byte address commands)            */

/***** QUAD PROTOCOL Operations **********************************************/
#define ENABLE_QSPI_CMD                      0x35U   /*!< Enable QPI; Enter QPI                                   */
#define RESET_QSPI_CMD                       0xF5U   /*!< Reset QPI; Exit QPI                                     */

/***** DEEP POWER-DOWN Operations ********************************************/
#define ENTER_DEEP_POWER_DOWN_CMD            0xB9U   /*!< Enter deep power down                                   */
#define RELEASE_FROM_DEEP_POWER_DOWN_CMD     0xABU   /*!< Release from Deep Power down                            */

/***** ADVANCED SECTOR PROTECTION Operations *********************************/
#define READ_SECTOR_PROTECTION_CMD           0x2DU   /*!< Read Sector Protection                                  */
#define PROGRAM_SECTOR_PROTECTION_CMD        0x2CU   /*!< Program Sector Protection                               */
#define READ_VOL_LOCK_BITS_CMD               0xE8U   /*!< Read Volatile Lock Bits 3/4 Byte Address                */
#define WRITE_VOL_LOCK_BITS_CMD              0xE5U   /*!< Write Volatile Lock Bits 3/4 Byte Address               */
#define READ_NONVOL_LOCK_BITS_CMD            0xE2U   /*!< Read NonVolatile Lock Bits 4 Byte Address               */
#define WRITE_NONVOL_LOCK_BITS_CMD           0xE3U   /*!< Write NonVolatile Lock Bits 4 Byte Address              */
#define ERASE_NONVOL_LOCK_BITS_CMD           0xE4U   /*!< Erase NonVolatile Lock Bits                             */
#define READ_GLOBAL_FREEZE_BIT_CMD           0xA7U   /*!< Read Global Freeze Bit                                  */
#define WRITE_GLOBAL_FREEZE_BIT_CMD          0xA6U   /*!< Write Global Freeze Bit                                 */
#define READ_PASSWORD_REGISTER_CMD           0x27U   /*!< Read Password                                           */
#define WRITE_PASSWORD_REGISTER_CMD          0x28U   /*!< Write Password                                          */
#define PASSWORD_UNLOCK_CMD                  0x29U   /*!< Unlock Password                                         */

/***** ADVANCED SECTOR PROTECTION Operations with 4-Byte Address *************/
#define ADDR_4BYTE_READ_VOL_LOCK_BITS_CMD	 0xE0U   /*!< Read Volatile Lock Bits 4 Byte Address                  */
#define ADDR_4BYTE_WRITE_VOL_LOCK_BITS_CMD   0xE1U   /*!< Write Volatile Lock Bits 4 Byte Address                 */

/***** ADVANCED FUNCTION INTERFACE Operations ********************************/
#define INTERFACE_ACTIVATION_CMD             0x9BU   /*!< Interface Activation                                    */
#define CYCLIC_REDUNDANCY_CHECK_CMD          0x27U   /*!< Cyclic Redundancy Check                                 */                          */

/******************************************************************************
 * @brief MT25QL01GBBB Registers
 * Converted bit position to hex.
 * i.e Bit 5 in Status Register for Top/bottom. Binary: 0010 0000 => Hex: 0x20U
 *
 * References:
 * Table 3: Status Register
 * Table 5: Flag Status Register
 * Table 6: Extended Address Register
 * Table 7: Non-volatile Configuration Register
 * Table 8: Volatile Configuration Register
 * Table 14: Enhanced Volatile Configuration Register
 * Table 15: Sector Protection Register
 * Table 16: Global Freeze Bit
 * Table 18: Volatile Lock Bit Register
  ****************************************************************************/
/* Status Register */
#define MT25QL_SR_WIP                               0x01U   /*!< Write in progress                                       */
#define MT25QL_SR_WEN                               0x02U   /*!< Write enable latch                                      */
#define MT25QL_SR_BLOCKPR                           0x5CU   /*!< Block protected against program and erase operations    */
#define MT25QL_SR_TB                                0x20U   /*!< Top/Bottom bit to configure the block protect area      */
#define MT25QL_SR_SRWD                              0x80U   /*!< Status register write enable/disable. Used with W#      */

/* Flag Status Register */
#define MT25QL_FSR_4BYTE                            0x01U   /*!< 3-Bytes or 4-Bytes addressing                           */
#define MT25QL_FSR_PRE                              0x02U   /*!< Failure or Protection Error                             */
#define MT25QL_FSR_P_SUSPEND                        0x04U   /*!< Program Suspend                                         */
#define MT25QL_FSR_P_FAIL                           0x10U   /*!< Program Failure                                         */
#define MT25QL_FSR_E_FAIL                           0x20U   /*!< Erase Failure                                           */
#define MT25QL_FSR_E_SUSPEND                        0x40U   /*!< Erase Suspend                                           */
#define MT25QL_FSR_READY                            0x80U   /*!< Busy or Ready                                           */

/* Extended Address Register */
#define MT25QL_EAR_128MBS                           0x03U   /*!< A[25:24] bits; 128Mb segment selection                  */

/* Nonvolatile Configuration Register */
#define MT25QL_NVCR_4BYTE                         0x0001U   /*!< 3-Bytes or 4-Bytes addressing                           */
#define MT25QL_NVCR_128MBS                        0x0002U   /*!< Highest or Lowest default 128Mb segment                 */
#define MT25QL_NVCR_DPI                           0x0004U   /*!< Dual IO command input mode                              */
#define MT25QL_NVCR_QPI                           0x0008U   /*!< Quad IO command input mode                              */
#define MT25QL_NVCR_RH                            0x0010U   /*!< Enable #HOLD or #RESET on DQ3                           */
#define MT25QL_NVCR_DTR                           0x0020U   /*!< Double Transfer Rate Protocol                           */
#define MT25QL_NVCR_ODS                           0x01C0U   /*!< Output driver strength                                  */
#define MT25QL_NVCR_XIP                           0x0E00U   /*!< XIP mode at power-on reset                              */
#define MT25QL_NVCR_DC                            0xF000U   /*!< Dummy Clock Cycles setting for FAST READ commands       */

/* Volatile Configuration Register */
#define MT25QL_VCR_WRAP                             0x03U   /*!< Wrap                                                    */
#define MT25QL_VCR_XIP                              0x08U   /*!< XIP                                                     */
#define MT25QL_VCR_DC                               0xF0U   /*!< Dummy Clock Cycles setting for FAST READ commands       */

/* Enhanced Volatile Configuration Register */
#define MT25QL_EVCR_ODS                             0x07U   /*!< Output driver strength                                  */
#define MT25QL_EVCR_RH                              0x10U   /*!< Enable #HOLD or #RESET on DQ3                           */
#define MT25QL_EVCR_DTR                             0x20U   /*!< Double Transfer Rate Protocol                           */
#define MT25QL_EVCR_DPI                             0x40U   /*!< Dual IO command input mode                              */
#define MT25QL_EVCR_QPI                             0x80U   /*!< Quad IO command input mode                              */

/* Sector Protection Security Register */
#define MT25QL_SPSR_SPL                           0x0002U   /*!< Sector Protection lock with or without password         */
#define MT25QL_SPSR_PPL                           0x0004U   /*!< Password Protection lock                                */

/* Global Freeze Register */
#define MT25QL_GFR_GFB                              0x01U   /*!< Global Freeze bit                                       */

/* Volatile Lock Bit Security Register */
#define MT25QL_VLBSR_SWL                            0x01U   /*!< Sector write lock                                       */
#define MT25QL_VLBSR_SLD                            0x02U   /*!< Sector lock down                                        */

// Add additional FM chips here
typedef enum{
	MT25QL_FM1 = 0,
	MT25QL_FM2
}MT25QL_FM_NO_t;

typedef enum{
	MT25QL_ERASE_4K = 0,
	MT25QL_ERASE_32K,
	MT25QL_ERASE_64K,
	MT25QL_ERASE_CHIP,
}MT25QL_Erase_t;

typedef enum{
	MT25QL_OK = 0,			//No error
	MT25QL_ERROR = -1,		//Error
	MT25QL_READY = 0		//Ready for program/erase
}MT25QL_Status_t;



//Basic command functions
MT25QL_Status_t MT25QL_Reset(SPI_HandleTypeDef *hspi, uint8_t FM);
uint32_t MT25QL_ReadID(SPI_HandleTypeDef *hspi, uint8_t FM);

//Status register functions
uint8_t MT25QL_ReadSR(SPI_HandleTypeDef *hspi, uint8_t FM);
uint8_t MT25QL_ReadFSR(SPI_HandleTypeDef *hspi, uint8_t FM);
uint16_t MT25QL_ReadNONVOL(SPI_HandleTypeDef *hspi, uint8_t FM);
uint8_t MT25QL_ReadVOL(SPI_HandleTypeDef *hspi, uint8_t FM);
uint8_t MT25QL_ReadXADDR(SPI_HandleTypeDef *hspi, uint8_t FM);
MT25QL_Status_t MT25QL_WriteSR(SPI_HandleTypeDef *hspi, uint8_t FM, uint8_t reg_val);
MT25QL_Status_t MT25QL_ClearFSR(SPI_HandleTypeDef *hspi, uint8_t FM);

//Read address functions
MT25QL_Status_t MT25QL_Read(SPI_HandleTypeDef *hspi, uint8_t FM, uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData);
MT25QL_Status_t MT25QL_ReadAddr(SPI_HandleTypeDef *hspi, uint8_t FM, uint32_t address, uint32_t size, uint8_t *rData);
MT25QL_Status_t MT25QL_FastRead(SPI_HandleTypeDef *hspi, uint8_t FM, uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData);
MT25QL_Status_t MT25QL_FastReadAddr(SPI_HandleTypeDef *hspi, uint8_t FM, uint32_t address, uint32_t size, uint8_t *rData);
MT25QL_Status_t MT25QL_4BRead(SPI_HandleTypeDef *hspi, uint8_t FM, uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData);
MT25QL_Status_t MT25QL_4BReadAddr(SPI_HandleTypeDef *hspi, uint8_t FM, uint32_t address, uint32_t size, uint8_t *rData);
MT25QL_Status_t MT25QL_4BFastRead(SPI_HandleTypeDef *hspi, uint8_t FM, uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData);
MT25QL_Status_t MT25QL_4BFastReadAddr(SPI_HandleTypeDef *hspi, uint8_t FM, uint32_t address, uint32_t size, uint8_t *rData);

//Program functions
MT25QL_Status_t MT25QL_PageProgram(SPI_HandleTypeDef *hspi, uint8_t FM, uint32_t page, uint16_t offset, uint32_t size, uint8_t *data);
MT25QL_Status_t MT25QL_4BPageProgram(SPI_HandleTypeDef *hspi, uint8_t FM, uint32_t page, uint16_t offset, uint32_t size, uint8_t *data);

//4-byte addressing enable/disable
MT25QL_Status_t MT25QL_4BAddrEnable(SPI_HandleTypeDef *hspi, uint8_t FM);
MT25QL_Status_t MT25QL_4BAddrDisable(SPI_HandleTypeDef *hspi, uint8_t FM);

//Block erase function
MT25QL_Status_t MT25QL_BlockErase(SPI_HandleTypeDef *hspi, MT25QL_FM_NO_t FM, uint32_t page, MT25QL_Erase_t BlockErase);

MT25QL_Status_t MT25QL_Init(SPI_HandleTypeDef *hspi, MT25QL_FM_NO_t FM);

#endif /* INC_MT25QLXX_H_ */
