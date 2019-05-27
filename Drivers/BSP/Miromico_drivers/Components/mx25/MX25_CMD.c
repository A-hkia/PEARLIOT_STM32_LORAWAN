/*
 * COPYRIGHT (c) 2010-2017 MACRONIX INTERNATIONAL CO., LTD
 * SPI Flash Low Level Driver (LLD) Sample Code
 *
 * SPI interface command set
 *
 * $Id: MX25_CMD.c,v 1.32 2015/12/15 06:29:01 mxclldb1 Exp $
 */

#include "MX25_CMD.h"
#include "hw_gpio.h"

/* internal functions */
static void MX25_CSHigh();
static void MX25_CSLow();
static void InsertDummyCycle(uint8_t dummy_cycle);
static void SendByte(uint8_t byte_value);
static uint8_t GetByte();
static bool IsFlash4Byte(void);
static void SendFlashAddr(uint32_t flash_address, bool addr_mode);
static bool WaitFlashReady(uint32_t ExpectTime);

static void* spiHandle = 0;
#define HW_SPI_Transmit HAL_SPI_Transmit
#define HW_SPI_Receive HAL_SPI_Receive

/*
 * Function:       Initial_Spi
 * Arguments:      None
 * Description:    Initial spi flash state and wait flash warm-up
 *                 (enable read/write).
 * Return Message: None
 */
void MX25_Init(void* handle)
{
  spiHandle = handle;

  GPIO_InitTypeDef  GPIO_InitStruct;
// SPI has already been initialized
// Set CSS as GPIO

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  HW_GPIO_Init(SPI2_NSS_PORT, SPI2_NSS_PIN, &GPIO_InitStruct);
  MX25_CSHigh();
  // Wait flash warm-up
  HAL_Delay(tPUW);
}

/*
 * Function:       CS_Low, CS_High
 * Arguments:      None.
 * Description:    Chip select go low / high.
 * Return Message: None.
 */
static void MX25_CSLow()
{
  HW_GPIO_Write(SPI2_NSS_PORT, SPI2_NSS_PIN, GPIO_PIN_RESET);
}

static void MX25_CSHigh()
{
  HW_GPIO_Write(SPI2_NSS_PORT, SPI2_NSS_PIN, GPIO_PIN_SET);
}

/*
 * Function:       InsertDummyCycle
 * Arguments:      dummy_cycle, number of dummy clock cycle
 * Description:    Insert dummy cycle of SCLK
 * Return Message: None.
 */
static void InsertDummyCycle(uint8_t dummy_cycle)
{
  uint8_t gna[4] = { 0 };

  if (dummy_cycle % 8 != 0) {
    dummy_cycle = (dummy_cycle / 8) + 1;
  } else {
    dummy_cycle = dummy_cycle / 8;
  }
  HW_SPI_Transmit(spiHandle, gna, dummy_cycle, 1000);
}

/*
 * Function:       SendByte
 * Arguments:      byte_value, data transfer to flash
 * Description:    Send one byte data to flash
 * Return Message: None.
 */
static void SendByte(uint8_t byte_value)
{
  HW_SPI_Transmit(spiHandle, &byte_value, 1, 1000);
}
/*
 * Function:       GetByte
 * Arguments:      byte_value, data receive from flash
 * Description:    Get one byte data to flash
 * Return Message: 8 bit data
 */
static uint8_t GetByte()
{
  uint8_t data_buf;
  HW_SPI_Receive(spiHandle, &data_buf, 1, 1000);
  return data_buf;
}

/*
 * Function:       WaitFlashReady
 * Arguments:      ExpectTime, expected time-out value of flash operations.
 *                 No use at non-synchronous IO mode.
 * Description:    Synchronous IO:
 *                 If flash is ready return true.
 *                 If flash is time-out return false.
 *                 Non-synchronous IO:
 *                 Always return true
 * Return Message: true, false
 */
static bool WaitFlashReady(uint32_t ExpectTime)
{
#ifndef NON_SYNCHRONOUS_IO
  uint32_t tickstart = HAL_GetTick();
  while (MX25_IsFlashBusy()) {
    if ((HAL_GetTick() - tickstart) >= ExpectTime) {
      return false;
    }
  }
  return true;
#else
  return true;
#endif
}

bool MX25_WaitFlashReady(uint32_t timeout)
{
  uint32_t tickstart = HAL_GetTick();
  while (MX25_IsFlashBusy()) {
    if ((HAL_GetTick() - tickstart) >= timeout) {
      return false;
    }
  }
  return true;
}

///*
// * Function:       WaitRYBYReady
// * Arguments:      ExpectTime, expected time-out value of flash operations.
// *                 No use at non-synchronous IO mode.
// * Description:    Synchronous IO:
// *                 If flash is ready return true.
// *                 If flash is time-out return false.
// *                 Non-synchronous IO:
// *                 Always return true
// * Return Message: true, false
// */
//bool WaitRYBYReady( uint32_t ExpectTime )
//{
//#ifndef NON_SYNCHRONOUS_IO
//    uint32_t temp = 0;
//#ifdef GPIO_SPI
//    while( SO == 0 )
//#else
//    // Insert your code for waiting RYBY (SO) pin ready
//#endif
//    {
//        if( temp > ExpectTime )
//        {
//            return false;
//        }
//        temp = temp + 1;
//    }
//    return true;
//
//#else
//    return true;
//#endif
//}

/*
 * Function:       IsFlashBusy
 * Arguments:      None.
 * Description:    Check status register WIP bit.
 *                 If  WIP bit = 1: return true ( Busy )
 *                             = 0: return false ( Ready ).
 * Return Message: true, false
 */
bool MX25_IsFlashBusy(void)
{
  uint8_t gDataBuffer;

  MX25_CmdRDSR(&gDataBuffer);
  if ((gDataBuffer & FLASH_WIP_MASK) == FLASH_WIP_MASK) {
    return true;
  } else {
    return false;
  }
}

///*
// * Function:       IsFlashQIO
// * Arguments:      None.
// * Description:    If flash QE bit = 1: return true
// *                                 = 0: return false.
// * Return Message: true, false
// */
//bool IsFlashQIO(void)
//{
//#ifdef FLASH_NO_QE_BIT
//    return true;
//#else
//    uint8_t  gDataBuffer;
//    MX25_CmdRDSR( &gDataBuffer );
//    if( (gDataBuffer & FLASH_QE_MASK) == FLASH_QE_MASK )
//        return true;
//    else
//        return false;
//#endif
//}
/*
 * Function:       IsFlash4Byte
 * Arguments:      None
 * Description:    Check flash address is 3-byte or 4-byte.
 *                 If flash 4BYTE bit = 1: return true
 *                                    = 0: return false.
 * Return Message: true, false
 */
static bool IsFlash4Byte(void)
{
#ifdef FLASH_CMD_RDSCUR
#ifdef FLASH_4BYTE_ONLY
  return true;
#elif FLASH_3BYTE_ONLY
  return false;
#else
  uint8_t  gDataBuffer;
  MX25_CmdRDSCUR(&gDataBuffer);
  if ((gDataBuffer & FLASH_4BYTE_MASK) == FLASH_4BYTE_MASK) {
    return true;
  } else {
    return false;
  }
#endif
#else
  return false;
#endif
}

/*
 * Function:       SendFlashAddr
 * Arguments:      flash_address, 32 bit flash memory address
 *                 io_mode, I/O mode to transfer address
 *                 addr_4byte_mode,
 * Description:    Send flash address with 3-byte or 4-byte mode.
 * Return Message: None
 */
static void SendFlashAddr(uint32_t flash_address, bool addr_4byte_mode)
{
  /* Check flash is 3-byte or 4-byte mode.
     4-byte mode: Send 4-byte address (A31-A0)
     3-byte mode: Send 3-byte address (A23-A0) */
  if (addr_4byte_mode == true) {
    SendByte((flash_address >> 24));  // A31-A24
  }
  /* A23-A0 */
  SendByte((flash_address >> 16));
  SendByte((flash_address >> 8));
  SendByte((flash_address));
}

///*
// * Function:       GetDummyCycle
// * Arguments:      default_cycle, default dummy cycle
// *                 fsptr, pointer of flash status structure
// * Description:    Get dummy cycle for different condition
// *                 default_cycle: Byte3 | Byte2 | Byte1 | Byte0
// *                      DC 1 bit:   x       1       x       0
// *                      DC 2 bit:   11      10      01      00
// *                 Note: the variable dummy cycle only support
//                         in some product.
// * Return Message: Dummy cycle value
// */
//uint8_t GetDummyCycle( uint32_t default_cycle )
//{
//#ifdef FLASH_CMD_RDCR
//    uint8_t gDataBuffer;
//    uint8_t dummy_cycle = default_cycle;
//    MX25_CmdRDCR( &gDataBuffer );
//    #ifdef SUPPORT_CR_DC
//        // product support 1-bit dummy cycle configuration
//        if( (gDataBuffer & FLASH_DC_MASK) == FLASH_DC_MASK )
//            dummy_cycle = default_cycle >> 16;
//        else
//            dummy_cycle = default_cycle;
//    #elif SUPPORT_CR_DC_2bit
//        // product support 2-bit dummy cycle configuration
//        switch( gDataBuffer & FLASH_DC_2BIT_MASK ){
//            case 0x00:
//                dummy_cycle = default_cycle;
//            break;
//            case 0x40:
//                dummy_cycle = default_cycle >> 8;
//            break;
//            case 0x80:
//                dummy_cycle = default_cycle >> 16;
//            break;
//            case 0xC0:
//                dummy_cycle = default_cycle >> 24;
//            break;
//        }
//    #else
//         // configuration register not support dummy configuration
//         dummy_cycle = default_cycle;
//    #endif
//    return dummy_cycle;
//#else
//    // default case: return default dummy cycle
//    return default_cycle;
//#endif
//}

/*
 * ID Command
 */

/*
 * Function:       MX25_CmdRDID
 * Arguments:      Identification, 32 bit buffer to store id
 * Description:    The RDID instruction is to read the manufacturer ID
 *                 of 1-byte and followed by Device ID of 2-byte.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg MX25_CmdRDID(uint32_t* Identification)
{
  uint32_t temp;
  uint8_t  gDataBuffer[3];

  // Chip select go low to start a flash command
  MX25_CSLow();

  // Send command
  SendByte(FLASH_CMD_RDID);

  // Get manufacturer identification, device identification
  gDataBuffer[0] = GetByte();
  gDataBuffer[1] = GetByte();
  gDataBuffer[2] = GetByte();

  // Chip select go high to end a command
  MX25_CSHigh();

  // Store identification
  temp =  gDataBuffer[0];
  temp = (temp << 8) | gDataBuffer[1];
  *Identification = (temp << 8) | gDataBuffer[2];

  return FlashOperationSuccess;
}

/*
 * Function:       MX25_CmdRES
 * Arguments:      ElectricIdentification, 8 bit buffer to store electric id
 * Description:    The RES instruction is to read the Device
 *                 electric identification of 1-byte.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg MX25_CmdRES(uint8_t* ElectricIdentification)
{

  // Chip select go low to start a flash command
  MX25_CSLow();

  // Send flash command and insert dummy cycle
  SendByte(FLASH_CMD_RES);
  InsertDummyCycle(24);

  // Get electric identification
  *ElectricIdentification = GetByte();

  // Chip select go high to end a flash command
  MX25_CSHigh();

  return FlashOperationSuccess;
}

/*
 * Function:       MX25_CmdREMS
 * Arguments:      REMS_Identification, 16 bit buffer to store id
 *                 fsptr, pointer of flash status structure
 * Description:    The REMS instruction is to read the Device
 *                 manufacturer ID and electric ID of 1-byte.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg MX25_CmdREMS(uint16_t* REMS_Identification, FlashStatus* fsptr)
{
  uint8_t  gDataBuffer[2];

  // Chip select go low to start a flash command
  MX25_CSLow();

  // Send flash command and insert dummy cycle ( if need )
  // ArrangeOpt = 0x00 will output the manufacturer's ID first
  //            = 0x01 will output electric ID first
  SendByte(FLASH_CMD_REMS);
  InsertDummyCycle(16);
  SendByte(fsptr->ArrangeOpt);

  // Get ID
  gDataBuffer[0] = GetByte();
  gDataBuffer[1] = GetByte();

  // Store identification information
  *REMS_Identification = (gDataBuffer[0] << 8) | gDataBuffer[1];

  // Chip select go high to end a flash command
  MX25_CSHigh();

  return FlashOperationSuccess;
}

/*
 * Register  Command
 */

/*
 * Function:       MX25_CmdRDSR
 * Arguments:      StatusReg, 8 bit buffer to store status register value
 * Description:    The RDSR instruction is for reading Status Register Bits.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg MX25_CmdRDSR(uint8_t* StatusReg)
{
  uint8_t  gDataBuffer;

  // Chip select go low to start a flash command
  MX25_CSLow();

  // Send command
  SendByte(FLASH_CMD_RDSR);
  gDataBuffer = GetByte();

  // Chip select go high to end a flash command
  MX25_CSHigh();

  *StatusReg = gDataBuffer;

  return FlashOperationSuccess;
}

///*
// * Function:       MX25_CmdWRSR
// * Arguments:      UpdateValue, 8/16 bit status register value to updata
// * Description:    The WRSR instruction is for changing the values of
// *                 Status Register Bits (and configuration register)
// * Return Message: FlashIsBusy, FlashTimeOut, FlashOperationSuccess
// */
//#ifdef SUPPORT_WRSR_CR
//ReturnMsg MX25_CmdWRSR( uint16_t UpdateValue )
//#else
//ReturnMsg MX25_CmdWRSR( uint8_t UpdateValue )
//#endif
//{
//    // Check flash is busy or not
//    if( IsFlashBusy() )    return FlashIsBusy;
//
//    // Setting Write Enable Latch bit
//    MX25_CmdWREN();
//
//    // Chip select go low to start a flash command
//    MX25_CSLow();
//
//    // Send command and update value
//    SendByte( FLASH_CMD_WRSR);
//    SendByte( UpdateValue);
//#ifdef SUPPORT_WRSR_CR
//    SendByte( UpdateValue >> 8);    // write configuration register
//#endif
//
//    // Chip select go high to end a flash command
//    MX25_CSHigh();
//
//
//    if( WaitFlashReady( WriteStatusRegCycleTime ) )
//        return FlashOperationSuccess;
//    else
//        return FlashTimeOut;
//
//}

/*
 * Function:       MX25_CmdRDSCUR
 * Arguments:      SecurityReg, 8 bit buffer to store security register value
 * Description:    The RDSCUR instruction is for reading the value of
 *                 Security Register bits.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg MX25_CmdRDSCUR(uint8_t* SecurityReg)
{
  uint8_t  gDataBuffer;

  // Chip select go low to start a flash command
  MX25_CSLow();

  //Send command
  SendByte(FLASH_CMD_RDSCUR);
  gDataBuffer = GetByte();

  // Chip select go high to end a flash command
  MX25_CSHigh();

  *SecurityReg = gDataBuffer;

  return FlashOperationSuccess;
}
//
///*
// * Function:       MX25_CmdWRSCUR
// * Arguments:      None.
// * Description:    The WRSCUR instruction is for changing the values of
// *                 Security Register Bits.
// * Return Message: FlashIsBusy, FlashOperationSuccess, FlashWriteRegFailed,
// *                 FlashTimeOut
// */
//ReturnMsg MX25_CmdWRSCUR( void )
//{
//    uint8_t  gDataBuffer;
//
//    // Check flash is busy or not
//    if( IsFlashBusy() )    return FlashIsBusy;
//
//    // Setting Write Enable Latch bit
//    MX25_CmdWREN();
//
//    // Chip select go low to start a flash command
//    MX25_CSLow();
//
//    // Write WRSCUR command
//    SendByte( FLASH_CMD_WRSCUR);
//
//    // Chip select go high to end a flash command
//    MX25_CSHigh();
//
//    if( WaitFlashReady( WriteSecuRegCycleTime ) ){
//
//        MX25_CmdRDSCUR( &gDataBuffer );
//
//        // Check security register LDSO bit
//        if( (gDataBuffer & FLASH_LDSO_MASK) == FLASH_LDSO_MASK )
//                return FlashOperationSuccess;
//        else
//                return FlashWriteRegFailed;
//    }
//    else
//        return FlashTimeOut;
//
//}
//
///*
// * Function:       MX25_CmdRDCR
// * Arguments:      ConfigReg, 8 bit buffer to store Configuration register value
// * Description:    The RDCR instruction is for reading Configuration Register Bits.
// * Return Message: FlashOperationSuccess
// */
//ReturnMsg MX25_CmdRDCR( uint8_t *ConfigReg )
//{
//    uint8_t  gDataBuffer;
//
//    // Chip select go low to start a flash command
//    MX25_CSLow();
//
//    // Send command
//    SendByte( FLASH_CMD_RDCR);
//    gDataBuffer = GetByte();
//
//    // Chip select go high to end a flash command
//    MX25_CSHigh();
//
//    *ConfigReg = gDataBuffer;
//
//    return FlashOperationSuccess;
//}
//
//
///*
// * Read Command
// */

/*
 * Function:       MX25_CmdREAD
 * Arguments:      flash_address, 32 bit flash memory address
 *                 target_address, buffer address to store returned data
 *                 byte_length, length of returned data in byte unit
 * Description:    The READ instruction is for reading data out.
 * Return Message: FlashAddressInvalid, FlashOperationSuccess
 */
ReturnMsg MX25_CmdREAD(uint32_t flash_address, uint8_t* target_address, uint32_t byte_length)
{
  uint32_t index;
  uint8_t  addr_4byte_mode;

  // Check flash address
  if (flash_address > FlashSize) { return FlashAddressInvalid; }

  // Check 3-byte or 4-byte mode
  if (IsFlash4Byte()) {
    addr_4byte_mode = true;  // 4-byte mode
  } else {
    addr_4byte_mode = false;  // 3-byte mode
  }

  // Chip select go low to start a flash command
  MX25_CSLow();

  // Write READ command and address
  SendByte(FLASH_CMD_READ);
  SendFlashAddr(flash_address, addr_4byte_mode);

  // Set a loop to read data into buffer
  for (index = 0; index < byte_length; index++) {
    // Read data one byte at a time
    *(target_address + index) = GetByte();
  }

  // Chip select go high to end a flash command
  MX25_CSHigh();

  return FlashOperationSuccess;
}

///*
// * Function:       MX25_CmdFASTREAD
// * Arguments:      flash_address, 32 bit flash memory address
// *                 target_address, buffer address to store returned data
// *                 byte_length, length of returned data in byte unit
// * Description:    The FASTREAD instruction is for quickly reading data out.
// * Return Message: FlashAddressInvalid, FlashOperationSuccess
// */
//ReturnMsg MX25_CmdFASTREAD( uint32_t flash_address, uint8_t *target_address, uint32_t byte_length )
//{
//    uint32_t index;
//    uint8_t  addr_4byte_mode;
//    uint8_t  dc;
//
//    // Check flash address
//    if( flash_address > FlashSize ) return FlashAddressInvalid;
//
//    // Check 3-byte or 4-byte mode
//    if( IsFlash4Byte() )
//        addr_4byte_mode = true;  // 4-byte mode
//    else
//        addr_4byte_mode = false; // 3-byte mode
//
//    dc = GetDummyCycle( DUMMY_CONF_FASTREAD );
//
//    // Chip select go low to start a flash command
//    MX25_CSLow();
//
//    // Write Fast Read command, address and dummy cycle
//    SendByte( FLASH_CMD_FASTREAD);
//    SendFlashAddr( flash_address, addr_4byte_mode );
//    InsertDummyCycle ( dc );          // Wait dummy cycle
//
//    // Set a loop to read data into data buffer
//    for( index=0; index < byte_length; index++ )
//    {
//        *(target_address + index) = GetByte();
//    }
//
//    // Chip select go high to end a flash command
//    MX25_CSHigh();
//
//    return FlashOperationSuccess;
//}
//
//
///*
// * Function:       MX25_CmdRDSFDP
// * Arguments:      flash_address, 32 bit flash memory address
// *                 target_address, buffer address to store returned data
// *                 byte_length, length of returned data in byte unit
// * Description:    RDSFDP can retrieve the operating characteristics, structure
// *                 and vendor-specified information such as identifying information,
// *                 memory size, operating voltages and timinginformation of device
// * Return Message: FlashAddressInvalid, FlashOperationSuccess
// */
//ReturnMsg MX25_CmdRDSFDP( uint32_t flash_address, uint8_t *target_address, uint32_t byte_length )
//{
//    uint32_t index;
//    uint8_t  addr_4byte_mode;
//
//    // Check flash address
//    if( flash_address > FlashSize ) return FlashAddressInvalid;
//
//    // Check 3-byte or 4-byte mode
//    if( IsFlash4Byte() )
//        addr_4byte_mode = true;  // 4-byte mode
//    else
//        addr_4byte_mode = false; // 3-byte mode
//
//    // Chip select go low to start a flash command
//    MX25_CSLow();
//
//    // Write Read SFDP command
//    SendByte( FLASH_CMD_RDSFDP);
//    SendFlashAddr( flash_address, addr_4byte_mode );
//    InsertDummyCycle ( 8 );        // Insert dummy cycle
//
//    // Set a loop to read data into data buffer
//    for( index=0; index < byte_length; index++ )
//    {
//        *(target_address + index) = GetByte();
//    }
//
//    // Chip select go high to end a flash command
//    MX25_CSHigh();
//
//    return FlashOperationSuccess;
//}
///*
// * Program Command
// */

/*
 * Function:       MX25_CmdWREN
 * Arguments:      None.
 * Description:    The WREN instruction is for setting
 *                 Write Enable Latch (WEL) bit.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg MX25_CmdWREN(void)
{
  // Chip select go low to start a flash command
  MX25_CSLow();

  // Write Enable command = 0x06, Setting Write Enable Latch Bit
  SendByte(FLASH_CMD_WREN);

  // Chip select go high to end a flash command
  MX25_CSHigh();

  return FlashOperationSuccess;
}

///*
// * Function:       MX25_CmdWRDI
// * Arguments:      None.
// * Description:    The WRDI instruction is to reset
// *                 Write Enable Latch (WEL) bit.
// * Return Message: FlashOperationSuccess
// */
//ReturnMsg MX25_CmdWRDI( void )
//{
//    // Chip select go low to start a flash command
//    MX25_CSLow();
//
//    // Write Disable command = 0x04, resets Write Enable Latch Bit
//    SendByte( FLASH_CMD_WRDI);
//
//    MX25_CSHigh();
//
//    return FlashOperationSuccess;
//}
//

/*
 * Function:       MX25_CmdPP
 * Arguments:      flash_address, 32 bit flash memory address
 *                 source_address, buffer address of source data to program
 *                 byte_length, byte length of data to programm
 * Description:    The PP instruction is for programming
 *                 the memory to be "0".
 *                 The device only accept the last 256 byte to program.
 *                 If the page address ( flash_address[7:0] ) reach 0xFF, it will
 *                 program next at 0x00 of the same page.
 * Return Message: FlashAddressInvalid, FlashIsBusy, FlashOperationSuccess,
 *                 FlashTimeOut
 */
ReturnMsg MX25_CmdPP(uint32_t flash_address, uint8_t* source_address, uint16_t byte_length)
{
  uint32_t index;
  uint8_t  addr_4byte_mode;

  // Check flash address
  if (flash_address > FlashSize) { return FlashAddressInvalid; }

  // Check flash is busy or not
  if (MX25_IsFlashBusy()) { return FlashIsBusy; }

  // Check 3-byte or 4-byte mode
  if (IsFlash4Byte()) {
    addr_4byte_mode = true;  // 4-byte mode
  } else {
    addr_4byte_mode = false;  // 3-byte mode
  }

  // Setting Write Enable Latch bit
  MX25_CmdWREN();

  // Chip select go low to start a flash command
  MX25_CSLow();

  // Write Page Program command
  SendByte(FLASH_CMD_PP);
  SendFlashAddr(flash_address, addr_4byte_mode);

  // Set a loop to down load whole page data into flash's buffer
  // Note: only last 256 byte ( or 32 byte ) will be programmed
  for (index = 0; index < byte_length; index++) {
    SendByte(*(source_address + index));
  }

  // Chip select go high to end a flash command
  MX25_CSHigh();

  if (WaitFlashReady(PageProgramCycleTime)) {
    return FlashOperationSuccess;
  } else {
    return FlashTimeOut;
  }
}

/*
 * Function:       MX25_CmdSE
 * Arguments:      flash_address, 32 bit flash memory address
 * Description:    The SE instruction is for erasing the data
 *                 of the chosen sector (4KB) to be "1".
 * Return Message: FlashAddressInvalid, FlashIsBusy, FlashOperationSuccess,
 *                 FlashTimeOut
 */
ReturnMsg MX25_CmdSE(uint32_t flash_address)
{
  uint8_t  addr_4byte_mode;

  // Check flash address
  if (flash_address > FlashSize) { return FlashAddressInvalid; }

  // Check flash is busy or not
  if (MX25_IsFlashBusy()) { return FlashIsBusy; }

  // Check 3-byte or 4-byte mode
  if (IsFlash4Byte()) {
    addr_4byte_mode = true;  // 4-byte mode
  } else {
    addr_4byte_mode = false;  // 3-byte mode
  }

  // Setting Write Enable Latch bit
  MX25_CmdWREN();

  // Chip select go low to start a flash command
  MX25_CSLow();

  //Write Sector Erase command = 0x20;
  SendByte(FLASH_CMD_SE);
  SendFlashAddr(flash_address, addr_4byte_mode);

  // Chip select go high to end a flash command
  MX25_CSHigh();

  if (WaitFlashReady(SectorEraseCycleTime)) {
    return FlashOperationSuccess;
  } else {
    return FlashTimeOut;
  }
}

/*
 * Function:       MX25_CmdBE32K
 * Arguments:      flash_address, 32 bit flash memory address
 * Description:    The BE32K instruction is for erasing the data
 *                 of the chosen sector (32KB) to be "1".
 * Return Message: FlashAddressInvalid, FlashIsBusy, FlashOperationSuccess,
 *                 FlashTimeOut
 */
ReturnMsg MX25_CmdBE32K(uint32_t flash_address)
{
  uint8_t  addr_4byte_mode;

  // Check flash address
  if (flash_address > FlashSize) { return FlashAddressInvalid; }

  // Check flash is busy or not
  if (MX25_IsFlashBusy()) { return FlashIsBusy; }

  // Check 3-byte or 4-byte mode
  if (IsFlash4Byte()) {
    addr_4byte_mode = true;  // 4-byte mode
  } else {
    addr_4byte_mode = false;  // 3-byte mode
  }

  // Setting Write Enable Latch bit
  MX25_CmdWREN();

  // Chip select go low to start a flash command
  MX25_CSLow();

  //Write Block Erase32KB command;
  SendByte(FLASH_CMD_BE32K);
  SendFlashAddr(flash_address, addr_4byte_mode);

  // Chip select go high to end a flash command
  MX25_CSHigh();

  if (WaitFlashReady(BlockErase32KCycleTime)) {
    return FlashOperationSuccess;
  } else {
    return FlashTimeOut;
  }
}

/*
 * Function:       MX25_CmdBE
 * Arguments:      flash_address, 32 bit flash memory address
 * Description:    The BE instruction is for erasing the data
 *                 of the chosen sector (64KB) to be "1".
 * Return Message: FlashAddressInvalid, FlashIsBusy, FlashOperationSuccess,
 *                 FlashTimeOut
 */
ReturnMsg MX25_CmdBE(uint32_t flash_address)
{
  uint8_t  addr_4byte_mode;

  // Check flash address
  if (flash_address > FlashSize) { return FlashAddressInvalid; }

  // Check flash is busy or not
  if (MX25_IsFlashBusy()) { return FlashIsBusy; }

  // Check 3-byte or 4-byte mode
  if (IsFlash4Byte()) {
    addr_4byte_mode = true;  // 4-byte mode
  } else {
    addr_4byte_mode = false;  // 3-byte mode
  }

  // Setting Write Enable Latch bit
  MX25_CmdWREN();

  // Chip select go low to start a flash command
  MX25_CSLow();

  //Write Block Erase command = 0xD8;
  SendByte(FLASH_CMD_BE);
  SendFlashAddr(flash_address, addr_4byte_mode);

  // Chip select go high to end a flash command
  MX25_CSHigh();

  if (WaitFlashReady(BlockEraseCycleTime)) {
    return FlashOperationSuccess;
  } else {
    return FlashTimeOut;
  }
}

/*
 * Function:       MX25_CmdCE
 * Arguments:      None.
 * Description:    The CE instruction is for erasing the data
 *                 of the whole chip to be "1".
 * Return Message: FlashIsBusy, FlashOperationSuccess, FlashTimeOut
 */
ReturnMsg MX25_CmdCE(void)
{
  // Check flash is busy or not
  if (MX25_IsFlashBusy()) { return FlashIsBusy; }

  // Setting Write Enable Latch bit
  MX25_CmdWREN();

  // Chip select go low to start a flash command
  MX25_CSLow();

  //Write Chip Erase command = 0x60;
  SendByte(FLASH_CMD_CE);

  // Chip select go high to end a flash command
  MX25_CSHigh();

  if (WaitFlashReady(ChipEraseCycleTime)) {
    return FlashOperationSuccess;
  } else {
    return FlashTimeOut;
  }
}

/*
 * Mode setting Command
 */

/*
 * Function:       MX25_CmdDP
 * Arguments:      None.
 * Description:    The DP instruction is for setting the
 *                 device on the minimizing the power consumption.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg MX25_CmdDP(void)
{
  // Chip select go low to start a flash command
  MX25_CSLow();

  // Deep Power Down Mode command
  SendByte(FLASH_CMD_DP);

  // Chip select go high to end a flash command
  MX25_CSHigh();

  return FlashOperationSuccess;
}

///*
// * Function:       MX25_CmdENSO
// * Arguments:      None.
// * Description:    The ENSO instruction is for entering the secured OTP mode.
// * Return Message: FlashOperationSuccess
// */
//ReturnMsg MX25_CmdENSO( void )
//{
//    // Chip select go low to start a flash command
//    MX25_CSLow();
//
//    // Write ENSO command
//    SendByte( FLASH_CMD_ENSO);
//
//    // Chip select go high to end a flash command
//    MX25_CSHigh();
//
//    return FlashOperationSuccess;
//}
//
///*
// * Function:       MX25_CmdEXSO
// * Arguments:      None.
// * Description:    The EXSO instruction is for exiting the secured OTP mode.
// * Return Message: FlashOperationSuccess
// */
//ReturnMsg MX25_CmdEXSO( void )
//{
//    // Chip select go low to start a flash command
//    MX25_CSLow();
//
//    // Write EXSO command = 0xC1
//    SendByte( FLASH_CMD_EXSO);
//
//    // Chip select go high to end a flash command
//    MX25_CSHigh();
//
//    return FlashOperationSuccess;
//}
//
//
///*
// * Function:       MX25_CmdSBL
// * Arguments:      burstconfig, burst length configuration
// * Description:    To set the Burst length
// * Return Message: FlashOperationSuccess
// */
//ReturnMsg MX25_CmdSBL( uint8_t burstconfig )
//{
//    // Chip select go low to start a flash command
//    MX25_CSLow();
//
//    // Send SBL command and config data
//    SendByte( FLASH_CMD_SBL);
//    SendByte( burstconfig);
//
//    // Chip select go high to end a flash command
//    MX25_CSHigh();
//
//    return FlashOperationSuccess;
//}
//
//
///*
// * Reset setting Command
// */
//
///*
// * Function:       MX25_CmdRSTEN
// * Arguments:      None.
// * Description:    Enable RST command
// * Return Message: FlashOperationSuccess
// */
//ReturnMsg MX25_CmdRSTEN( void )
//{
//    // Chip select go low to start a flash command
//    MX25_CSLow();
//
//    // Write RSTEN command
//    SendByte( FLASH_CMD_RSTEN);
//
//    // Chip select go high to end a flash command
//    MX25_CSHigh();
//
//    return FlashOperationSuccess;
//}
//
///*
// * Function:       MX25_CmdRST
// * Arguments:      fsptr, pointer of flash status structure
// * Description:    The RST instruction is used as a system (software) reset that
// *                 puts the device in normal operating Ready mode.
// * Return Message: FlashOperationSuccess
// */
//ReturnMsg MX25_CmdRST( FlashStatus *fsptr )
//{
//    // Chip select go low to start a flash command
//    MX25_CSLow();
//
//    // Write RST command = 0x99
//    SendByte(  FLASH_CMD_RST);
//
//    // Chip select go high to end a flash command
//    MX25_CSHigh();
//
//    // Reset current state
//    fsptr->ArrangeOpt = false;
//    fsptr->ModeReg = 0x00;
//
//    return FlashOperationSuccess;
//}
//
///*
// * Security Command
// */
//
//
///*
// * Suspend/Resume Command
// */
//
///*
// * Function:       MX25_CmdPGM_ERS_S
// * Arguments:      None
// * Description:    The PGM_ERS_S suspend Sector-Erase, Block-Erase or
// *                 Page-Program operations and conduct other operations.
// * Return Message: FlashOperationSuccess
// */
//ReturnMsg MX25_CmdPGM_ERS_S( void )
//{
//
//    // Chip select go low to start a flash command
//    MX25_CSLow();
//
//    // Send program/erase suspend command
//    SendByte( FLASH_CMD_PGM_ERS_S);
//
//    // Chip select go high to end a flash command
//    MX25_CSHigh();
//
//    return FlashOperationSuccess;
//}
//
///*
// * Function:       MX25_CmdPGM_ERS_R
// * Arguments:      None
// * Description:    The PGM_ERS_R resume Sector-Erase, Block-Erase or
// *                 Page-Program operations.
// * Return Message: FlashOperationSuccess
// */
//ReturnMsg MX25_CmdPGM_ERS_R( void )
//{
//
//    // Chip select go low to start a flash command
//    MX25_CSLow();
//
//    // Send resume command
//    SendByte( FLASH_CMD_PGM_ERS_R);
//
//    // Chip select go high to end a flash command
//    MX25_CSHigh();
//
//    return FlashOperationSuccess;
//}
//
//
///*
// * Function:       MX25_CmdNOP
// * Arguments:      None.
// * Description:    The NOP instruction is null operation of flash.
// * Return Message: FlashOperationSuccess
// */
//ReturnMsg MX25_CmdNOP( void )
//{
//    // Chip select go low to start a flash command
//    MX25_CSLow();
//
//    // Write NOP command = 0x00
//    SendByte( FLASH_CMD_NOP);
//
//    // Chip select go high to end a flash command
//    MX25_CSHigh();
//
//    return FlashOperationSuccess;
//}

