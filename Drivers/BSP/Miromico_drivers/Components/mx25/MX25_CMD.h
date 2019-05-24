/*
 * COPYRIGHT (c) 2010-2017 MACRONIX INTERNATIONAL CO., LTD
 * SPI Flash Low Level Driver (LLD) Sample Code
 *
 * SPI interface command hex code, type definition and function prototype.
 *
 * $Id: MX25_CMD.h,v 1.23 2015/12/15 06:31:21 mxclldb1 Exp $
 */
#ifndef    __MX25_CMD_H__
#define    __MX25_CMD_H__

#include "MX25_DEF.h"
#include "hw.h"

/*** MX25 series command hex code definition ***/
//ID comands
#define    FLASH_CMD_RDID      0x9F    //RDID (Read Identification)
#define    FLASH_CMD_RES       0xAB    //RES (Read Electronic ID)
#define    FLASH_CMD_REMS      0x90    //REMS (Read Electronic & Device ID)

//Register comands
#define    FLASH_CMD_WRSR      0x01    //WRSR (Write Status Register)
#define    FLASH_CMD_RDSR      0x05    //RDSR (Read Status Register)
#define    FLASH_CMD_WRSCUR    0x2F    //WRSCUR (Write Security Register)
#define    FLASH_CMD_RDSCUR    0x2B    //RDSCUR (Read Security Register)
#define    FLASH_CMD_RDCR      0x15    //RDCR (Read Configuration Register)

//READ comands
#define    FLASH_CMD_READ        0x03    //READ (1 x I/O)
#define    FLASH_CMD_2READ       0xBB    //2READ (2 x I/O)
#define    FLASH_CMD_4READ       0xEB    //4READ (4 x I/O)
#define    FLASH_CMD_FASTREAD    0x0B    //FAST READ (Fast read data)
#define    FLASH_CMD_DREAD       0x3B    //DREAD (1In/2 Out fast read)
#define    FLASH_CMD_QREAD       0x6B    //QREAD (1In/4 Out fast read)
#define    FLASH_CMD_RDSFDP      0x5A    //RDSFDP (Read SFDP)

//Program comands
#define    FLASH_CMD_WREN     0x06    //WREN (Write Enable)
#define    FLASH_CMD_WRDI     0x04    //WRDI (Write Disable)
#define    FLASH_CMD_PP       0x02    //PP (page program)
#define    FLASH_CMD_4PP      0x38    //4PP (Quad page program)

//Erase comands
#define    FLASH_CMD_SE       0x20    //SE (Sector Erase)
#define    FLASH_CMD_BE32K    0x52    //BE32K (Block Erase 32kb)
#define    FLASH_CMD_BE       0xD8    //BE (Block Erase)
#define    FLASH_CMD_CE       0x60    //CE (Chip Erase) hex code: 60 or C7

//Mode setting comands
#define    FLASH_CMD_DP       0xB9    //DP (Deep Power Down)
#define    FLASH_CMD_ENSO     0xB1    //ENSO (Enter Secured OTP)
#define    FLASH_CMD_EXSO     0xC1    //EXSO  (Exit Secured OTP)
#ifdef SBL_CMD_0x77
#define    FLASH_CMD_SBL      0x77    //SBL (Set Burst Length) new: 0x77
#else
#define    FLASH_CMD_SBL      0xC0    //SBL (Set Burst Length) Old: 0xC0
#endif

//Reset comands
#define    FLASH_CMD_RSTEN     0x66    //RSTEN (Reset Enable)
#define    FLASH_CMD_RST       0x99    //RST (Reset Memory)

//Security comands
#ifdef LCR_CMD_0xDD_0xD5
#else
#endif

//Suspend/Resume comands
#define    FLASH_CMD_PGM_ERS_S    0xB0    //PGM/ERS Suspend (Suspends Program/Erase)
#define    FLASH_CMD_PGM_ERS_R    0x30    //PGM/ERS Erase (Resumes Program/Erase)
#define    FLASH_CMD_NOP          0x00    //NOP (No Operation)

// Use non synchronous io mode
#define NON_SYNCHRONOUS_IO

// Return Message
typedef enum {
  FlashOperationSuccess,
  FlashWriteRegFailed,
  FlashTimeOut,
  FlashIsBusy,
  FlashQuadNotEnable,
  FlashAddressInvalid
} ReturnMsg;

// Flash status structure define
struct sFlashStatus {
  /* Mode Register:
   * Bit  Description
   * -------------------------
   *  7   RYBY enable
   *  6   Reserved
   *  5   Reserved
   *  4   Reserved
   *  3   Reserved
   *  2   Reserved
   *  1   Parallel mode enable
   *  0   QPI mode enable
  */
  uint8_t    ModeReg;
  bool     ArrangeOpt;
};

typedef struct sFlashStatus FlashStatus;

/* Utility functions */
void MX25_Init(void* handle);

/** Check whether flash is busy or not */
bool MX25_IsFlashBusy(void);

/**
 * Wait while flash is busy with timeout
 *
 * @param timeout timeout in [ms]
 * Arguments:      ExpectTime, expected time-out value of flash operations.
 *                 No use at non-synchronous IO mode.
 * Description:    Synchronous IO:
 *                 If flash is ready return true.
 *                 If flash is time-out return false.
 *                 Non-synchronous IO:
 *                 Always return true
 * Return Message: true, false
 */
bool MX25_WaitFlashReady(uint32_t timeout);

/* Flash commands */
ReturnMsg MX25_CmdRDID(uint32_t* Identification);
ReturnMsg MX25_CmdRES(uint8_t* ElectricIdentification);
ReturnMsg MX25_CmdREMS(uint16_t* REMS_Identification, FlashStatus* fsptr);

ReturnMsg MX25_CmdRDSR(uint8_t* StatusReg);
#ifdef SUPPORT_WRSR_CR
ReturnMsg MX25_CmdWRSR(uint16_t UpdateValue);
#else
ReturnMsg MX25_CmdWRSR(uint8_t UpdateValue);
#endif
ReturnMsg MX25_CmdRDSCUR(uint8_t* SecurityReg);
ReturnMsg MX25_CmdWRSCUR(void);
ReturnMsg MX25_CmdRDCR(uint8_t* ConfigReg);

ReturnMsg MX25_CmdREAD(uint32_t flash_address, uint8_t* target_address, uint32_t byte_length);
ReturnMsg MX25_Cmd2READ(uint32_t flash_address, uint8_t* target_address, uint32_t byte_length);
ReturnMsg MX25_Cmd4READ(uint32_t flash_address, uint8_t* target_address, uint32_t byte_length);
ReturnMsg MX25_CmdFASTREAD(uint32_t flash_address, uint8_t* target_address, uint32_t byte_length);
ReturnMsg MX25_CmdDREAD(uint32_t flash_address, uint8_t* target_address, uint32_t byte_length);
ReturnMsg MX25_CmdQREAD(uint32_t flash_address, uint8_t* target_address, uint32_t byte_length);
ReturnMsg MX25_CmdRDSFDP(uint32_t flash_address, uint8_t* target_address, uint32_t byte_length);

ReturnMsg MX25_CmdWREN(void);
ReturnMsg MX25_CmdWRDI(void);
ReturnMsg MX25_CmdPP(uint32_t flash_address, uint8_t* source_address, uint16_t byte_length);
ReturnMsg MX25_Cmd4PP(uint32_t flash_address, uint8_t* source_address, uint32_t byte_length);

ReturnMsg MX25_CmdSE(uint32_t flash_address);
ReturnMsg MX25_CmdBE32K(uint32_t flash_address);
ReturnMsg MX25_CmdBE(uint32_t flash_address);
ReturnMsg MX25_CmdCE(void);

ReturnMsg MX25_CmdDP(void);
ReturnMsg MX25_CmdENSO(void);
ReturnMsg MX25_CmdEXSO(void);
ReturnMsg MX25_CmdSBL(uint8_t burstconfig);

ReturnMsg MX25_CmdRSTEN(void);
ReturnMsg MX25_CmdRST(FlashStatus* fsptr);

ReturnMsg MX25_CmdPGM_ERS_S(void);
ReturnMsg MX25_CmdPGM_ERS_R(void);
ReturnMsg MX25_CmdNOP(void);
#endif    /* __MX25_CMD_H__ */
