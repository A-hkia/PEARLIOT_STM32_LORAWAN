/*
 * COPYRIGHT (c) 2010-2017 MACRONIX INTERNATIONAL CO., LTD
 * SPI Flash Low Level Driver (LLD) Sample Code
 *
 * Flash device information define
 *
 * $Id: MX25_DEF.h,v 1.582 2017/06/15 09:51:20 mxclldb1 Exp $
 */

#ifndef    __MX25_DEF_H__
#define    __MX25_DEF_H__
/*
  Compiler Option
*/
/* Select your flash device type */
#define MX25R4035F

/* Note:
   Synchronous IO     : MCU will polling WIP bit after
                        sending prgram/erase command
   Non-synchronous IO : MCU can do other operation after
                        sending prgram/erase command
   Default is synchronous IO
*/
//#define    NON_SYNCHRONOUS_IO

/*
  Flash Related Parameter Define
*/

#define    Block_Offset       0x10000     // 64K Block size
#define    Block32K_Offset    0x8000      // 32K Block size
#define    Sector_Offset      0x1000      // 4K Sector size
#define    Page_Offset        0x0100      // 256 Byte Page size
#define    Page32_Offset      0x0020      // 32 Byte Page size (some products have smaller page size)
#define    Block_Num          (FlashSize / Block_Offset)

// Flash control register mask define
// status register
#define    FLASH_WIP_MASK         0x01
#define    FLASH_LDSO_MASK        0x02
#define    FLASH_QE_MASK          0x40
// security register
#define    FLASH_OTPLOCK_MASK     0x03
#define    FLASH_4BYTE_MASK       0x04
#define    FLASH_WPSEL_MASK       0x80
// configuration reigster
#define    FLASH_DC_MASK          0x80
#define    FLASH_DC_2BIT_MASK     0xC0
#define    FLASH_DC_3BIT_MASK     0x07
// other
#define    BLOCK_PROTECT_MASK     0xff
#define    BLOCK_LOCK_MASK        0x01

/*
  Flash ID, Timing Information Define
  (The following information could get from device specification)
*/

#ifdef MX25R4035F
#define    FlashID          0xc22813
#define    ElectronicID     0x13
#define    RESID0           0xc213
#define    RESID1           0x13c2
#define    FlashSize        0x40000        // 128 KB
#define    CE_period        10000
#define    MAX_FLASH_WAIT   3500
#define    tW               30       // 30ms
#define    tDP              1          // 10us
#define    tBP              1         // 100us
#define    tPP              100       // 10ms
#define    tSE              240      // 240ms
#define    tBE              3500     // 3.5s
#define    tBE32            1750     // 1.75s
#define    tPUW             10             // 10ms
#define    tWSR             1        // 1ms
// Support I/O mode
#define    SIO              0

#define    SUPPORT_RDCR     1
#define    SUPPORT_WRSR_CR  1
#define    SUPPORT_CR_DC    1
#endif

// dummy cycle configration
#ifdef DUMMY_CONF_2

#define    DUMMY_CONF_2READ       0x08040804
#define    DUMMY_CONF_4READ       0x0A080406
#define    DUMMY_CONF_FASTREAD    0x08080808
#define    DUMMY_CONF_DREAD       0x08080808
#define    DUMMY_CONF_QREAD       0x08080808

#else

#define    DUMMY_CONF_2READ       0x0A080604
#define    DUMMY_CONF_4READ       0x0A080406
#define    DUMMY_CONF_FASTREAD    0x0A080608
#define    DUMMY_CONF_DREAD       0x0A080608
#define    DUMMY_CONF_QREAD       0x0A080608

#endif

// dummy cycle configration
#ifdef DUMMY_CONF_2

#define    DUMMY_CONF_2READ       0x08040804
#define    DUMMY_CONF_4READ       0x0A080406
#define    DUMMY_CONF_FASTREAD    0x08080808
#define    DUMMY_CONF_DREAD       0x08080808
#define    DUMMY_CONF_QREAD       0x08080808

#else

#define    DUMMY_CONF_2READ       0x0A080604
#define    DUMMY_CONF_4READ       0x0A080406
#define    DUMMY_CONF_FASTREAD    0x0A080608
#define    DUMMY_CONF_DREAD       0x0A080608
#define    DUMMY_CONF_QREAD       0x0A080608

#endif

// dummy cycle configration
#ifdef DUMMY_CONF_2

#define    DUMMY_CONF_2READ       0x08040804
#define    DUMMY_CONF_4READ       0x0A080406
#define    DUMMY_CONF_FASTREAD    0x08080808
#define    DUMMY_CONF_DREAD       0x08080808
#define    DUMMY_CONF_QREAD       0x08080808

#else

#define    DUMMY_CONF_2READ       0x0A080604
#define    DUMMY_CONF_4READ       0x0A080406
#define    DUMMY_CONF_FASTREAD    0x0A080608
#define    DUMMY_CONF_DREAD       0x0A080608
#define    DUMMY_CONF_QREAD       0x0A080608

#endif

// dummy cycle configration
#ifdef DUMMY_CONF_2

#define    DUMMY_CONF_2READ       0x08040804
#define    DUMMY_CONF_4READ       0x0A080406
#define    DUMMY_CONF_FASTREAD    0x08080808
#define    DUMMY_CONF_DREAD       0x08080808
#define    DUMMY_CONF_QREAD       0x08080808

#else

#define    DUMMY_CONF_2READ       0x0A080604
#define    DUMMY_CONF_4READ       0x0A080406
#define    DUMMY_CONF_FASTREAD    0x0A080608
#define    DUMMY_CONF_DREAD       0x0A080608
#define    DUMMY_CONF_QREAD       0x0A080608

#endif

// dummy cycle configration
#ifdef DUMMY_CONF_2

#define    DUMMY_CONF_2READ       0x08040804
#define    DUMMY_CONF_4READ       0x0A080406
#define    DUMMY_CONF_FASTREAD    0x08080808
#define    DUMMY_CONF_DREAD       0x08080808
#define    DUMMY_CONF_QREAD       0x08080808

#else

#define    DUMMY_CONF_2READ       0x0A080604
#define    DUMMY_CONF_4READ       0x0A080406
#define    DUMMY_CONF_FASTREAD    0x0A080608
#define    DUMMY_CONF_DREAD       0x0A080608
#define    DUMMY_CONF_QREAD       0x0A080608

#endif

// Flash information define
#define    WriteStatusRegCycleTime     tW
#define    PageProgramCycleTime        tPP
#define    SectorEraseCycleTime        tSE
#define    BlockEraseCycleTime         tBE
#define    ChipEraseCycleTime          CE_period

#ifdef tBP
#define    ByteProgramCycleTime        tBP
#endif
#ifdef tWSR
#define    WriteSecuRegCycleTime       tWSR
#endif
#ifdef tBE32
#define    BlockErase32KCycleTime      tBE32
#endif
#ifdef tWREAW
#define    WriteExtRegCycleTime        tWREAW
#endif
#endif    /* end of __MX25_DEF_H__  */

