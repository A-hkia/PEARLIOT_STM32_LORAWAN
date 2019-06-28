/*!
 * \file      LoRaMacCrypto-SE.c
 *
 * \brief     LoRa MAC layer cryptography implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 *               ___ _____ _   ___ _  _____ ___  ___  ___ ___
 *              / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 *              \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 *              |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 *              embedded.connectivity.solutions===============
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Daniel Jaeckle ( STACKFORCE )
 *
 * \author    Johannes Bruder ( STACKFORCE )
 *
 * \author    Francois Lorrain ( IDEMIA )
 *
 *  */

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include "utilities.h"
#include "util_console.h"
#include "LoRaMacParser.h"
#include "LoRaMacSerializer.h"
#include "LoRaMacCrypto.h"
#include <string.h>




/*
 * Initial value of the frame counters
 */
#define FCNT_DOWN_INITAL_VALUE          0xFFFFFFFF

/*
 * Frame direction definition for uplink communications
 */
#define UPLINK                          0

/*
 * Frame direction definition for downlink communications
 */
#define DOWNLINK                        1

/*
 * CMAC/AES Message Integrity Code (MIC) Block B0 size
 */
#define MIC_BLOCK_BX_SIZE               16

/*
 * Size of JoinReqType is field for integrity check
 */
#define JOIN_REQ_TYPE_SIZE              1

/*
 * Size of DevNonce is field for integrity check
 */
#define DEV_NONCE_SIZE                  2

/*
 * Number of security context entries
 */
#define NUM_OF_SEC_CTX                  5

/*
 * Maximum size of the message that can be handled by the crypto operations
 */
#define CRYPTO_MAXMESSAGE_SIZE          256

/*
 * Maximum size of the buffer for crypto operations
 */
#define CRYPTO_BUFFER_SIZE              CRYPTO_MAXMESSAGE_SIZE + MIC_BLOCK_BX_SIZE

/*
 * MIC computaion offset
 */
#define CRYPTO_MIC_COMPUTATION_OFFSET   JOIN_REQ_TYPE_SIZE + LORAMAC_JOIN_EUI_FIELD_SIZE + DEV_NONCE_SIZE + LORAMAC_MHDR_FIELD_SIZE

//SE API communication
//***************************************
#define SE_API_SUCCESS                                  0x00    /*!< Generic no error */


extern I2C_HandleTypeDef *PearlIot_i2c;


//Debug
//***************************************

 /* LOG LEVELS */
 #define LOG_NONE    0 /*!< No Log level */
 #define LOG_ERR     1 /*!< Only error Log level*/
 #define LOG_WARN    2 /*!< Error and warning Log level */
 #define LOG_INFO    3 /*!< Info, error, warning Log level */
 #define LOG_DBG     4 /*!< Debug, info, error, warning Log level */

 uint8_t se_log_level = LOG_DBG;
 const char* const LOG_HEADER[] = {"#NONE#", "#ERR#", "#WARN", "#INFO#", "#DBG#"};



//I2C
//***************************************
#define SE_RSC_I2C_SUCCESS                         0x00 /*!< Generic no error */
#define SE_RSC_I2C_FAIL                            0xFC /*!< Generic error for a I2C fail */

#define SE_RSC_I2C_ADDRESS                         0x5C /*!< address 8-bit I2C slave address [ addr | 0 ] */

#define SE_RSC_I2C_TIMEOUT_1S                      1000 /*!< Timeout 1s */
#define SE_RSC_I2C_TIMEOUT_100MS                   100  /*!< Timeout 100ms */

// define for I2C layer
#define MDL_I2C_PROT_DELAY              0x10    /*!< polling delay in ms, min time between 2 READ_RESP_STATUS  */
#define MDL_I2C_PROT_WAIT               0x20    /*!< Protocol delay between 2 consecutive reads  */
#define MDL_I2C_PROT_RETRIES_MAX        0xF0    /*!< Maximum number of retries */

static const uint8_t slaveAddress = SE_RSC_I2C_ADDRESS;
static uint32_t timeout = SE_RSC_I2C_TIMEOUT_1S;        /*!< I2C timeout */

//static I2C_HandleTypeDef I2cHandle;                     /*!< I2C handler declaration */
//#define I2Cx                            I2C1
//#define I2Cx_CLK_ENABLE()               __HAL_RCC_I2C1_CLK_ENABLE()
//#define I2Cx_SDA_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
//#define I2Cx_SCL_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
//
//#define I2Cx_FORCE_RESET()              __HAL_RCC_I2C1_FORCE_RESET()
//#define I2Cx_RELEASE_RESET()            __HAL_RCC_I2C1_RELEASE_RESET()
//
///* Definition for I2Cx Pins */
//#define I2Cx_SCL_PIN                    GPIO_PIN_8
//#define I2Cx_SCL_GPIO_PORT              GPIOB
//#define I2Cx_SDA_PIN                    GPIO_PIN_9
//#define I2Cx_SDA_GPIO_PORT              GPIOB
//#define I2Cx_SCL_SDA_AF                 GPIO_AF4_I2C1
//#define I2C_DUTYCYCLE                   I2C_DUTYCYCLE_16_9
//
//#if defined (USE_B_L072Z_LRWAN1)
//#define NUCLEO_I2C_EXPBD_TIMING_100KHZ  0x10A13E56 /* Analog Filter ON, Rise Time 400ns, Fall Time 100ns */
//#define NUCLEO_I2C_EXPBD_TIMING_400KHZ  0x00B1112E /* Analog Filter ON, Rise Time 250ns, Fall Time 100ns */
//#endif

#define LOG_DBG     4 /*!< Debug, info, error, warning Log level */
// Command tags
#define TAG_SECURE_UPLINK   0x60  /*!< LORA INS tag for secure uplink */
#define TAG_VERIFY_DOWNLINK 0x61  /*!< LORA INS tag for verify downlink */
#define TAG_JOIN_REQUEST    0x62  /*!< LORA INS tag for join request */
#define TAG_REJOIN_REQUEST  0x63  /*!< LORA INS tag for rejoin request */
#define TAG_JOIN_ACCEPT     0x64  /*!< LORA INS tag for join accept */

//Communication tags
#define TAG_MLS_WRAP                       0x21 /*!< Tag for MLS Wrap command */
#define TAG_MLS_UNWRAP                     0x20 /*!< Tag for MLS Unwrap command */
#define TAG_PUT_DATA                       0x04 /*!< Tag for Put Data command */
#define TAG_GET_DATA                       0x0D /*!< Tag for Get Data command */
#define TAG_GET_RANDOM                     0x10 /*!< Tag for Get Random command */
#define TAG_READ_GPIO                      0xE0 /*!< Tag for Read GPIO line state command */
#define TAG_WRITE_GPIO                     0xE1 /*!< Tag for Write GPIO line state command */

// Command tag
//#define DEV_EUI_ADDRESS		0x8400  /*!< DevEUI's address in the SE */

#define SE_RSC_I2C_SUCCESS                         0x00 /*!< Generic no error */
// Status
#define MDL_I2C_PROT_SE_STATUS_BASE            (uint16_t)0x0000    /*!< Transport protocol SE status  base*/
#define MDL_I2C_PROT_FAIL                      (uint16_t)0x0100    /*!< Transport protocol issue */
#define MDL_I2C_PROT_RETRIES_OVER              (uint16_t)0x0200    /*!< Transport protocol max retries done */

// define for I2C layer
#define MDL_I2C_PROT_DELAY              0x10    /*!< polling delay in ms, min time between 2 READ_RESP_STATUS  */
#define MDL_I2C_PROT_WAIT               0x20    /*!< Protocol delay between 2 consecutive reads  */
#define MDL_I2C_PROT_RETRIES_MAX        0xF0    /*!< Maximum number of retries */

// Response status
#define MDL_I2C_PROT_RS_SIZE            0x03    /*!< Size of response status + length */


#define MDL_I2C_PROT_RS_WTX_FRAME       0xF0    /*!< Response first part - WTX frame */
#define MDL_I2C_PROT_RS_ISSUE           0xF1    /*!< Response first part - Transport protocol issue */
#define MDL_I2C_PROT_RS_FIRST_RFU       0xF2    /*!< Response first part - first RFU value */
#define MDL_I2C_PROT_RS_LAST_RFU        0xFF    /*!< Response first part - last RFU value */
#define MDL_I2C_PROT_RS_FIRST_OK        0x00    /*!< Response first part - Status first possible value */
#define MDL_I2C_PROT_RS_LAST_OK         0xEF    /*!< Response first part - Status last possible value */

// SE communication
#define MW_STATUS_SE_STATUS_BASE                0x0000   /*!< No error on MW side, SE status */
#define MW_STATUS_WRONG_PARAMETER               0x0100   /*!< Wrong parameter provided to the MW function */
#define MW_STATUS_BUFFER_LENGTH                 0x0200   /*!< Buffer allocation on MW side has failed */
#define MW_STATUS_OUTPUT_LENGTH                 0x0300   /*!< MW output buffer not large enough to get the answer from the SE */
#define MW_STATUS_COMM_FAILURE                  0x0400   /*!< Communication failure with the SE */

typedef enum
{
    // Success
    SE_STATUS_SUCCESSFUL_EXECUTION = 0x00,              /*!< SE successful execution */

    // Error system
    SE_STATUS_UNSPECIFIED_ERROR = 0x10,                 /*!< SE unspecified error */
    SE_STATUS_INSTRUCTION_NOT_SUPPORTED = 0x11,         /*!< SE instruction not supported */
    SE_STATUS_INVALID_DATA_OBJECT_IDENTIFIER = 0x12,    /*!< SE invalid data object identifier */
    SE_STATUS_INCONSISTENT_FORMAT = 0x13,               /*!< SE inconsistent format */
    SE_STATUS_ACCESS_FORBIDDEN = 0x14,                  /*!< SE access forbidden */
    SE_STATUS_INVALID_STATE = 0x15,                     /*!< SE invalid state */
    SE_STATUS_OUTPUT_BUFFER_OVERFLOW = 0x16,            /*!< SE output buffer overflow */
    SE_STATUS_SECURITY_ERROR = 0x17,                    /*!< SE security error */

    // Error application
    SE_STATUS_COUNTER_OVERFLOW = 0x20,                  /*!< SE counter overflow */

    // Proprietary 80-FF

    // Only in debug mode
    SE_STATUS_DBG_MUTE = 0xE0,                          /*!< SE debug MUTE */
    SE_STATUS_DBG_IOL = 0xE1,                           /*!< SE debug IOL */
    SE_STATUS_DBG_RAM_ALLOC = 0xE2,                     /*!< SE debug RAM ALLOC */
    SE_STATUS_DBG_RAM_OVERWRITE = 0xE3,                 /*!< SE debug RAM OVERWRITE */
    SE_STATUS_DBG_CSU = 0xE4,                           /*!< SE debug CSU */
    SE_STATUS_DBG_TEARING = 0xE5,                       /*!< SE debug TEARING */

    // Internal
    SE_STATUS_INT_SUB_COMMAND = 0xF0                    /*!< SE internal sub command*/

} se_status_t;

//Second buffer to be used in functions that may overwrite the first buffer
//***************************************
static uint8_t pearliot_buffer[256];

//Initialization of the peripheral
//***************************************
//uint8_t SE_RSC_i2c_Init(uint32_t frequency)
//{
//
//    I2cHandle.Instance             = I2Cx;

//#if defined(USE_STM32L1XX_NUCLEO)
//    I2cHandle.Init.ClockSpeed      = frequency;
//    I2cHandle.Init.DutyCycle       = I2C_DUTYCYCLE;
//    I2cHandle.Init.OwnAddress1     = (slaveAddress<<1);
//#elif defined (USE_B_L072Z_LRWAN1)
//    I2cHandle.Init.OwnAddress1     = (slaveAddress<<1);
//    if (frequency == 100000)
//    {
//        I2cHandle.Init.Timing = NUCLEO_I2C_EXPBD_TIMING_100KHZ;
//    }
//    else if (frequency == 400000)
//    {
//        I2cHandle.Init.Timing = NUCLEO_I2C_EXPBD_TIMING_400KHZ;
//    }
//    else
//    {
//        /* Not supported yet -> please add your desired frequency settings */
//        return SE_RSC_I2C_FAIL;
//    }
//#elif defined(USE_MIROMICO)
//    //I2cHandle.Init.Timing =0x00B07DB9;
//    I2cHandle.Init.Timing = 0x00B1112E;
//    I2cHandle.Init.OwnAddress1     = (slaveAddress<<1);
//    I2cHandle.Init.OwnAddress2     = 0;
//#elif defined(USE_STM32WB55_NUCLEO)
//    I2cHandle.Init.Timing =0x00707CBB;
//    I2cHandle.Init.OwnAddress1     = (slaveAddress<<1);
//    I2cHandle.Init.OwnAddress2     = 0;
//#else
//#error "Add your platform here"
//#endif
//    I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
//    I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//    //I2cHandle.Init.OwnAddress2     = 0xFF;
//    I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//    I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
//
//    if(HAL_I2C_Init(&I2cHandle) != HAL_OK)
//    {
//        /* Initialization Error */
//        return SE_RSC_I2C_FAIL;
//    }

//    return SE_RSC_I2C_SUCCESS;

//}
/*
 *************************************************************************************************************
 * Defining Read and Write functions to be able to use them in the MDL_i2c_prot_SendReceiveAppCommand Function
 *************************************************************************************************************
 * */
static uint8_t SE_RSC_i2c_Write(uint8_t *writeBuffer, uint8_t writeBufferLength)
{
  /**
   // DEFINEs the device's manufacturer code used to communicate with the SE through the I2C interface
   * \warning I2C module must be initially initialized
   */
    /*## Start the transmission process #####################################*/
    /* While the I2C in reception process, user can transmit data through
       "aTxBuffer" buffer */
    while(HAL_I2C_Master_Transmit(PearlIot_i2c, (uint16_t)(slaveAddress<<1), writeBuffer, writeBufferLength, timeout)!= HAL_OK)
    {
        /* Error_Handler() function is called when Timeout error occurs.
        When Acknowledge failure occurs (Slave don't acknowledge it's address)
        Master restarts communication */
        if (HAL_I2C_GetError(PearlIot_i2c) != HAL_I2C_ERROR_AF)
                return SE_RSC_I2C_FAIL;
    }

    return SE_RSC_I2C_SUCCESS;
}

static uint8_t SE_RSC_i2c_Read(uint8_t *rcvBuffer, uint8_t rcvBufferLength)
{
  /**
   * use here your manufacturer board maker I2C Read function
   * \warning Initiate I2C into your application init
   */

    /*## Put I2C peripheral in reception process ############################*/
    while(HAL_I2C_Master_Receive(PearlIot_i2c, (uint16_t)(slaveAddress<<1), (uint8_t*)rcvBuffer, rcvBufferLength, timeout)!= HAL_OK)
    {
        /* Error_Handler() function is called when Timeout error occurs.
           When Acknowledge failure occurs (Slave don't acknowledge it's address)
           Master restarts communication */
        if (HAL_I2C_GetError(PearlIot_i2c) != HAL_I2C_ERROR_AF)
            return SE_RSC_I2C_FAIL;
    }

    return SE_RSC_I2C_SUCCESS;
}
/*
*************************************************************************************************************
* Defining Debug functions
*************************************************************************************************************
* */
void SE_RSC_serial_debug_log (uint8_t level,  char * buffer)
{
    if (level <= se_log_level)
    {
        PRINTF("%s ", LOG_HEADER[level]);
        PRINTF("%s", buffer);
        PRINTF("\n\r");
    }
}

void SE_RSC_serial_debug_hex (uint8_t level,  char * buffer, uint8_t * hexBuffer, uint16_t hexLen)
{
    if (level <= se_log_level)
    {
        PRINTF("%s ", LOG_HEADER[level]);
        PRINTF("%s", buffer);
        for ( uint16_t i=0 ; i<hexLen ; i++ )
        {
            PRINTF("%.2x ",hexBuffer[i]);
        }
        PRINTF("\n\r");
    }
}

void SE_RSC_serial_debug_log (uint8_t level,  char * buffer);

/*
 *************************************************************************************************************
 * Defining Send and Receive commands between the SE and its component
 *************************************************************************************************************
 * */

static uint16_t MDL_i2c_prot_SendReceiveAppCommand(uint8_t* sendRcvBuffer, uint8_t* sendRcvBufferLength)
{
    uint8_t ret_i2c = 0;
    uint8_t nb_read_resp = 0;
    uint8_t status;

    SE_RSC_serial_debug_hex (LOG_DBG, (char*)"[I2C PROT] Command: ", sendRcvBuffer, *sendRcvBufferLength);

    //Initialize peripheral
//    ret_i2c=SE_RSC_i2c_Init(400000);
//    if(ret_i2c == SE_RSC_I2C_SUCCESS){
//    }
//    else{
//    	SE_RSC_serial_debug_log (LOG_ERR, "Peripheral could not be initialized");
//    }
//    /* Enable Systick for the duration of the I2C command to handle timeouts and wait */


    ret_i2c = SE_RSC_i2c_Write( sendRcvBuffer, *sendRcvBufferLength );

    // If write operation is passed, then we are waiting for the answer
    if(ret_i2c == SE_RSC_I2C_SUCCESS)
    {
        do
        {
            // Wait before polling
            HAL_Delay(MDL_I2C_PROT_WAIT);
            // Read Response Status
            nb_read_resp++;
            ret_i2c = SE_RSC_i2c_Read (sendRcvBuffer, MDL_I2C_PROT_RS_SIZE);

            // I2C Read failed
            if ( ret_i2c != SE_RSC_I2C_SUCCESS)
            {
                // the MSB part of the return is the I2C status
                SE_RSC_serial_debug_log (LOG_ERR, "[I2C PROT] I2C RESPONSE STATUS FAILED, error code: SE_RSC_I2C_FAIL");
                return MDL_I2C_PROT_FAIL;
            }

            // Check Transport protocol issue - Protocol error and RFU
            if ( sendRcvBuffer[0] > MDL_I2C_PROT_RS_WTX_FRAME)
            {
                // the MSB part of the return is the I2C status , the LSB one the SE status
                SE_RSC_serial_debug_log (LOG_ERR, "[I2C PROT] TRANSPORT PROTOCOL ISSUE");
                return MDL_I2C_PROT_SE_STATUS_BASE + (uint16_t)sendRcvBuffer[0];
            }
        }
        // Extra time required or retries are over
        while ( (sendRcvBuffer[0] == MDL_I2C_PROT_RS_WTX_FRAME) && (nb_read_resp <= MDL_I2C_PROT_RETRIES_MAX) );
    }
    else
    {
        SE_RSC_serial_debug_log (LOG_ERR, "[I2C PROT] I2C WRITE STATUS FAILED, error code: SE_RSC_I2C_FAIL");
        return MDL_I2C_PROT_FAIL;
    }

    if (nb_read_resp > MDL_I2C_PROT_RETRIES_MAX)
    {
        // Software reset ?
        SE_RSC_serial_debug_log (LOG_ERR, "[I2C PROT] RETRIES MAX IS OVER, error code MDL_I2C_PROT_RETRIES_OVER");
        return MDL_I2C_PROT_RETRIES_OVER;
    }

    SE_RSC_serial_debug_hex (LOG_DBG, (char*)"[I2C PROT] Response 1: ", sendRcvBuffer, MDL_I2C_PROT_RS_SIZE);

    status = sendRcvBuffer[0];
    // Extract data length available from response status,
    *sendRcvBufferLength = (uint16_t)((sendRcvBuffer[1]<<8) & 0xFF00) | sendRcvBuffer[2];

    if (*sendRcvBufferLength >0 )
    {
        // Wait between 2 consecutive reads
    	//HAL_Delay(MDL_I2C_PROT_WAIT);

        // Read response
        ret_i2c = SE_RSC_i2c_Read (sendRcvBuffer, *sendRcvBufferLength);

        // I2C Read failed
        if ( ret_i2c != SE_RSC_I2C_SUCCESS)
        {
            SE_RSC_serial_debug_log (LOG_ERR, "[I2C PROT] READ DATA STATUS FAILED, error code: SE_RSC_I2C_FAIL");
            return MDL_I2C_PROT_FAIL;
        }

        SE_RSC_serial_debug_hex (LOG_DBG, (char*)"[I2C PROT] Response 2: ", sendRcvBuffer, *sendRcvBufferLength);
    }

    return MDL_I2C_PROT_SE_STATUS_BASE + (uint16_t)status;
}


/*
 *  API functions
 */




LoRaMacCryptoStatus_t LoRaMacCryptoInit( EventNvmCtxChanged cryptoNvmCtxChanged )
{
// Need to power up the Pearl IoT

    return LORAMAC_CRYPTO_SUCCESS;
}

LoRaMacCryptoStatus_t LoRaMacCryptoSetLrWanVersion( Version_t version )
{
    return LORAMAC_CRYPTO_SUCCESS;
}

LoRaMacCryptoStatus_t LoRaMacCryptoRestoreNvmCtx( void* cryptoNvmCtx )
{
   return LORAMAC_CRYPTO_SUCCESS;
}

void* LoRaMacCryptoGetNvmCtx( size_t* cryptoNvmCtxSize )
{
    return LORAMAC_CRYPTO_SUCCESS;
}

LoRaMacCryptoStatus_t LoRaMacCryptoSetKey( KeyIdentifier_t keyID, uint8_t* key )
{
    return LORAMAC_CRYPTO_SUCCESS;
}

LoRaMacCryptoStatus_t LoRaMacCryptoPrepareJoinRequest( LoRaMacMessageJoinRequest_t* macMsg )
{
	uint16_t status;
    if( macMsg == 0 )
    {
        return LORAMAC_CRYPTO_ERROR_NPE;
    }
    macMsg->Buffer[0]=TAG_JOIN_REQUEST;
    macMsg->Buffer[1]= macMsg->MHDR.Value;
    macMsg->BufSize = 2;
    status = MDL_i2c_prot_SendReceiveAppCommand(macMsg->Buffer, &macMsg->BufSize);

    if (status == SE_API_SUCCESS) {
    	return LORAMAC_CRYPTO_SUCCESS;
    } else {
    	return LORAMAC_CRYPTO_ERROR;
    }
}

LoRaMacCryptoStatus_t LoRaMacCryptoPrepareReJoinType1( LoRaMacMessageReJoinType1_t* macMsg )
{
    if( macMsg == 0 )
    {
        return LORAMAC_CRYPTO_ERROR_NPE;
    }

    return LORAMAC_CRYPTO_SUCCESS;
}

LoRaMacCryptoStatus_t LoRaMacCryptoPrepareReJoinType0or2( LoRaMacMessageReJoinType0or2_t* macMsg )
{
    if( macMsg == 0 )
    {
        return LORAMAC_CRYPTO_ERROR_NPE;
    }
    return LORAMAC_CRYPTO_SUCCESS;
}

LoRaMacCryptoStatus_t LoRaMacCryptoHandleJoinAccept( JoinReqIdentifier_t joinReqType, uint8_t* joinEUI, LoRaMacMessageJoinAccept_t* macMsg )
{
	uint8_t length, status;

	if( ( macMsg == 0 ) )
    {
        return LORAMAC_CRYPTO_ERROR_NPE;
    }
	pearliot_buffer[0] = TAG_JOIN_ACCEPT;
	memcpy(&pearliot_buffer[1],macMsg->Buffer, macMsg->BufSize);
    length = macMsg->BufSize +1;
    status = MDL_i2c_prot_SendReceiveAppCommand(pearliot_buffer, &length);
    if (status != SE_API_SUCCESS) {
    	return LORAMAC_CRYPTO_ERROR;
    }

    // Parse the message
    if( LoRaMacParserJoinAccept( macMsg ) != LORAMAC_PARSER_SUCCESS )
    {
        return LORAMAC_CRYPTO_ERROR_PARSER;
    }


    return LORAMAC_CRYPTO_SUCCESS;
}

LoRaMacCryptoStatus_t LoRaMacCryptoSecureMessage( uint32_t fCntUp, uint8_t txDr, uint8_t txCh, LoRaMacMessageData_t* macMsg )
{

	uint8_t length, status;
	uint8_t pearl_iot_buf [256];

	if( ( macMsg == 0 ) )
    {
        return LORAMAC_CRYPTO_ERROR_NPE;
    }
	pearl_iot_buf[0] = TAG_SECURE_UPLINK;
	memcpy(&pearl_iot_buf[1],macMsg->Buffer, macMsg->BufSize);
    length = macMsg->BufSize +1;
    status = MDL_i2c_prot_SendReceiveAppCommand(pearl_iot_buf, &length);
    if (status != SE_API_SUCCESS) {
    	return LORAMAC_CRYPTO_ERROR;
    }
    memcpy(macMsg->Buffer,pearliot_buffer,length);
    macMsg->BufSize=length;

return LORAMAC_CRYPTO_SUCCESS;
}

LoRaMacCryptoStatus_t LoRaMacCryptoUnsecureMessage( AddressIdentifier_t addrID, uint32_t address, FCntIdentifier_t fCntID, uint32_t fCntDown, LoRaMacMessageData_t* macMsg )
{

	uint8_t length, status;
	uint8_t pearl_iot_buf [256];

	if( ( macMsg == 0 ) )
    {
        return LORAMAC_CRYPTO_ERROR_NPE;
    }
	pearl_iot_buf[0] = TAG_VERIFY_DOWNLINK;
	memcpy(&pearl_iot_buf[1],macMsg->Buffer, macMsg->BufSize);
    length = macMsg->BufSize +1;
    status = MDL_i2c_prot_SendReceiveAppCommand(pearl_iot_buf, &length);
    if (status != SE_API_SUCCESS) {
    	return LORAMAC_CRYPTO_ERROR;
    }
    memcpy(macMsg->Buffer,pearliot_buffer,length);
    macMsg->BufSize=length;

return LORAMAC_CRYPTO_SUCCESS;
}

LoRaMacCryptoStatus_t LoRaMacCryptoDeriveMcKEKey( KeyIdentifier_t keyID, uint16_t nonce, uint8_t* devEUI )
{
    if( devEUI == 0 )
    {
        return LORAMAC_CRYPTO_ERROR_NPE;
    }

    // Nonce SHALL be greater than 15
    if( nonce < 16 )
    {
        return LORAMAC_CRYPTO_FAIL_PARAM;
    }

    return LORAMAC_CRYPTO_SUCCESS;
}

LoRaMacCryptoStatus_t LoRaMacCryptoDeriveMcSessionKeyPair( AddressIdentifier_t addrID, uint32_t mcAddr )
{
    if( mcAddr == 0 )
    {
        return LORAMAC_CRYPTO_ERROR_NPE;
    }

     return LORAMAC_CRYPTO_SUCCESS;
}



void HW_GetUniqueId(uint8_t *id)

{
	uint8_t length;
	uint8_t  status;

    //Make sure the Secure Element is ready to accept the command
	//HAL_Delay(10);
    //SE_RSC_i2c_Read (pearliot_buffer, MDL_I2C_PROT_RS_SIZE);

	pearliot_buffer[0]=TAG_GET_DATA;
	pearliot_buffer[1]=0x84;
	pearliot_buffer[2]=0x00;
	length = 3;
	status = MDL_i2c_prot_SendReceiveAppCommand(pearliot_buffer, &length);
	if (status == SE_API_SUCCESS) {
		memcpy(id,pearliot_buffer,length);
	}
	else{
	return;
	}
}
