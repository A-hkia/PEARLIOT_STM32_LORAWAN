/* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "low_power_manager.h"
#include "lora.h"
#include "bsp.h"
#include "timeServer.h"
#include "version.h"
#include "tiny_sscanf.h"

#define BLINK_TIME  50

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFF_SIZE  64

const volatile static lora_Configuration_t lora_config = {
  .otaa = ENABLE,
#if defined(REGION_EU868)
  .duty_cycle = ENABLE,
#else
  .duty_cycle = DISABLE,
#endif
  .DevEui = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  .AppEui = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  .AppKey = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  .NwkSKey = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  .AppSKey = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  .DevAddr = 0x00000000,
  .NetID = 13,
  .application_port = 2,
  .TxDutyCycleTime = 15000,
  .class = CLASS_A,
  .ReqAck = DISABLE,
};

/*!
 * User application data
 */
static uint8_t AppDataBuff[LORAWAN_APP_DATA_BUFF_SIZE];

/*!
 * User application data structure
 */
static lora_AppData_t AppData = {
  AppDataBuff,
  LORAWAN_APP_DATA_BUFF_SIZE,
  0,
  3
};

/* Flag to notify main loop about acked/nacked messages */
static volatile bool txMsgFlag = false;
/* Message ack/nack state */
static volatile bool txMsgAck = false;
/* Confirmed message uplink frame counter */
static volatile uint32_t txMsgFrmCnt = 0;

/*!
 * Timer to handle the application Tx Led to toggle
 */
static TimerEvent_t TxLedTimer;
static void OnTimerLedEvent(void);

/* call back when LoRa will transmit a frame*/
static void LoraTxData(lora_AppData_t* AppData);

/* call back when LoRa has received a frame*/
static void LORA_RxData(lora_AppData_t* AppData);

/* call back when LoRa endNode has just joined*/
static void LORA_HasJoined(void);

/* call back when LoRa endNode has just switch the class*/
static void LORA_ConfirmClass(DeviceClass_t Class);

static void LORA_Confirmation(uint32_t frmCnt, bool ack);

static void OnTxTimerEvent(void);

static void LoraStartTx();

/* load call backs*/
static LoRaMainCallback_t LoRaMainCallbacks = {
  HW_GetBatteryLevel,
  HW_GetTemperatureLevel,
  HW_GetUniqueId,
  HW_GetRandomSeed,
  LORA_RxData,
  LORA_HasJoined,
  LORA_ConfirmClass,
  LORA_Confirmation
};

/*!
 * Specifies the state of the application LED
 */
static uint8_t AppLedStateOn = RESET;

// LoRaWAN packet timer
TimerEvent_t TxTimer;

/* !
 * Initializes the Lora Parameters
 */
const volatile static LoRaParam_t LoRaParamInit = {
  DISABLE,       // ADR enable / disable
  DR_0,          // data rate
  true,          // public network
  3              // Join retrails
};

/** Device configuration data */
const volatile static eSensorConfig_t sensorConfig = {
  .reportBattLevel = ENABLE,
  .internalTemp = ENABLE,
  .upperThreshold = 3000,
  .lowerThreshold = -800,
  .hysteresis = 10
};

static uint32_t alarm_hi = 0;
static uint32_t alarm_lo = 0;
static uint32_t intervalCnt = 0;
const volatile static uint32_t interval = 0;

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32 HAL library initialization*/
  HAL_Init();

  /* Configure the system clock*/
  SystemClock_Config();

  /* Configure the debug mode*/
  DBG_Init();

  /* Configure the hardware*/
  HW_Init();

  // Disable Stand-by mode
  LPM_SetOffMode(LPM_APPLI_Id , LPM_Disable);

  printSystemInformation("Miromico LoRaWAN", VERSION, APP_VERSION_STRING);

  /** Initialize sensors */
  BSP_sensor_Init(&sensorConfig);

  /* Configure the Lora Stack*/
  LORA_Init(&LoRaMainCallbacks, &LoRaParamInit, &lora_config, GET_LORA_SPI());

  PRINTF("Start joining" NL NL);
  LORA_Join();

  LoraStartTx();

  /* main loop*/
  while (1) {
    if (txMsgFlag) {
      txMsgFlag = false;
      TimerInit(&TxLedTimer, OnTimerLedEvent);
      TimerSetValue(&TxLedTimer, BLINK_TIME);
      LED_On(txMsgAck ? LED_GREEN : LED_RED) ;
      TimerStart(&TxLedTimer);
      PRINTF("Message => ack %u, cnt %u" NL, txMsgAck, txMsgFrmCnt);
    }

    DISABLE_IRQ();
    /* if an interrupt has occurred after DISABLE_IRQ, it is kept pending
     * and cortex will not enter low power anyway  */
#ifndef LOW_POWER_DISABLE
    LPM_EnterLowPower();
#endif
    ENABLE_IRQ();
  }
}

static void LORA_HasJoined(void)
{
  if (lora_config.otaa != 0) {
    PRINTF("JOINED" NL);
  }
  LORA_RequestClass(lora_config.class);
}

static void LoraTxData(lora_AppData_t* AppData)
{
  if (LORA_JoinStatus() != LORA_SET) {
    // Not joined, try again later
    return;
  }
  // Maximum buffer size
  uint8_t length = LORAWAN_APP_DATA_BUFF_SIZE;

  TimerInit(&TxLedTimer, OnTimerLedEvent);
  TimerSetValue(&TxLedTimer, BLINK_TIME);

  const sensor_t* sensors = BSP_sensor_Read();

  uint8_t i = 0;
  uint8_t sensor_nr;
  uint8_t stream_cnt = 0;
  int8_t n = 0;

  for (sensor_nr = 0; sensor_nr < MAX_SENSORS; sensor_nr++) {
    switch (sensors[sensor_nr].type) {
    case eNoSensor:
      // break will only abort case, we need to end loop
      goto sensors_done;

    case eSHT21:
      n = LPP_Temperature(stream_cnt++, sensors[sensor_nr].data.humidity.t, AppData->Buff + i, &length);
      if (n > 0) {
        i += n;
        n = LPP_Humidity(stream_cnt++, sensors[sensor_nr].data.humidity.rh, AppData->Buff + i, &length);
      }
      break;

    case eSiLab:
      n = LPP_Temperature(sensor_nr, sensors[sensor_nr].data.humidity.t, AppData->Buff + i, &length);
      if (n > 0) {
        i += n;
        n = LPP_Humidity(sensor_nr, sensors[sensor_nr].data.humidity.rh, AppData->Buff + i, &length);
      }
      break;

    case eUS100:
      n = LPP_Analog(sensor_nr, sensors[sensor_nr].data.distance.d, AppData->Buff + i, &length);
      break;

    case eTempInt:
    case eTmp112Ext:
    case eTmp112:
    case eDS18B20:
    case eSTS21: {
      int16_t t = sensors[sensor_nr].data.temperature.t;

      if (t >= sensorConfig.upperThreshold) {
        alarm_hi |= 1 << sensor_nr;
      } else if (t <= (sensorConfig.upperThreshold - sensorConfig.hysteresis)) {
        alarm_hi &= ~(1 << sensor_nr);
      }
      if (t <= sensorConfig.lowerThreshold) {
        alarm_lo |= 1 << sensor_nr;
      } else if (t >= (sensorConfig.lowerThreshold + sensorConfig.hysteresis)) {
        alarm_lo &= ~(1 << sensor_nr);
      }

      n = LPP_Temperature(stream_cnt++, sensors[sensor_nr].data.temperature.t, AppData->Buff + i, &length);
      break;
    }

    default:
      break;
    }
    if (n > 0) {
      i += n;
    }
  }

sensors_done:
  do {
    uint8_t batteryLevel = HW_GetBatteryLevel();
    n = LPP_Digital(stream_cnt, batteryLevel, AppData->Buff + i, &length);
    i += n;
    length -= n;

    // if interval counter is expired, we send anyway (heartbeat)
    bool doSend = false;
    if (intervalCnt++ >= interval) {
      intervalCnt = 0;
      doSend = true;
    }

    if (doSend || alarm_hi || alarm_lo) {
      PRINTF("LoRaTx" NL);
      AppData->BuffSize = i;
      AppData->Port = lora_config.application_port;
      AppData->nbTrials = LoRaParamInit.NbTrials;

      LORA_send(AppData, lora_config.ReqAck);
      LED_On(LED_BLUE) ;
      TimerStart(&TxLedTimer);
    } else { PRINTF("Not sending..." NL); }
  } while (0);
}

static void LORA_RxData(lora_AppData_t* AppData)
{
  if (AppData->Port == lora_config.application_port) {
    if (AppData->BuffSize == 1) {
      AppLedStateOn = AppData->Buff[0] & 0x01;
      if (AppLedStateOn == RESET) {
        PRINTF("LED OFF" NL);
        LED_Off(LED_BLUE) ;

      } else {
        PRINTF("LED ON" NL);
        LED_On(LED_BLUE) ;
      }
    }
  }
}

static void LORA_ConfirmClass(DeviceClass_t Class)
{
  PRINTF("switch to class %c done" NL, "ABC"[Class]);

  // Optional: informs the server that switch has occurred ASAP
  AppData.BuffSize = 0;
  AppData.Port = lora_config.application_port;

  LORA_send(&AppData, LORAWAN_UNCONFIRMED_MSG);
}

static void LORA_Confirmation(uint32_t frmCnt, bool ack)
{
  txMsgAck = ack;
  txMsgFrmCnt = frmCnt;
  txMsgFlag = true;
}

static void OnTxTimerEvent(void)
{
  // on fully timer driven applications trigger send
  LoraTxData(&AppData);

  // and restart timer
  TimerStart(&TxTimer);
}

static void LoraStartTx()
{
  // Set timer in timer based applications
  // or configure event
  TimerInit(&TxTimer, OnTxTimerEvent);
  TimerSetValue(&TxTimer, lora_config.TxDutyCycleTime);
  OnTxTimerEvent();
}

static void OnTimerLedEvent(void)
{
  LED_Off(LED_RED);
  LED_Off(LED_GREEN);
  LED_Off(LED_BLUE);
}

