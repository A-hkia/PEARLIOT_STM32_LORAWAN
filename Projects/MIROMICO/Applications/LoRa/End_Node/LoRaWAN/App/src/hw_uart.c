/******************************************************************************
 * @file    HW_UART1.c
 * @author  MCD Application Team
 * @version V1.0.2
 * @date    15-November-2016
 * @brief   manages virtual com port
 ******************************************************************************
 */

#include "hw.h"
#include "hw_uart.h"
#include "hw_gpio.h"
#include <stdarg.h>
#include "stm32l0xx_ll_usart.h"
#include "stm32l0xx_ll_bus.h"
#include "FMLR72_L0.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
#define IS_USART1(h)  ((h) == &uart1Context)

#define BUFSIZE 256

/* Private variables ---------------------------------------------------------*/
typedef struct {
  USART_TypeDef* usart;
  char buffTx[BUFSIZE];               /**< buffer to transmit */
  char buffRx[BUFSIZE];               /**< Circular buffer of received chars */
  int rx_idx_free;                    /**< 1st free index in BuffRx */
  int rx_idx_toread;                  /**< next char to read in buffRx, when not rx_idx_free */

  uint8_t rxOn;                       ///< Flag to check whether reception is on (needed to handle pins for low power)
} sUartContext_t;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static sUartContext_t uart1Context;
static sUartContext_t uart2Context;

/* Functions Definition ------------------------------------------------------*/

void* HW_UART1_GetHandle()
{
  return &uart1Context;
}

void* HW_UART2_GetHandle()
{
  return &uart2Context;
}

void HW_UART_IoInit(void* handle)
{
  //    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  //
  //    /* Configure Tx Pin as : Alternate function, High Speed, Push pull, Pull up */
  //    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
  //    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_9, LL_GPIO_AF_4);
  //    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_VERY_HIGH);
  //    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_PUSHPULL);
  //    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_9, LL_GPIO_PULL_UP);
  //
  //    /* Configure Rx Pin as : Alternate function, High Speed, Push pull, Pull up */
  //    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
  //    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_10, LL_GPIO_AF_4);
  //    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_VERY_HIGH);
  //    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_PUSHPULL);
  //    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_10, LL_GPIO_PULL_UP);

  GPIO_InitTypeDef  GPIO_InitStruct = {0};
  if (IS_USART1(handle)) {
#if defined(USE_USART1)
    /* UART TX GPIO pin configuration  */
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = USART1_TX_AF;

    HW_GPIO_Init(USART1_TX_GPIO_PORT, USART1_TX_PIN, &GPIO_InitStruct);

    /* UART RX GPIO pin configuration  */
    if (uart1Context.rxOn == 1) {
      GPIO_InitStruct.Alternate = USART1_RX_AF;
    } else {
      GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
    }
    HW_GPIO_Init(USART1_RX_GPIO_PORT, USART1_RX_PIN, &GPIO_InitStruct);
#endif
  } else {
#if defined(USE_USART2)

    GPIO_InitTypeDef  GPIO_InitStruct = {0};
    /* UART TX GPIO pin configuration  */
    GPIO_InitStruct.Pin       = USART2_TX_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = USART2_TX_AF;

    HAL_GPIO_Init(USART2_TX_GPIO_PORT, &GPIO_InitStruct);

    /* UART RX GPIO pin configuration  */
    GPIO_InitStruct.Pin = USART2_RX_PIN;
    if (uart2Context.rxOn == 1) {
      GPIO_InitStruct.Alternate = USART2_RX_AF;
    } else {
      GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
    }

    HAL_GPIO_Init(USART2_RX_GPIO_PORT, &GPIO_InitStruct);
#endif
  }
}

void HW_UART_IoDeInit(void* handle)
{
  GPIO_InitTypeDef GPIO_InitStructure = {0};

  if (IS_USART1(handle)) {
#if defined(USE_USART1)
    USART1_TX_GPIO_CLK_ENABLE();
    USART1_RX_GPIO_CLK_ENABLE();

    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Pin =  USART1_TX_PIN;
    HAL_GPIO_Init(USART1_TX_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin =  USART1_RX_PIN;
    HAL_GPIO_Init(USART1_RX_GPIO_PORT, &GPIO_InitStructure);
#endif
  } else {
#if defined(USE_USART2)
    USART2_TX_GPIO_CLK_ENABLE();
    USART2_RX_GPIO_CLK_ENABLE();

    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStructure.Pull = GPIO_NOPULL;

    GPIO_InitStructure.Pin =  USART2_TX_PIN ;
    HAL_GPIO_Init(USART2_TX_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin =  USART2_RX_PIN ;
    HAL_GPIO_Init(USART2_RX_GPIO_PORT, &GPIO_InitStructure);
#endif
  }
}

void HW_UART_Init(void* handle, uint32_t br)
{
  sUartContext_t* c = (sUartContext_t*)handle;

  if (c == &uart1Context) {
    c->usart = USART1;
    NVIC_SetPriority(USART1_IRQn, 0);
    NVIC_EnableIRQ(USART1_IRQn);

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

#if defined(LL_RCC_USART2_CLKSOURCE_PCLK1)
    // Set clock source (only on L0 controllers)
    LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK2);
#endif
  } else if (c == &uart2Context) {
    c->usart = USART2;
    NVIC_SetPriority(USART2_IRQn, 0);
    NVIC_EnableIRQ(USART2_IRQn);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

#if defined(LL_RCC_USART2_CLKSOURCE_PCLK1)
    // Set clock source (only on L0 controllers)
    LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);
#endif
  } else {
    // wrong parameter => abort
    Error_Handler();
  }

  // common initialization
  c->rxOn = 0;
  HW_UART_IoInit(c);

  /* TX/RX direction */
  LL_USART_SetTransferDirection(c->usart, LL_USART_DIRECTION_TX_RX);

  /* 8 data bit, 1 start bit, 1 stop bit, no parity */
  LL_USART_ConfigCharacter(c->usart, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);

  LL_USART_SetBaudRate(c->usart, SystemCoreClock, LL_USART_OVERSAMPLING_16, br);

  LL_USART_Enable(c->usart);

#if defined(USART_ISR_TEACK)
  // Polling USART initialization (only on L0 controllers)
  while ((!(LL_USART_IsActiveFlag_TEACK(c->usart))) || (!(LL_USART_IsActiveFlag_REACK(c->usart))));
#endif
}

void HW_UART_DeInit(void* handle)
{
#if 1
  LL_USART_Disable(((sUartContext_t*)handle)->usart);
  HW_UART_IoDeInit(handle);

  if (IS_USART1(handle)) {
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  } else {
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  }
#endif
}

void HW_UART_SendBuffer(void* handle, const uint8_t* buffer, uint16_t size)
{
  sUartContext_t* c = (sUartContext_t*)handle;
  uint16_t sent = 0;

  // Send characters one per one, until last char was sent
  while (sent < size) {
    // Wait for TXE flag to be raised
    while (!LL_USART_IsActiveFlag_TXE(c->usart));
    // Write character in Transmit Data register.
    // TXE flag is cleared by writing data in TDR register
    LL_USART_TransmitData8(c->usart, buffer[sent++]);
  }
  while (!LL_USART_IsActiveFlag_TXE(c->usart));
}

void HW_UART_Send(void* handle, const char* format, ...)
{
  sUartContext_t* c = (sUartContext_t*)handle;

  va_list args;
  va_start(args, format);

  /*convert into string at buff[0] of length iw*/
  uint16_t iw = vsprintf(c->buffTx, format, args);

  HW_UART_SendBuffer(handle, (uint8_t*)c->buffTx, iw);

  va_end(args);
}

int8_t HW_UART_ReceiveBuffer(void* handle, uint8_t* buffer, uint16_t size, uint16_t timeout_ms)
{
  sUartContext_t* c = (sUartContext_t*)handle;
  uint32_t tickstart = 0;

  for (uint16_t s = 0; s < size; s++) {
    tickstart = HAL_GetTick();

    // wait for byte to be received
    while (!LL_USART_IsActiveFlag_RXNE(c->usart)) {
      if ((HAL_GetTick() - tickstart) >= timeout_ms) return -1;
    }

    buffer[s] = LL_USART_ReceiveData8(c->usart);
  }
  return 0;
}

void HW_UART_FlushRx(void* handle)
{
  sUartContext_t* c = (sUartContext_t*)handle;

  /* Clear flags, in case characters have already been sent to USART */
  LL_USART_ClearFlag_ORE(c->usart);
  LL_USART_ClearFlag_FE(c->usart);
  LL_USART_ReceiveData8(c->usart);
}

void HW_UART_ReceiveInit(void* handle, bool useInt)
{
  sUartContext_t* c = (sUartContext_t*)handle;
  HW_UART_FlushRx(handle);

  if (useInt) {
    /* Enable RXNE and Error interrupts */
    LL_USART_EnableIT_RXNE(c->usart);
    LL_USART_EnableIT_ERROR(c->usart);
  }
  c->rxOn = 1;
  HW_UART_IoInit(handle);
}



void HW_UART_DeInitReceive(void* handle)
{
  sUartContext_t* c = (sUartContext_t*)handle;

  /* Enable RXNE and Error interrupts */
  LL_USART_DisableIT_RXNE(c->usart);
  LL_USART_DisableIT_ERROR(c->usart);
  c->rxOn = 0;
}

FlagStatus HW_UART_IsNewCharReceived(void* handle)
{
  sUartContext_t* c = (sUartContext_t*)handle;
  FlagStatus status;

  BACKUP_PRIMASK();
  DISABLE_IRQ();

  status = ((c->rx_idx_toread == c->rx_idx_free) ? RESET : SET);

  RESTORE_PRIMASK();
  return status;
}

uint8_t HW_UART_GetNewChar(void* handle)
{
  sUartContext_t* c = (sUartContext_t*)handle;
  uint8_t NewChar;

  BACKUP_PRIMASK();
  DISABLE_IRQ();

  NewChar = c->buffRx[c->rx_idx_toread];
  c->rx_idx_toread = (c->rx_idx_toread + 1) % sizeof(c->buffRx);

  RESTORE_PRIMASK();
  return NewChar;
}

void HW_UART_Rx_Callback(void* instance)
{
  sUartContext_t* c;
  if (instance == uart1Context.usart) {
    c = &uart1Context;
  } else {
    c = &uart2Context;
  }

  // Read Received character. RXNE flag is cleared by reading of RDR register
  c->buffRx[c->rx_idx_free] = LL_USART_ReceiveData8(c->usart);
  int next_free = (c->rx_idx_free + 1) % sizeof(c->buffRx);
  if (next_free != c->rx_idx_toread) {
    /* this is ok to read as there is no buffer overflow in input */
    c->rx_idx_free = next_free;
  } else {
    /* force the end of a command in case of overflow so that we can process it */
    c->buffRx[c->rx_idx_free] = '\r';
    PRINTF("uart_context.buffRx buffer overflow %d" NL);
  }
}
