
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"
#include "lwip/api.h"
#include <string.h>

void Error_Handler(void);
uint32_t rll4b6b_encode(uint8_t *in, uint16_t byteLen, uint8_t *out);
uint32_t modulatePacket(
         uint8_t *payload,   // The payload (4B6B RLL is already done)
         uint32_t data_size, // The actual data size without RLL. This is used to generate the header
         uint32_t size,      // The payload size in bytes (includes RLL)
         uint16_t halfPeriod, uint16_t macDim,  // Period and dimming info
         uint16_t *timBuffer,     // Buffer where the waves are output
         uint16_t *altTimBuffer,  // alternate waves buffer where the modified header is stored. Used for retransmitions of the same frame
         uint16_t **size2TimBuff  // returns the pointer in timBuffer, where the alternate header will be written for retransmitions of the same frame
         );

extern TIM_HandleTypeDef htim1;

extern osMessageQueueId_t newFlitQHandle;
extern osSemaphoreId_t getNextFlitHandle;

extern const uint16_t halfPeriod;   
extern uint16_t macDim; // dimming factor 0-1000.

extern IWDG_HandleTypeDef hiwdg1;

extern volatile uint16_t *timBufferPtr[2];  // pointers to the wave(form) buffers
extern uint16_t *altTimBufferPtr[2];  // pointers to the alt wave(form) buffers
extern uint16_t *size2TimBuffPtr[2];  // points to the pulse in timBuffer where we sneak in the above pulses

uint8_t rllPacket[2048*3/2];  // Buffer for the RLL encoded data


FlitInfo_t process_data(struct netbuf *buf, uint32_t rxLen);

// ----------------------------------------------------------------------------

void vlcXmit(void *argument)
{
  uint8_t bufIdx = 0;  // Index of the buffer to use
  uint8_t repCounter = 0;  // Number of times to repeat the VLC xmission
  FlitInfo_t flitInfo = {   // Struct to hold the VLC xmission info
    .repLimit = 0,
    .waveSize = 0
  };

  uint8_t reqIsDue = 0;  // Flag to indicate if a new request is due

  // Get the first VLC xmission info
  if (osOK != osMessageQueueGet(newFlitQHandle, &flitInfo, NULL, osWaitForever)) {
    Error_Handler();
  }

  while(1) {
    // Wait for the timer to complete the current VLC xmission
    osThreadFlagsWait(TIMER_COMPLETE_FLAG, osFlagsWaitAll, osWaitForever);

    // ---- First, stop the current PWM output. ------------------
    //  We need the following to stop "seeing" waveforms
    if (HAL_TIM_DMABurst_WriteStop(&htim1, TIM_DMA_UPDATE) == HAL_ERROR) {
      // In the following case HAL_TIM_DMABurst_WriteStop doesn't disable this DMA channel, and
      //  I get longer gaps in the VLC signal when I restart the timer - dma
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
      if (htim1.hdma[TIM_DMA_ID_UPDATE]->ErrorCode == HAL_DMA_ERROR_NO_XFER) {
        __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_UPDATE);
      } else {
        // Other error, light LD2 to debug
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
      }
    }
    repCounter += 1;  // Count the VLC xmission

    if (repCounter == 1) {  // The firstXmission is complete
      // Insert the special "duplicate" mark in the current waveform
      memcpy(size2TimBuffPtr[bufIdx], altTimBufferPtr[bufIdx], 12); // Copy the 2 pulses (x3 int16_u each)
      // This memory is un-cacheable, so no need to flush the cache.
      // If we reached the limit and have a new packet, this is wasted, but it is not a slow operation.
    }
    reqIsDue = 0;
    if (repCounter >= flitInfo.repLimit) {
      if (repCounter == flitInfo.repLimit)
        reqIsDue = 1; // Check if we need to send a request for the next flit, but don't send it yet.
      // Check if there is a new flit to transmit, but don't block waiting for it.
      //  we re-transmit the one we've got if there is no new flit.
      // The existing flitInfo is not touched if the queue is empty.
      if (osOK == osMessageQueueGet(newFlitQHandle, &flitInfo, NULL, 0)) {
        repCounter = 0;
        bufIdx = (bufIdx + 1) % 2;  // Switch the buffer index
        if (flitInfo.repLimit == 1) {
          reqIsDue = 1; // Send new request immediately if we transmit only once
          //  The reason for this is that we want to get and modulate the next flit, while the current one
          //   is being transmitted on the VLC channel.
          flitInfo.repLimit = 0; // This will prevent us releasing the semaphore again when the timer expires
        }
      }
    }

    // Restart the PWM signal generation. It may be a repeat of the previous or a new one
    if (HAL_TIM_DMABurst_MultiWriteStart(&htim1, TIM_DMABASE_ARR, TIM_DMA_UPDATE,
                                (uint32_t *) timBufferPtr[bufIdx], TIM_DMABURSTLENGTH_3TRANSFERS, flitInfo.waveSize) != HAL_OK) {
      Error_Handler();
    }

    // Send the REQ if it is due
    if (reqIsDue != 0) {
      /* Restart the watchdog counter 
      */
      if (HAL_IWDG_Refresh(&hiwdg1) != HAL_OK) {
        Error_Handler();
      }
      if (osSemaphoreGetCount(getNextFlitHandle) < 1) {
        // For tiny packets, the other task may not have aquired the previous semaphore yet.
        osSemaphoreRelease(getNextFlitHandle);
      }
    }
  }
}


// ----------------------------------------------------------------------------

const uint8_t tooLong[10] = "Too long\n";

// The buffer number used to store the modulated data
//  used in rcvModulate and process_data, thus global.
static uint8_t  curRcvBuf = 0;



// Receing TCP task - we use the default task as it needs to stay alive.
void StartDefaultTask(void *argument)
{
  LWIP_UNUSED_ARG(argument);

  ip4_addr_t server_ip; 
  uint8_t  reqMessage[7];
  struct netconn *inConn;  
  struct netbuf  *buf;
  uint8_t firstTime = 1;

  /* init code for LWIP */
  MX_LWIP_Init();
  
  // Wait long enough for the IP address to be assigned
  osDelay(5000);

  // Change this for the server used.
  myIP4_ADDR(&server_ip);

  // Prepare the request message - it doesn't change
  uint32_t devID = HAL_GetDEVID();
  reqMessage[0] = 'R';
  reqMessage[1] = 'E';
  reqMessage[2] = 'Q';
  // not bothered with byte order, as I won't use the id for now.
  memcpy(&reqMessage[3], (uint8_t *) &devID, 4);
  
  while (1) {
    err_t err;
    /* Create a new connection identifier. */
    inConn = netconn_new(NETCONN_TCP);
    if (inConn == NULL)
      continue;

    /* Tell connection to go into listening mode. */
    err = netconn_connect(inConn, &server_ip, SERVER_PORT_NUMBER);
    if (err != ERR_OK) {
      // On error, loop and try again
      netconn_delete(inConn);
      continue;
    }
    // Send the initial request for data to xmit.
    //  this is out of the loop. I don't need to wait for a semaphore.
    if ((err = netconn_write(inConn, reqMessage, 7, NETCONN_COPY)) != ERR_OK) {
      netconn_delete(inConn);
      continue;
    }  
    while ((err = netconn_recv(inConn, &buf)) == ERR_OK) {
      FlitInfo_t flitInfo;
      uint32_t xLen = netbuf_len(buf);  // total length of data in buffer
      
      if (xLen > INC_DATA_SIZE) {
        // Invalid flit received, notifiy the sender, ignore the flit and wait for the next one
        netconn_write(inConn, tooLong, 10, NETCONN_COPY);
        netbuf_delete(buf);
        continue;
      }
      
      // RLL and Modulate the data
      flitInfo = process_data(buf, xLen);
      netbuf_delete(buf);

      if (osOK != osMessageQueuePut(newFlitQHandle, &flitInfo, 0, osWaitForever)) {
        Error_Handler();
      }

      // Special case for the very first VLC transmission
      if (firstTime == 1) {
        firstTime = 0;
        // Start watchdog. We wait for the 1st flit arrival from ETH, to avoid continuous resets 
        //  before the server initiates transmissions on ETH
        // Counter Reload Value = (LsiFreq(Hz) * Timeout(ms)) / (prescaler * 1000) 
        // LsiFreq = 32kHz, 
        hiwdg1.Instance = IWDG1;
        hiwdg1.Init.Prescaler = IWDG_PRESCALER_16;
        hiwdg1.Init.Window    = 0x0FFF;  // Set to max: I don't want a reset window
        hiwdg1.Init.Reload   = (32000 * 1000) / (8 * 1000);  // 1sec, 0x7d0.
        HAL_StatusTypeDef t = HAL_IWDG_Init(&hiwdg1);
        if (t != HAL_OK) {
          //printf("IWDG init failed %d\n", t);
          Error_Handler();
        }
#if 0
#endif
        // Start the timer in PWM mode
        if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) {
          Error_Handler();
        }
        // Must start PWM now. I'll get an interrupt when xmition is finished
        if (HAL_TIM_DMABurst_MultiWriteStart(&htim1, TIM_DMABASE_ARR, TIM_DMA_UPDATE,
                                    (uint32_t *) timBufferPtr[0], TIM_DMABURSTLENGTH_3TRANSFERS, flitInfo.waveSize) != HAL_OK) {
          Error_Handler();
        }
      } // end firstTime
      else {  // for the first time, don't wait for the 1st xmission to request again
        // Wait for VLC xmission to start. Then send another request
        if (osOK != osSemaphoreAcquire(getNextFlitHandle, osWaitForever)) {
          Error_Handler();
        }
      }
      if ((err = netconn_write(inConn, reqMessage, 7, NETCONN_COPY)) != ERR_OK) {
        break;  // break out of this loop and reconnect in the outer loop
      }  
      curRcvBuf = (curRcvBuf+1) % 2;  // toggle timer waves buffer every time
    }  // End inner loop
    // Close connection and go to outer loop to re-connect.
    netconn_close(inConn);
    netconn_delete(inConn);
  } // end outer loop (forever)
}


// process (rll encode and modulate) all data from a chain of netbuf
FlitInfo_t process_data(struct netbuf *buf, uint32_t rxLen)
{
  uint32_t   rllSize = 0;
  FlitInfo_t flitInfo = {   // Struct to hold the VLC xmission info
    .repLimit = 0,
    .waveSize = 0
  };

  // RLL encode the data
  do {
    uint8_t  *cur_out = rllPacket;
    void     *data;
    uint32_t outSize;
    uint16_t len;
    uint8_t  first_buf = 1;

    netbuf_data(buf, &data, &len);
    if (first_buf) {
      first_buf = 0;
      // the 1st byte is the number of (VLC) trasmissions, not real data
      flitInfo.repLimit = *((uint8_t *) data);
      len -= 1;
      data = (uint8_t *) data + 1;
    }
    if (len % 2 != 0)
      Error_Handler();  // should not happen...
    outSize = rll4b6b_encode((uint8_t *) data, len, cur_out);
    cur_out += outSize;
    rllSize += outSize;
  } while (netbuf_next(buf) != -1);
  netbuf_first(buf);  // reset the buffer to the first part

  // Modulate the packet payload and heafer, after the, already prepared, preample
  flitInfo.waveSize = modulatePacket(rllPacket, rxLen-1,  // remove the repetition count byte
                         rllSize, halfPeriod, macDim,
                         timBufferPtr[curRcvBuf]+30,  // the first 30 are the preample
                         altTimBufferPtr[curRcvBuf], &size2TimBuffPtr[curRcvBuf]);
  if (flitInfo.waveSize < 0) {
    //printf("modulate returned < 0\n");
    Error_Handler();
  }
  flitInfo.waveSize += 30;
  if (flitInfo.waveSize > MAX_PULSES) {
    Error_Handler();
  }

  return flitInfo;
}
