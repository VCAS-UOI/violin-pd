#include "cmsis_os.h"
#include "demodulate.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "printf.h"

extern void MX_TIM2_Init(void);

typedef struct {
  uint8_t command;
  uint8_t param;
} usbCommand_t;

typedef struct {
  // Trick to align the 1st element of the struct to the cache line size
  // https://stackoverflow.com/questions/11558371/is-it-possible-to-align-a-particular-structure-member-in-single-byte-aligned-str
  struct {} __attribute__ ((aligned (__SCB_DCACHE_LINE_SIZE)));
  uint8_t pbuff[MAX_PACKET_SIZE];  // storage for the packet
  uint8_t isNew;
  uint16_t size;
  uint16_t numGlitches;    // number of shorter pulses
  uint16_t numBadSymbols;  // number of bad 4b6b symbols
	} packetHandler_t;


extern osMessageQueueId_t usbCmdQHandle;
extern osMessageQueueId_t flitQHandle;
extern osMessageQueueId_t freeQHandle;
extern osThreadId_t USB_XMITHandle;

uint8_t filter; // Passed on to the timer initialization function

// I need this for the DMA to work. The default RAM location is not accessible 
//  by the DMA!
/* AHB SRAM (D2 domain): */
#define __SECTION_TIMER_IC_DATA \
  __attribute__((section(".timerIC")))

//extern WWDG_HandleTypeDef hwwdg;
//extern uint16_t wdCounter;

extern TIM_HandleTypeDef htim2;  // tim2 for input capture

demodulationHandleDef  dh;
__attribute__((section(".RAM_D1"))) packetHandler_t fifo[FLIT_Q_SIZE];

uint8_t usbEnableXmit;

// This is the input capture data from the timer peripheral
__SECTION_TIMER_IC_DATA ALIGN_32BYTES(__IO uint32_t   timerBuffer[2*HALF_BUFF_SIZE]); 

uint16_t macDim = 500;  // Dimming value 0 - 10000

/* Counters for events. Used for measuring quality - performance
 */
uint32_t 
         cnt_inGaps,     // Number of inter-flit gaps dectected
         cnt_inFlits,    // Number of flit-starts: i.e. the FLP was complete
         cnt_tdpErrors,  // Unexpected pulse in TDP or from FLP to TDP. These flits are dropped
         cnt_demodFlits, // Number of flits successfully completed - should be equal to flits received by the app through the USB
         cnt_badSymbols, // Number of bad 4B6B symbols. Some may be recoverable by RS decode in the app
         cnt_fecDroppedFlits,  // Not used as FEC is moved to app
         cnt_glitches, cnt_doubleGlitches;  // Not used.
//uint32_t glitchWidths[20];


// ------------------------------------------------------------
// Waits for start command from USB host
// Gets buffer space (index into the "fifo" array) from the freeQ
// Initializes a demodulation handler structure
// Starts timer in input capture mode and DMA transfers, which cause task notifications
//   every time half of the buffer is filled
// The first event is used to measure the delays. Essentially we detect the transmitter's frequency. These are required for the demodulation

// IL: The inner loop blocks waiting for such an event, 
//     Demodulate the samples in the buffer
//     if USB host wants us to stop receiving, 
//           stop timer and DMA
//           break into outer loop which waits fo USB receiver to enable VLC reception again
//     if the demodulation function does not return a full packet, keep looping
//     if we have a packet, 
//        send fifo index to the USB xmit task
//     get a new free buffer space (freeQ)
//     initialize the demodulation handler using the new buffer space
//     demodulate fany remaining samples in the buffer
// ------------------------------------------------------------
void task_dmaReceive(void *args __attribute((unused)))
{
  usbCommand_t cmd;  // The "command" from the USB host

  uint8_t  firstTime = 1;
  uint8_t  skipEntry;
  uint8_t  c; // current working fifo entry
  int32_t  rcode;  // return code from decoder - size of the flit
  uint32_t dmaComplete;
  uint16_t circBufOffset;

  // Dimming using the algorithm in clause 8.5.2.3 of IEEE std
  dh.dim_low  = (uint8_t) floorf(macDim / 100.0) * 10;  // Range: 0-100 with an 
  dh.dim_high = (uint8_t) ceilf(macDim / 100.0) * 10;   //  increment of 10. (0, 100 are not usable)
  uint8_t rep2 = macDim - 10 * dh.dim_low;
  dh.rep1 = 100 - rep2;
  dh.dim = dh.dim_low;

  
  printf("VLC receiver POR.\n");
  // Fill the freeQ. All fifo entries are free at the beginning.
  for (uint16_t i = 0; i < FLIT_Q_SIZE; i++) {
    osMessageQueuePut(freeQHandle, &i, 0, 0);
  }

  while (1) {
    do {
      osMessageQueueGet(usbCmdQHandle, &cmd, 0, osWaitForever);
    } while (cmd.command != 'S');  // Wait for the start command to start the receiver
    // Set the IC timer filter from the PC application
    filter = cmd.param;
    // Initialize the timer peripheral
    MX_TIM2_Init();
    printf("Starting VLC receiver with filter %d.\n", filter);

    // Get fifo index for first packet
    if (osMessageQueueGet(freeQHandle, &c, 0, osWaitForever) != osOK) {
      Error_Handler();
    }
    dh.state = MAN_ERROR_RECOVERY;
    initDemodulation(&dh, fifo[c].pbuff+3);  // +3 to skip the first 3 bytes of the buffer 

    // Start timer input capture DMA 
    if (HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *) timerBuffer, 2*HALF_BUFF_SIZE) != HAL_OK) {
      Error_Handler();
    }

    // ------------------- Measure the gap ---------------------------------
    // All pulses, but the gap, are in one of two categories and the sum of their widths are
    //  always equal to the optical clock period. They should also be approx. the same number of
    //  short and long pulses. Therefore the average * 2, should be the period * 1.5.
    // Any longer pulse will be the gap
    // I remove the very first value because there is a huge gap if the receiver started running
    //  before the xmiter.

    // Block, waiting for DMA interrupt (any of halfComplete or complete)
    // 1 - halfComplete, 2 - complete
    dmaComplete = osThreadFlagsWait(0x03, osFlagsWaitAny, osWaitForever);  // Wait for the last USB xmit to finish

    // Select buffer index (low or upper half) based on which part of the buffer is complete.
    circBufOffset = ((dmaComplete & 0x01) != 0)? 0 : HALF_BUFF_SIZE;
    // Invalidate Data Cache to get the updated content of the SRAM on the timer buffer 
    SCB_InvalidateDCache_by_Addr((uint32_t *) &timerBuffer[circBufOffset], 4*HALF_BUFF_SIZE);  // 2nd argument in bytes, hence the 4*
    // Remove the first sample as it could correspond to the initial lights off time
    __IO uint32_t *buf = &timerBuffer[circBufOffset+1];
    uint32_t sum = 0;
    uint32_t maxPulse = 0;
    for (uint16_t i = 0; i < HALF_BUFF_SIZE-1; i++) {
      if (*buf > maxPulse) {
        maxPulse = *buf;
      }
      sum += *buf++;
    }
    IDLE_SAMPLE_THRES = 2 * sum / (HALF_BUFF_SIZE - 1); 
    printf("Minimum gap  set to %d, max seen %d\n", IDLE_SAMPLE_THRES, maxPulse);
    // ------------------- end Measure the gap ---------------------------------

    /* Clear event counters
    */
    cnt_inGaps = cnt_inFlits = cnt_tdpErrors = cnt_demodFlits = cnt_fecDroppedFlits = cnt_badSymbols = cnt_glitches = cnt_doubleGlitches = 0;

    // timer IC loop: Only way out is when usb orders us to stop.
    while (1) {
      // Block, waiting for DMA interrupt (any of halfComplete or complete)
      dmaComplete = osThreadFlagsWait(0x03, osFlagsWaitAny, osWaitForever);  // Wait for the last USB xmit to finish
#if 0
      if (firstTime) {
        firstTime = 0;
        /* Init & Start WWDG peripheral ######################################*/
        //  Set all to max, so the timeout is long.
        //  120000000
        hwwdg.Instance = WWDG1;
        hwwdg.Init.Prescaler = WWDG_PRESCALER_64;
        hwwdg.Init.Window    = 0x7F;   // largest possible window
        hwwdg.Init.Counter   = 0x7F;
        hwwdg.Init.EWIMode   = WWDG_EWI_ENABLE;
        if (HAL_WWDG_Init(&hwwdg) != HAL_OK) {
          Error_Handler();
        }
	  /*
	  */
      } else {
        if (HAL_WWDG_Refresh(&hwwdg) != HAL_OK) {
          Error_Handler();
        }
	      wdCounter = 0;
      }
#endif
      circBufOffset = ((dmaComplete & 0x01) != 0)? 0 : HALF_BUFF_SIZE;
      // Invalidate Data Cache to get the updated content of the SRAM on the timer buffer 
      SCB_InvalidateDCache_by_Addr((uint32_t *) &timerBuffer[circBufOffset], 4*HALF_BUFF_SIZE);  // 2nd argument in bytes, hence the 4*

	    // demodulate, returns negative for error, 0 for not complete packet, or the size of packet in bytes
      rcode = demodulate(&timerBuffer[circBufOffset], &dh);

      /*
      if  (cnt_inFlits % 100 == 0)
         printf("Gaps: %d, Incoming: %d, Dedomulated: %d, FEC dropped: %d\n",
           cnt_inGaps, cnt_inFlits, cnt_demodFlits, cnt_fecDroppedFlits);
      */

      if (rcode < 0) // Error
        Error_Handler();  // This implies an "exit". Execution will not continue

      // Check if we should stop receiving. Moved this soon after demodulate (half buffer processing)
      //  It makes sense because the host may stop the receiver at any time, not only after a full flit.
      if (osOK == osMessageQueueGet(usbCmdQHandle, &cmd, 0, 0)) {
        if (cmd.command == 's') {  // Got a stop command
          // Stop timer IC 
          if (HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_1) != HAL_OK)
            Error_Handler();

          printf("Stopping VLC receiver with space in freeq %d in flitQ %d.\n", osMessageQueueGetSpace(freeQHandle), osMessageQueueGetSpace(flitQHandle));
          // Clear the queues.
          osMessageQueueReset(flitQHandle);
          osMessageQueueReset(freeQHandle);
          // Fill the freeQ. All fifo entries are free at the beginning.
          for (uint16_t i = 0; i < FLIT_Q_SIZE; i++) {
            osMessageQueuePut(freeQHandle, &i, 0, 0);
          }
          // Print counters to debug serial
          printf("Gaps %d\n", cnt_inGaps);     // Number of inter-flit gaps detected
          //printf("Wd %d\n", wdCounter);
          printf("Flits started %d\n", cnt_inFlits);    // Number of flit-starts: i.e. the FLP was complete
          printf("FLP-TDP errors %d\n", cnt_tdpErrors);  // Unexpected pulse in TDP or from FLP to TDP. These flits are dropped
          printf("Demodulated Flits %d\n", cnt_demodFlits); // Number of flits successfully completed - should be equal to flits received by the app through the USB
          printf("Total bad symbols %d\n", cnt_badSymbols); // Number of bad 4B6B symbols. Some may be recoverable by RS decode in the app
          // ----------------------
          break; // break to outer loop to block on waiting for usb host to re-enable reception
        }
      }

      if (rcode == 0) { // not a full packet yet
        continue;      // go block on task notification at the top of inner loop
      }
      // ---------------------------------------------------------
      // ------------ Got a complete flit ------------------------
      // Count it
      cnt_demodFlits += 1;

      fifo[c].size = rcode;
      fifo[c].isNew = dh.isNewFlit;
      fifo[c].numGlitches = dh.num_glitches;
      fifo[c].numBadSymbols = dh.num_badSymbols;
      if (osMessageQueuePut(flitQHandle, &c, 0, 0) != osOK) {
        // In the unlikely event that the queue is full, drop the flit.
        printf("send Q failed\n");
        //Error_Handler();
      }

      // Get a new free buffer space. If not available, panic!
      if (osMessageQueueGet(freeQHandle, &c, 0, osWaitForever) != osOK) {
        Error_Handler();
      }
      // Clear structs for new flit, but not for circular buffer pointer
	    initDemodulation(&dh, fifo[c].pbuff+3);
	    // Go back to process the rest of the DMA buffer. dh has all the info about pointers to circular buffer etc
	    //   Assumption: the DMA (half) buffer is not large enough to hold 2 packets! This 2nd call should always return 0.
	    if (demodulate(NULL, &dh) != 0)
	      Error_Handler(); 
    }  // end of inner loop (the host has enabled the receiver)
  }  // end of outer loop
}

// ----------------------------------------------------------------------------
// Transmit flit to USB
// ----------------------------------------------------------------------------
void task_usbXmit(void *args __attribute((unused)))
{
  uint8_t c;    // fifo index of packet to be transmitted to USB

  //MX_USB_DEVICE_Init();

  while (1) {
    // wait for a complete flit to be received from VLC
    osMessageQueueGet(flitQHandle, &c, 0, osWaitForever);

    // Assume 3 bytes in pbuf are available ... This should be done in the other task....
    fifo[c].pbuff[0] = 0;  // Start with a 0 to signal a the start of new data on serial pipe.
    fifo[c].pbuff[2] = (fifo[c].size >> 8) & 0xff;
    // Mark new flits with MSB of size... Not used in the app, but it is here.
    if (fifo[c].isNew)
      fifo[c].pbuff[2] |= 0x80;
    fifo[c].pbuff[1] = (fifo[c].size & 0xff);

    // Flush the cache,
    SCB_CleanDCache_by_Addr((uint32_t *) &fifo[c].pbuff, fifo[c].size+3);

    int ret;
    // Send the whole thing in one go to take advantage of the USB DMA 
    do {
      ret = CDC_Transmit_FS(fifo[c].pbuff, fifo[c].size+3);
      if (ret != USBD_OK)    // check for USBD_FAIL???
          ;//printf("UFM\n");
    } while (ret != USBD_OK);

    // Block until the USB xmit is done. No need for concurency here: we do nothing else in this task.
    osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);  // Wait for the last USB xmit to finish

    // Release the buffer into the free Queue
    osMessageQueuePut(freeQHandle, &c, 0, 0); // Should not block.
  }
}

void pseudotask_usbRcv(uint8_t *buf)
{
  // This function is called when a new packet of length>0 is received from USB

  usbCommand_t cmd;  // Queues copy the data, so it can be on the stack
  cmd.command = *buf;
  cmd.param = *(buf+1);
  printf("Got USB cmd %c %d\n", cmd.command, cmd.param);
  if (osMessageQueueGetSpace(usbCmdQHandle) != 0) {
    osMessageQueuePut(usbCmdQHandle, &cmd, 0, 0); // Cannot block. This is an ISR
  }
  // Ignore the command comming from the host, if the queue is full
}

/* USER CODE END Application */
