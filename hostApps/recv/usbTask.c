/*
  This file contains the code for the thread handling the USB connection to the nucleo receiver board
  Although a baud rate of 115200 is set on the serial port, the actual throughput is much higher.

  The libserialport library: https://sigrok.org/wiki/Libserialport is used for the serial communication.
  The Reed-Solomon decoder is from libcorrect: https://github.com/quiet/libcorrect Copyright (c) 2016, Brian Armstrong
  The COBS decoder is from the corresponding Wikipedia article https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing

  The rest of the code is (c) Aristides Efthymiou, VCAS lab, University of Ioannina, provided under the MIT license. See the top-level repository LICENSE file.
 */
#include <libserialport.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <arpa/inet.h>


#include <pthread.h>
#include <semaphore.h>
#include "vlcRecv.h"
#include "correct.h"

// These are defined in usbd_desc.c file in the Nucleo source code
#define NUCLEO_USBD_VID     1155
#define NUCLEO_USBD_PID    22336

// Globals shared with other thread
extern pthread_mutex_t task_lock;
extern sem_t  *queueSem;
extern uint8_t  allDone; //  flags to signal when the meta packets (0) has been parsed, when all received
extern uint8_t  verbLevel;  // verbosity level for messages
extern fileInfo fileDetails;
extern queueWrapper *qtail, *qhead;

struct sp_port *port;


// Globals not shared
correct_reed_solomon     *rs_usb;  // Reed-Solomon struct pointer

char enableString[2] = "S";
char stopString[] = "s";


void startSerialPort();


// Helper function for error reporting of serial port API calls.
int check(enum sp_return result)
{
  char *error_message;
 
  switch (result) {
  case SP_ERR_ARG:
    printf("USB_T: Error: Invalid argument %d.\n", result);
    //exit(EXIT_FAILURE);
    return result;
  case SP_ERR_FAIL:
    error_message = sp_last_error_message();
    printf("USB_T: Error: %d Failed: %s\n", sp_last_error_code(),  error_message);
    sp_free_error_message(error_message);
    return result;
  case SP_ERR_SUPP:
    printf("USB_T: Error: Not supported.\n");
    return result;
  case SP_ERR_MEM:
    printf("USB_T: Error: Couldn't allocate memory.\n");
    return result;
  case SP_OK:
  default:
    return result;
  }
}


// ---------------------------------------------------------------------------
// uint8_t *cobsDecode(int flitLen, uint8_t *rcvBuff, queueWrapper *newFb)
//   COBS decode. https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing
//   returns a decoded flit, or null if COBS failed.
//   the decoded message's length is set in the queueWrapper newFb struct.
// ---------------------------------------------------------------------------
uint8_t *cobsDecode(int flitLen, uint8_t *rcvBuff, queueWrapper *newFb)
{
  uint8_t  *decBuff;

  decBuff = malloc(flitLen);  // Slightly more than needed, but the waste is very small
  if (decBuff == NULL) {
    printf("USB_T: ERROR: failed to allocate data for (unstuffed) flit buffer\n");
    exit(EXIT_FAILURE);
  }
  const uint8_t *byte = rcvBuff; // Encoded input byte pointer
  uint8_t *decode = decBuff;     // Decoded output byte pointer

  for (uint8_t code = 0xff, block = 0; byte < rcvBuff + flitLen; --block) {
    if (block) // Decode block byte
      *decode++ = *byte++;
    else {
      if (code != 0xff) // Encoded zero, write it
        *decode++ = 0;
      block = code = *byte++; // Next block length
      if (code == 0x00) { // Delimiter code found
        if (verbLevel > 1) {
          printf("USB_T: COBS decode: delimiter code found before the end of data %ld flitLen %d\n", decode - decBuff, flitLen);
        }
        free(decBuff);
        free(rcvBuff);
        free(newFb);
        return NULL;
      }
    }
  }
  if (verbLevel > 2)
    printf("USB_T: COBS OK: len %ld original len %d\n", decode - decBuff, flitLen);
  free(rcvBuff);
  newFb->flitLen = decode - decBuff;  // Correct the lenght to remove COBS extra bytes
  newFb->flitStorage = decBuff;
  return decBuff;
}


// ---------------------------------------------------------------------------
// queueWrapper *USB_receiveHeader()
// Wait for a new USB "flit" to start (with a single byte of value 0), then
//  get the USB "flit" size and allocate a queueWrapper object.
// Set the USB flit size in flitLen member of the object.
// In case of errors, returns NULL
// ---------------------------------------------------------------------------
queueWrapper *USB_receiveHeader()
{
  uint8_t  headerBuff[2]; // buffer for flitSize
  uint8_t  codeBuff; // buffer for COBS delimiter
  queueWrapper *newFb;
  uint8_t  isDuplicate;
  int t;

  // ----------------------------------------------------------------------------
  // A. First wait for a new flit. Identified by a byte of zero value (COBS encoded)
  //    In case of error, return null
  // ----------------------------------------------------------------------------
  do {
    int tries = 0;
    if (check(sp_blocking_read(port, &codeBuff, 1, 0)) != 1) {
      if (verbLevel > 1) 
        printf("USB_T: Serial port failed waiting for COBS delimiter. Restarting serial port.\n");
      sleep(1); 
      startSerialPort();  // This blocks until the port is available
      sleep(3); 
      return NULL; // Back to caller empty-handed
    }
    if ((codeBuff != 0) && (tries == 0)) {
      if (verbLevel > 1) 
        printf("USB_T: Wrong start of flit code %x. Re-trying.\n", codeBuff);
    }
    tries = (tries+1) % 1000;
  } while (codeBuff != 0);
  
  // ----------------------------------------------------------------------------
  // B. Get the flit size and initialise flit storage
  // receive the flit size - 2 bytes, with blocking read
  // ----------------------------------------------------------------------------
  if ((t=check(sp_blocking_read(port, headerBuff, 2, 0))) != 2) {
    if (verbLevel > 2) 
      printf("USB_T: error reading flit size. Error: %d Size: %x %x. Restarting serial port\n",
           t, headerBuff[1], headerBuff[0]);
    startSerialPort();  // This blocks until the port is available
    return NULL;
  }

  isDuplicate = 0;
  // MSB of size will be set, if this is a different flit to the previously transmitted
  isDuplicate = ((headerBuff[1] & 0x80) == 0);
  headerBuff[1] &= 0x7F;

  // Allocate a new queue element
  newFb = malloc(sizeof(queueWrapper));
  if (newFb == NULL) {
    printf("USB_T: ERROR: failed to allocate a flit buffer.\n");
    exit(EXIT_FAILURE);
  }

  // Get the COBS length for now. It will be corrected later
  newFb->flitLen = headerBuff[0] | (headerBuff[1] << 8);
  if (verbLevel > 2)
    printf("USB_T: New Flit. size (with COBS) %d\n", newFb->flitLen);
  newFb->isDuplicate = isDuplicate;
  newFb->next = NULL;
  return newFb;
}


// ---------------------------------------------------------------------------
// queueWrapper *receiveFlit(struct sp_port *port)
// Receive a flit from USB and return a queueWrapper object containing it
// or NULL, if there has been trouble.
// ---------------------------------------------------------------------------
queueWrapper *receiveFlit(struct sp_port *port)
{
  uint8_t  *rcvBuff, *decBuff;
  int      bytesReceived, flitBytesReceived;
  uint32_t flitSeq;
  int      timeout = 1000;  // 1 sec timeout when receiving the flit data
  uint16_t blockSize;
  queueWrapper *newFb;

  // Block wait on serial port for a new flit and its size info 
  newFb = USB_receiveHeader();
  if (newFb == NULL) {
    return NULL;
  }
  // Allocate space for the payload
  rcvBuff = malloc(newFb->flitLen);
  if (rcvBuff == NULL) {
    printf("USB_T: ERROR: failed to allocate data for flit buffer\n");
    exit(EXIT_FAILURE);
  }

  // -------------- receive a complete flit from the serial port ----------------------
  uint8_t retry = 0;
  flitBytesReceived = 0;
  while (flitBytesReceived < newFb->flitLen) {
    bytesReceived = check(sp_blocking_read_next(port, rcvBuff+flitBytesReceived,
                                                newFb->flitLen-flitBytesReceived, timeout));
    if (bytesReceived <= 0) {
      retry++;
    } else {
      retry = 0;
    }
    if (retry == 5) {
      if (verbLevel > 1)
        printf("USB_T: Timeout while receiving data. Restarting serial port.\n");
      free(rcvBuff);
      free(newFb);
      startSerialPort();  // This will block until the USB/Serial port becomes available
      return NULL;  // This will return to outer loop to get a new flit
    }
    flitBytesReceived += bytesReceived;
    if (verbLevel > 2)
      printf("USB_T: Flit part - Got %d, total %d.\n", bytesReceived, flitBytesReceived);
  }

  // --- COBS decode. It also corrects newFb->flitLen (removes the extra COBS bytes)
  if ((decBuff = cobsDecode(newFb->flitLen, rcvBuff, newFb)) == NULL)
    return NULL;

  // --- Reed-Solomon decode 1st block, so I can trust the header info
  //  If it fails, reject the flit
  if (newFb->flitLen < RS_DATA_BLOCK_SIZE+RS_PARITY_SIZE)
    blockSize = newFb->flitLen-RS_PARITY_SIZE;
  else
    blockSize = RS_DATA_BLOCK_SIZE;
  if (correct_reed_solomon_decode(rs_usb, newFb->flitStorage, blockSize+RS_PARITY_SIZE, newFb->flitStorage) < 0) {
    if (verbLevel > 2)
      printf("USB_T: RS failed for block 0 of flit\n");
    free(decBuff);
    free(newFb);
    return NULL;
  }

  // ---------- get header info from the new flit -------------------
  flitSeq = newFb->flitSeq = ntohl(*((uint32_t *) newFb->flitStorage));
  if (flitSeq == 0) {
    newFb->packetSize = ntohl(*((uint32_t *) &newFb->flitStorage[4]));
    if (verbLevel > 3)
      printf("USB_T: Got flit 0. Packet size %d. Flit size %d\n", newFb->packetSize, newFb->flitLen);
  } else {
    if (verbLevel > 3)
      printf("USB_T: Got flit %d. Flit size %d\n", flitSeq, newFb->flitLen);
  }

  return newFb;
}

// ---------------------------------------------------------------------------
// void queueInsert(queueWrapper *p)
//   Insert new packet at end of queue
// ---------------------------------------------------------------------------
void queueInsert(queueWrapper *p)
{
  queueWrapper *q;

  p->next = NULL;
  if (qtail != NULL) {
    qtail->next = p;
    qtail = p;
  } else  // empty queue
    qtail = qhead = p;
}



// ---------------------------------------------------------------------------
// struct sp_port *getNucleoPort()
//     Scan all USB ports for nucleo receiver board
// ---------------------------------------------------------------------------

struct sp_port *getNucleoPort()
{
  struct sp_port **port_list;
  
  enum sp_return result = sp_list_ports(&port_list);

  if (result != SP_OK) {
    printf("USB_T: sp_list_ports() failed!\n");
    exit(EXIT_FAILURE);
  }

  int i;
  for (i = 0; port_list[i] != NULL; i++) {
    struct sp_port *port;

    // Skip any ports which are not USB
    if (sp_get_port_transport(port_list[i]) != SP_TRANSPORT_USB)
      continue;
    int usb_vid, usb_pid;
    sp_get_port_usb_vid_pid(port_list[i], &usb_vid, &usb_pid);

    if ((usb_vid == NUCLEO_USBD_VID) && (usb_pid == NUCLEO_USBD_PID)) {
      sp_copy_port(port_list[i], &port);
      sp_free_port_list(port_list);
      if (verbLevel > 2)
        printf("USB_T: Got Nucleo port\n");
      return port;
    }
  }
  sp_free_port_list(port_list);
  return NULL;
}

// ---------------------------------------------------------------------------
// void startSerialPort()
//   Discovers Nucleo receiver board, connects and configures serial
//    communication and enables Nucleo to start receiving from VLC channel
// ---------------------------------------------------------------------------
void startSerialPort()
{
    uint8_t tries = 0;

    // This tries forever to find the RCV nucleo USB port
    do {
      port = getNucleoPort();
      if ((tries == 0) && (port == NULL)) {
        printf("USB_T: Nucleo STM32 Virtual ComPort VID %d, PID %d not found\n", NUCLEO_USBD_VID, NUCLEO_USBD_PID);
      }
      tries = (tries + 1) % 10;
      sleep(1); 
    } while (port == NULL);

    check(sp_open(port, SP_MODE_READ_WRITE));
    if (verbLevel > 0) 
      printf("USB_T: Setting port to 115200 8N1, no flow control.\n");
    check(sp_set_baudrate(port, 115200));
    check(sp_set_bits(port, 8));
    check(sp_set_parity(port, SP_PARITY_NONE));
    check(sp_set_stopbits(port, 1));
    check(sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE));
    // Clear any older stuff still in the buffer.
    check(sp_flush(port, SP_BUF_BOTH));

    // This send the start symbol (ASCII 'S') and the IC filter value (plain byte)
    printf("USB_T: Enabling VLC receiver.\n");
    if (sp_blocking_write(port, enableString, 2, 0) != 2) {
      printf("USB_T: Could not enable VLC receiver.\n");
      sp_close(port);
      sp_free_port(port);
    }
}



// ---------------------------------------------------------------------------
// void *usb_thread(uint8_t *filter)
//   Separate thread. Starts USB communication with Nucleo receiver. Then loops:
//     - reading from USB
//     -  RS decode of 1st RS block only, to get flit Sequence number
//         and size of packets for flit 0 of a packet
//     - checks if the flit is already received and dumps it
//     - passes to a queue unfiltered flits only
//   until notified that everything has been received
// ---------------------------------------------------------------------------
void *usb_thread(uint8_t *filter)
{
    queueWrapper   *newFb;
    int semVal;

    // Get the filter value from the main thread, and store it
    //   after the 'S' symbol which enables the nucleo
    enableString[1] = *filter;

    startSerialPort();

    rs_usb = correct_reed_solomon_create(correct_rs_primitive_polynomial_8_4_3_2_0,
                                         1, 1, RS_PARITY_SIZE);
    if (rs_usb == NULL) {
      printf("USB_T: ERROR: Failed to initialize RS\n");
      exit(EXIT_FAILURE);
    }

    while (1) { // only way out is abort() or allDone == 1
      // Get flit from serial port. packetSize is assigned to only for when flitSeq is 0
      newFb = receiveFlit(port);
      if (newFb == NULL) {
          continue;  // bad flit/connection error/..., retry
      } 
      // -- A flit has been received from USB

      // Protect shared data from concurrent access
      pthread_mutex_lock(&task_lock);
      if (allDone) {
        // Main thread says: complete file has been received.
        pthread_mutex_unlock(&task_lock);
        break;  // out of while loop, reception is complete
      }

      //  skip all flits which I have already received correctly
      if (   (newFb->flitSeq <= fileDetails.lastFlitNo)   // check bounds before accessing the array
          && (fileDetails.flitArray[newFb->flitSeq].badBlocks == 0)) {
        pthread_mutex_unlock(&task_lock);
        if (verbLevel > 2)
          printf("USB_T: Skipping existing flit %d\n", newFb->flitSeq);
        free(newFb->flitStorage);
        free(newFb);
        continue;
      }     

      queueInsert(newFb); // Add flit to the queue and notify the other thread
      if (sem_post(queueSem) != 0)  {  // Could not post semaphore
        sem_getvalue(queueSem, &semVal);
        printf("USB_T: COULD NOT POST SEMAPHORE!. Value = %d\n", semVal);
      }
      pthread_mutex_unlock(&task_lock);

    } // end while
	
    // We only get here when the file has been received

    // Send stop to nucleo through USB
    printf("USB_T: All received. Stoping VLC receiver.\n");
    if (sp_blocking_write(port, stopString, strlen(stopString), 0) != strlen(stopString)) {
      printf("USB_T: Could not stop VLC receiver.\n");
    }
    // Clear any data in buffers. Possibly superfluous as sp_close might do it too.
    check(sp_flush(port, SP_BUF_BOTH));
    sp_close(port);
    sp_free_port(port);
    pthread_exit(0);
}


