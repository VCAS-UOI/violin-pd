/* ----------------------------------------------------------------------------
 Demo transmission code for the VIOLIN VLC project with a photodiode receiver.
 Continuously transmits a file (argv[1]).  You must kill this app to stop it!

 The file is first split into "flit"s, and encoded for error correction using Reed-Solomon. To avoid doing this all the time, it is done once and stored encoded in a temporary file. A flit contains up to 1024 bytes of data (parity and some headers are added).
 The nucleo xmit board sends a request message though its Ethernet connection to the host to port 3443. This application responds with the "next" flit of the file and this continuous for ever. When network errors are encountered the transmission starts aftesh.
 An automatically created log file records when disconnections occur, etc.
  
 Port 3443 must be enabled though your firewall.
 The IP of the host running this code, must be entered into Inc/main.h of the xmit/firmware

  Reed-Solomon implementation uses the libcorrect library: https://github.com/quiet/libcorrect Copyright (c) 2016, Brian Armstrong

 Copyright (c) 2024 Aristides Efthymiou (efthym@uoi.gr),
  VLSI Systems and Computer Architecture Laboratory (VCAS),
  Dep. of Computer Science and Engineering, University of Ioannina, Greece
 MIT license. See the top-level repository LICENSE file.
 ------------------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdlib.h> 
#include <string.h> 
#include <unistd.h>
#include <stdint.h>
#include <errno.h>

#include <sys/param.h>
#include <sys/stat.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include <sys/time.h>
#include <time.h>

// Reed Solomon library
#include "correct.h"

#define RS_DATA_BLOCK_SIZE 128
#define RS_PARITY_SIZE 32
// It it the sum of the above two
#define RS_BLOCK_SIZE 160


// Buffer size
#define MAX_BUF (5*1024)

// The TCP port where the nucleo boards sends requests to get the next data to xmit.
#define PORT 3443 

// Uncomment to add (recoverable) RS errors on purpose in the transmitted file.
//#define ADD_ERRORS 1

// Number of transmissions of the first flit of a packet, before transmitting the next flit
const uint8_t firstFlitRepetition = 1;
// Number of transmissions of a flit, before transmitting the next flit
const uint8_t commonFlitRepetition = 1;
// Tha max data payload (headers are extra) of a flit.
const uint16_t vlcMaxChunk = 1024;

correct_reed_solomon  *rs;  // Reed-Solomon struct pointer

FILE *logfp; // File pointer of logfile


typedef struct {
  char     *fname;       // filename
  FILE     *encFile;     // file pointer to the RS encoded equivalent file
  uint32_t size;
  uint32_t firstFlitEncSize;  // includes the flit rep times byte
  uint32_t lastFlitEncSize;
  uint16_t numFlits;
} fileItem;

// --------------------------------------------------------
// The transmission buffer
// bufXmit[0] - 4 copies of flitRepetition byte
// bufXmit[1] - flit sequence number
// bufXmit[2] - packet size, only for first flit of a packet
//              otherwise the payload starts here.
typedef __attribute__((aligned(4))) uint32_t ui32_aligned;
ui32_aligned bufXmit[MAX_BUF]; 



void logMessage(char *s)
{
  struct tm *timenow;
  time_t now;
  char  timeString[50];

  now = time(NULL);
  timenow = gmtime(&now);
  strftime(timeString, sizeof(timeString), "%Y-%m-%d_%H:%M:%S", timenow);
  fprintf(logfp, "%s  %s\n", timeString, s);
  fflush(logfp);
}


// ----------------------------------------------------------------------------

// Copy the in byte into all 4 bytes of the uint32_t result
uint32_t byte2uint32(uint8_t in)
{
    uint32_t temp = (in << 8) | in;
    return ((temp << 16) | temp);
}



// ----------------------------------------------------------------------------
// uint32_t getFlit(uint32_t *readChunk, uint32_t currFlit, uint32_t packetSize, uint8_t *dst, uint8_t extraBytes, FILE *fp)
//  Reads data from file fp, encodes it using Reed-Solomon and stores  the encoded data (and parity bits) into dst. 
//  This is done per RS block with the parity bytes appended after each block.
//  The header is already inserted before calling this function
//    and it is RS encoded with the first part of the payload.
// Returns the total number of parity bytes added. readChunk returns the number of data bytes read.
// ----------------------------------------------------------------------------
uint32_t getFlit(uint32_t *readChunk, uint32_t currFlit, uint32_t packetSize, uint8_t *dst, uint8_t extraBytes, FILE *fp)
{
  uint8_t flitComplete = 0;
  uint16_t blockNumber;
  uint32_t maxToRead, readBlockBytes;
  uint8_t  *startRSBlock;

  *readChunk = 0;
  blockNumber = 0;
  while (!flitComplete) {
    if (blockNumber == 0) {
      maxToRead = MIN(RS_DATA_BLOCK_SIZE - extraBytes + 1,  // The +1 is because extraBytes include the flit repetition byte
                  vlcMaxChunk - *readChunk);  // the remaining bytes
    } else
      maxToRead = MIN(RS_DATA_BLOCK_SIZE,  vlcMaxChunk - *readChunk);

    readBlockBytes = fread(dst, 1, maxToRead, fp);
    if (readBlockBytes < maxToRead)  // When end of file is reached, the flit is complete
      flitComplete = 1;
    (*readChunk) += readBlockBytes;
    if ((*readChunk == vlcMaxChunk) || (*readChunk == packetSize))
      flitComplete = 1;
  
    startRSBlock = ((uint8_t *) &bufXmit[1]) + blockNumber*RS_BLOCK_SIZE;

    // This is usefull only for the first block, so we can read just enough bytes to fill
    //   the first Reed-Solomon block, but we need to advance the desitation by as much as read plus the header
    if (blockNumber == 0)
      readBlockBytes += extraBytes - 1;
    //printf(" Encoding %d bytes\n", readBlockBytes);
    correct_reed_solomon_encode(rs, 
                   startRSBlock,
                   readBlockBytes,
                   startRSBlock);
    dst = startRSBlock + readBlockBytes + RS_PARITY_SIZE;
    blockNumber += 1;
  }
  //printf("Flit %d blockNumber %d readChunk %d\n", currFlit, blockNumber, *readChunk);
  return RS_PARITY_SIZE * blockNumber;
}

// ----------------------------------------------------------------------------
// void encodePacket(fileItem *flp)
// RS encode a single packet/file and store it in temporary file
//  keeping info (number of flits, their sizes, the temp file pointer) in the fileItem struct
// ----------------------------------------------------------------------------
void encodePacket(fileItem *flp)
{
  FILE     *fp;
  uint32_t packetSize, accPacketSize;
  uint32_t currFlit;
  uint32_t readChunk;
  uint8_t  *flitBuffer, *payloadAddr;
  uint8_t extraBytes;
  uint32_t parityBytes;
  // --------------------------------------------------------------
  // To keep everything aligned, I got the flit repetion byte just before the 32b flit seqeuence number
  //  so the flit buffer in not bufXmit, but bufXmit + 3 bytes!
  flitBuffer = ((uint8_t *) &bufXmit[1]) - 1;


  // Create and open temporary file for encoded version. This includes everything to be sent (repetition, headers, ...)
  flp->encFile = tmpfile();
  fp = fopen(flp->fname , "rb" );
  if (fp == NULL) {
    printf("ERROR:Could not open file %s\n", flp->fname); 
    exit(1); 
  }
  // Get packet (file) size
  packetSize = flp->size;
  accPacketSize = 0;  // accumulated size of the packet read from file up to now
  currFlit = 0;

  // ---------------- For each flit of the packet --------------------------
  while (accPacketSize < packetSize) {
    // For the first flit of a packet, add the packet size too.
    if (currFlit == 0) {
      // ---- Add packet size (only at 1st flit of a packet)
      bufXmit[2] = htonl(packetSize);  // convert to network byte order
      payloadAddr = (uint8_t *) &bufXmit[3];
      extraBytes = 9;  // 2*4 (flit sequence number, pack size) + 1 (flit repetition)
    } else {
      payloadAddr= (uint8_t *) &bufXmit[2];
      extraBytes = 5; // 1*4 (flit sequence number) + 1 (flit repetition)
    }

    // ---- Add flit sequence number
    bufXmit[1] = htonl(currFlit);
    // Read a flit from file and do RS encoding
    parityBytes = getFlit(&readChunk, currFlit, packetSize, payloadAddr, extraBytes, fp);
    accPacketSize += readChunk;

    // Check for errors
    if (readChunk <= 0 || accPacketSize > packetSize) {
      printf("ERROR: In file %s (flit %d), read 0 bytes (or more than file size %d)\n",
             flp->fname, currFlit, packetSize); 
      exit(1); 
    }
    if (readChunk % 2 != 0) {
      // Nucleo expects an even number of bytes (excluding the flit repetition byte)
      // Add a byte at the end, but keep the packetSize as is, so the receiver can ignore this byte
      //   This should happen only at the last flit of a packet/file
      extraBytes += 1;
    }
    if (currFlit == 0)
      bufXmit[0] = byte2uint32(firstFlitRepetition);
    else
      bufXmit[0] = byte2uint32(commonFlitRepetition);

    //printf("Storing flit %d. total size %d\n", currFlit, readChunk + parityBytes + extraBytes);
    if (fwrite(flitBuffer, 1, readChunk + parityBytes + extraBytes, flp->encFile)
         !=  readChunk + parityBytes + extraBytes) {
      printf("ERROR: Could not write to temp file\n");
      exit(1);
    }
    if (currFlit == 0)
      flp->firstFlitEncSize = readChunk + parityBytes + extraBytes;
    currFlit++;
  } // end while (flits in packet)
  fflush(flp->encFile);
  //printf("Temp file length %d\n", ftell(flp->encFile));
  flp->numFlits = currFlit;
  flp->lastFlitEncSize = readChunk + parityBytes + extraBytes;
  fclose(fp);  // close the file
  // DO NOT CLOSE THE TEMP FILE! IT WILL BE REMOVED!
}


void xmitLoop(fileItem *flp, int peerSock)
{
  uint32_t currFlit;
  uint32_t readChunk;
  int      size_rcv;
  uint32_t max2Read;
  uint8_t  *flitBuffer;
  FILE     *fp;

  char  logMsg[100];
  // Buffer for receiving responses from Nucleo
  char buff_rcv[20]; 

  // --------------------------------------------------------------
  // To keep everything aligned, I got the flit repetion byte just before the 32b flit seqeuence number
  //  so the flit buffer is not bufXmit, but bufXmit + 3 bytes!
  flitBuffer = ((uint8_t *) &bufXmit[1]) - 1;

  // Forever loop: we transmit the file forever
  while (1) {
    fp = flp->encFile;
    // rewind the file
    if (fseek(flp->encFile, 0, SEEK_SET) != 0) {
      printf("ERROR: fseek not allowed!\n");
      exit(1);
    }

    currFlit = 0;

    // ---------------- For each flit of the packet --------------------------
    while (currFlit < flp->numFlits) {
      // Wait for the request from Nucleo
      memset(buff_rcv, 0, 50);  // clear previous response
      uint8_t retry = 0;
      // Wait for a request from the Nucleo xmitter
      do {
        size_rcv = recv(peerSock, buff_rcv, sizeof(buff_rcv), 0); 
        if (size_rcv == -1 && errno == EAGAIN) { // resource temporarily unavailable 
          usleep(500);
          retry++;
        } else
          break;
      } while (retry < 1);  // I always get a timeout after EAGAIN, so 1 attempt should be enough.

      if (size_rcv != 7) { 
        // We didn't get the expected request message
        if (size_rcv < 0) {
          perror("Receive of REQ failed ");
          sprintf(logMsg, "Receive of REQ failed: %s", strerror(errno));
          logMessage(logMsg);
        } else {
          printf("Received %d %s Stopping.\n", size_rcv, buff_rcv);
          sprintf(logMsg, "Received %d %s re-starting.", size_rcv, buff_rcv);
          logMessage(logMsg);
        }
        // Close socket
        close(peerSock);
        return;
      }

      if (currFlit == 0) {
        max2Read = flp->firstFlitEncSize;
      } else if (currFlit == flp->numFlits-1) {
        max2Read = flp->lastFlitEncSize;
      } else
        max2Read = flp->firstFlitEncSize - 4;
      // Max bytes to read are 4 less for the 2nd flit onwards, as the header does not contain the packet size
      //printf("Reading flit %d \n", currFlit);
      readChunk = fread(flitBuffer, 1, max2Read, fp);
      // For the first flit of a packet, add the packet size too.
      if (readChunk != max2Read) {
        printf("ERROR: Read fewer than expected %d from %d\n", readChunk, max2Read);
        exit(1);
      }

#ifdef ADD_ERRORS
      // Invert the last byte of the data part of each RS block
      int last = (readChunk - 1) // -1 for the extra "repetition" byte
           % RS_BLOCK_SIZE;
      int numRSblocks = (readChunk-1)/RS_BLOCK_SIZE;
      for (int i = 0; i < numRSblocks; i += 1)
        *(flitBuffer+i*RS_BLOCK_SIZE+RS_DATA_BLOCK_SIZE-1) ^= 0xff; // -1 as 1st byte is at 0
       //invert the byte
      if (last != 0)
        *(flitBuffer-RS_PARITY_SIZE-1) ^= 0xff;
#endif

      //printf("Sending flit %d total size %d\n", currFlit, readChunk);
      //  --------------- Xmit flit and wait for ack -------------------------
      if (send(peerSock, flitBuffer, readChunk, 0) < 0) {
        perror("Send failed ");
        sprintf(logMsg, "Send failed with %s", strerror(errno));
        logMessage(logMsg);
        return;
      }

      currFlit++;
    } // end while (flits in packet)
  }  // end forever
}




int main (int argc, char **argv)
{
  int sockfd, peerSock; 
  int totalPackets;
  struct sockaddr_in nucleoAddr; 
  struct sockaddr_in peer_addr; 
  socklen_t peer_addr_size;

  // Timeout of 5sec for the Nucleo to communicate
  struct timeval tv;
  tv.tv_sec = 5;
  tv.tv_usec = 0;

  // Create log file
  struct tm *timenow;
  time_t now = time(NULL);
  timenow = gmtime(&now);
  char  logFile[100];
  char  logMsg[100];

  fileItem *flp;

  strftime(logFile, sizeof(logFile), "logVlcServer_%Y-%m-%d_%H%M%S", timenow);
  logfp = fopen(logFile, "w");
  if (logfp == NULL) {
    printf("Cannot open log file %s\n",logFile);
    exit(1);
  }

  printf("Loging in file: %s\n", logFile);

  rs = correct_reed_solomon_create(correct_rs_primitive_polynomial_8_4_3_2_0,
                                         1, 1, RS_PARITY_SIZE);
  if (rs == NULL) {
    printf("ERROR: Failed to initialize RS\n");
    exit(1);
  }

  flp = malloc(sizeof(fileItem));
  if (flp == NULL) {
    printf("ERROR: failed to allocate memory\n");
    exit(1);
  }
  flp->fname = argv[1];

  struct stat statBuf; // file attributes
  stat(argv[1], &statBuf); 
  flp->size = statBuf.st_size;

  // Encode the file with RS into a temp file
  encodePacket(flp);

  // --------------------------------------------------------------
  // Open socket to nucleo
  // socket create and varification 
  sockfd = socket(AF_INET, SOCK_STREAM, 0); 
  if (sockfd == -1) { 
    perror("socket creation failed"); 
    exit(0); 
  } else
    printf("Socket successfully created..\n"); 

  memset(&nucleoAddr, 0, sizeof(nucleoAddr)); 
  // assign IP, PORT 
  nucleoAddr.sin_family = AF_INET; 
  nucleoAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  nucleoAddr.sin_port = htons(PORT); 
 
  if (bind(sockfd, (struct sockaddr *) &nucleoAddr, sizeof(nucleoAddr)) < 0) {
    perror("bind to port failed"); 
    exit(0); 
  } 

  if (listen(sockfd, 2) < 0) {  // a queue of 2 connections should be more than enough for now
    perror("listen failed"); 
    exit(0); 
  } 

  peer_addr_size = sizeof(peer_addr);
  while (1) {
    peerSock = accept(sockfd, (struct sockaddr *) &peer_addr, &peer_addr_size);
    if (peerSock < 0) {
      perror("accept failed ");
      sprintf(logMsg, "accept failed with %s", strerror(errno));
      logMessage(logMsg);
      continue;
    }
    // Set timeout for reception
    setsockopt(peerSock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
    // Timeout for transmittion
    setsockopt(peerSock, SOL_SOCKET, SO_SNDTIMEO, (const char*)&tv, sizeof tv);

    // This function only returns when there is an error
    xmitLoop(flp, peerSock);
    printf("Re-doing accept\n");
    sprintf(logMsg, "Re-doing accept");
    logMessage(logMsg);
  }
  // Never happens
  close(sockfd);
}


