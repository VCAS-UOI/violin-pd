/*
 The VLC receiver. It takes 3 optional command-line arguments (do not add spaces. e.g. verbose 3 is -v3)
 A single file is received and stored in the current directory. The filename is specified by `-n`. 
  "received" is used if no filename is specified.
 When the file is complete, the nucleo board is instructed to stop receiving and the app exits.

  The Reed-Solomon decoder is from libcorrect: https://github.com/quiet/libcorrect Copyright (c) 2016, Brian Armstrong

  The rest of the code is (c) Aristides Efthymiou, VCAS lab, University of Ioannina, provided under the MIT license. See the top-level repository LICENSE file.
 */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <argp.h>
#include <unistd.h>
#include <arpa/inet.h>

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <semaphore.h>
#include "vlcRecv.h"

#include "correct.h"

// ----------   argp stuff
const char *argp_program_version = "vlcReceiver v.latest";
const char *argp_program_bug_address = "<efthym@cse.uoi.gr>";

/* Program documentation. */
static char doc[] =
  "vlcRecv -- Receives a file from a Nucleo board connected to a USB port via a  VLC link.";

/* A description of the arguments we accept. */
static char args_doc[] = "TODO";

/* The options we understand. */
static struct argp_option options[] = {
  {"verbose",  'v',  "VERBOSE", OPTION_ARG_OPTIONAL, "Produce verbose output" },
  {"filter",   'f',  "FILTER",  OPTION_ARG_OPTIONAL, "Glitch filter to pass on to Nucleo" },
  {"filename", 'n',  "LIMIT",   OPTION_ARG_OPTIONAL, "Filename for the received packet." },
  { 0 }
};

/* Used by main to communicate with parse_opt. */
struct arguments
{
  char *args[10];
  int  verbose;
  int  filter;
  char *fname;
};

/* Parse a single option. */
static error_t parse_opt (int key, char *arg, struct argp_state *state)
{
  /* Get the input argument from argp_parse, which we
     know is a pointer to our arguments structure. */
  struct arguments *arguments = state->input;

  switch (key)
    {
    case 'v':
      arguments->verbose = arg ? atoi(arg) : 1;
      break;
    case 'f':
      arguments->filter = arg ? atoi(arg) : 0;
      break;
    case 'n':
      arguments->fname = arg;
      break;
    case ARGP_KEY_ARG:
      if (state->arg_num >= 4)
        /* Too many arguments. */
        argp_usage (state);

      arguments->args[state->arg_num] = arg;

      break;

    case ARGP_KEY_END:
      if (state->arg_num < 0)
        /* Not enough arguments. */
        argp_usage (state);
      break;

    default:
      return ARGP_ERR_UNKNOWN;
    }
  return 0;
}

/* Our argp parser. */
static struct argp argp = { options, parse_opt, args_doc, doc };


const uint16_t vlcMaxChunk = 1024; 
  
extern void *usb_thread();

// ------------- Globals shared by both threads -------
pthread_mutex_t task_lock;
sem_t        *queueSem;
uint8_t      allDone; // 
queueWrapper *qtail, *qhead;

fileInfo     fileDetails; 

uint8_t  verbLevel = 1;  // verbosity level for messages


// ------------- Globals for this thread -------
correct_reed_solomon *rs;  // Reed-Solomon struct pointer


// ---------------------------------------------------------------------------
// queueWrapper *queueWaitAndRemove()
// Blocks until there is a queueWrapper object in the queue
//  removes and returns it
// ---------------------------------------------------------------------------
queueWrapper *queueWaitAndRemove()
{
  queueWrapper *q;

  // Block-wait until there is something in the queue
  if (sem_wait(queueSem) != 0) {
    printf("ERROR: sem_wait failed %s\n",strerror(errno));
    exit(EXIT_FAILURE);
  }
      
  pthread_mutex_lock(&task_lock);  // protect concurrent access to head/tail
  if (qhead == NULL) {
    printf("ERROR: The Queue head is NULL\n");
    exit(EXIT_FAILURE);
  }
  
  q = qhead;
  qhead = qhead->next;

  if (qhead == NULL)  // empty
    qtail = NULL;
  pthread_mutex_unlock(&task_lock);

  return q;
}



// ---------------------------------------------------------------------------
// uint8_t headerSzAndInitPacket0(queueWrapper *newFb)
//  Returns the size in bytes of the header of a flit
//  When flit 0 of a packet gets processed or one with a flit sequence higher than 
//   previously received ones:
//    space is allocated in fileDetails.flitArray
//    and its members get initialized, as well as
//    and fileDetails members badFlits, lastFlitNo get updated
// ---------------------------------------------------------------------------
uint8_t headerSzAndInitPacket0(queueWrapper *newFb)
{
  uint16_t flitLen = newFb->flitLen;
  int32_t  flitSeq = newFb->flitSeq;
  uint32_t packetSize = newFb->packetSize;
  uint8_t  *flitStorage = newFb->flitStorage;

  int32_t  lastFlitNo;  // NOTE: MUST BE SIGNED!
  uint16_t num_rs_blocks;
  int      i;

  if (flitSeq == 0) { 
    fileDetails.fileSize = packetSize;
  }
  // The packet size is initially unknown (or if we miss the 1st flit, we may still not know it)
  //  note that lastFlitNo starts as -1, so flit 0 will trigger true here
  if (fileDetails.lastFlitNo < flitSeq) {
    if (flitSeq == 0) { 
      // This can happen only if flit 0 is the first flit to arrive,
      //   which should be the norm for fault-free communication,
      //   as lastFlitNo will be set by the first flit to arrive
      // calculate lastFlitNo from packetSize
      lastFlitNo = packetSize / vlcMaxChunk - 1;
      if (packetSize % vlcMaxChunk != 0)
        lastFlitNo += 1;
      fileDetails.badFlits = lastFlitNo + 1;
      //  Note:  fileDetails.lastFlitNo  is set later
    } else {
      // We got a higher flit sequence, assume that's the last one
      lastFlitNo = flitSeq;
    }
    flitBuff *temp = realloc(fileDetails.flitArray, (lastFlitNo+1)*sizeof(flitBuff));
    if (temp == NULL) {
      printf("ERROR: Could not expand flit buffers of meta packet\n");
      exit(EXIT_FAILURE);
    }
    pthread_mutex_lock(&task_lock);  // protect concurrent access
    fileDetails.flitArray = temp;
    // Initialize our books for the flits we just learned about
    for (int i = fileDetails.lastFlitNo+1; i <= lastFlitNo; i++) {
      fileDetails.flitArray[i].num_rs_blocks = 0;
      fileDetails.flitArray[i].badBlocks = -1;
      fileDetails.flitArray[i].rsBlockStatus = NULL;
    }
    if (flitSeq != 0) {
      if (fileDetails.badFlits > 0)
        fileDetails.badFlits += (lastFlitNo - fileDetails.lastFlitNo);
      else // fix for the initial -1 value of badFlits...
        fileDetails.badFlits += (lastFlitNo - fileDetails.lastFlitNo) + 1;
    }
    fileDetails.lastFlitNo = lastFlitNo;
    pthread_mutex_unlock(&task_lock);  // protect concurrent access
  }

  pthread_mutex_lock(&task_lock);  // protect concurrent access
  if (fileDetails.flitArray[flitSeq].badBlocks == -1) {
    num_rs_blocks = flitLen / RS_BLOCK_SIZE;
    if (flitLen % RS_BLOCK_SIZE != 0) {
      num_rs_blocks += 1; 
      fileDetails.flitArray[flitSeq].lastBlockLen = flitLen % RS_BLOCK_SIZE - RS_PARITY_SIZE;
    } else
      fileDetails.flitArray[flitSeq].lastBlockLen = RS_DATA_BLOCK_SIZE;
    fileDetails.flitArray[flitSeq].badBlocks = num_rs_blocks;
    fileDetails.flitArray[flitSeq].num_rs_blocks = num_rs_blocks;
    if (fileDetails.flitArray[flitSeq].rsBlockStatus == NULL)
      fileDetails.flitArray[flitSeq].rsBlockStatus = malloc(num_rs_blocks*sizeof(uint8_t));
    for (i = 0; i <  num_rs_blocks; i++)
      fileDetails.flitArray[flitSeq].rsBlockStatus[i] = 0;
  }
  pthread_mutex_unlock(&task_lock);  // protect concurrent access

  // Calc the header bytes at the begging of the flit
  if (flitSeq == 0)
    return 8;
  else
    return 4;
}



// ---------------------------------------------------------------------------
// void insertFlit(queueWrapper *newFb)
// Decode RS blocks and store data in the corresponding file.
//  Keep track of correctly received RS blocks per flit, and flits per file/packet
//  When all have been received, set allDone to 1
// ---------------------------------------------------------------------------
void insertFlit(queueWrapper *newFb)
{
  int32_t  flitSeq = newFb->flitSeq;
  uint8_t  *flitStorage = newFb->flitStorage;

  uint8_t  updated;
  uint32_t offset;
  uint16_t blockSize;
  uint8_t  *blockDataPtr;
  uint8_t  headerBytes;
  int      i;


  if (verbLevel > 2) {
    printf("Inserting flit %d\n", flitSeq);
  }

  // Handles initialization of a new packet: initializes structs, calculates number - size of RS blocks, ...
  headerBytes = headerSzAndInitPacket0(newFb);

  if (verbLevel > 2) {
    printf(" BadFlits: %d\n", fileDetails.badFlits);
    printf("  badBlocks %d number of rs blocks %d lastFlitNo %d\n", fileDetails.flitArray[flitSeq].badBlocks,
           fileDetails.flitArray[flitSeq].num_rs_blocks, fileDetails.lastFlitNo);
    for (int j = 0; j <= fileDetails.lastFlitNo; j++) {
        printf("Flit %d \n\t num RS blocks %d, lastBlockLen %d, badBlocks %d\n", j,
               fileDetails.flitArray[j].num_rs_blocks, fileDetails.flitArray[j].lastBlockLen,
               fileDetails.flitArray[j].badBlocks);
        for (int k = 0; k < fileDetails.flitArray[j].num_rs_blocks; k++) {
            printf("\t\t %d ", fileDetails.flitArray[j].rsBlockStatus[k]);
        }
        printf("\n");
    }
  }

  updated = 0;
  // Foreach RS block of the flit
  for (i = 0; 
             (fileDetails.flitArray[flitSeq].badBlocks > 0)
              &&
             (i < fileDetails.flitArray[flitSeq].num_rs_blocks);
       i++) {
    if (fileDetails.flitArray[flitSeq].rsBlockStatus[i] == 1) {
      if (verbLevel > 2) {
        printf("Duplicate RS block in flit %d, block %d\n", flitSeq, i);
      }
      continue;
    }
    blockDataPtr = flitStorage + i * RS_BLOCK_SIZE;
    if (i == fileDetails.flitArray[flitSeq].num_rs_blocks-1) { // last block
      blockSize = fileDetails.flitArray[flitSeq].lastBlockLen;
    } else
      blockSize = RS_DATA_BLOCK_SIZE;
    // check FEC - decode the block, but not for block 0, which has been decoded in the usb task
    if (i != 0) {
      ssize_t ret = correct_reed_solomon_decode(rs, blockDataPtr, blockSize+RS_PARITY_SIZE, blockDataPtr);
      if (ret < 0) {
        if (verbLevel > 2) 
          printf("RS failed for %d\n", i);
        continue;  // Failed, try next RS block
      } else {
        if (verbLevel > 2) {
          if (ret > 0)
            printf("RS recovered for %d\n", i);
          else
            printf("RS OK for %d\n", i);
        }
      }
    }
    // Copy from newFb to file 
    //  All flits are completely filled to their max capacity, except for the last one.
    //  Similarly for RS blocks, except for the last one.
    //  vlcMaxChunk must be a multiple of the RS data block size (128)
    //  The last RS block of each flit will be tiny (8 for flit 0, 4 for others for complete flits).
    //   This is not efficient. I should revisit...
    if (i == 0) {
      // adjust pointer and size to remove the header present at 1st RS block only
      blockDataPtr += headerBytes;
      blockSize -= headerBytes;
    }
    // Calculate the file offset for the current RS block.
    offset = flitSeq * vlcMaxChunk + i * RS_DATA_BLOCK_SIZE - ((i == 0)? 0: headerBytes);
    // fseek from start of file to the position of the specific block of this flit
    if (fseek(fileDetails.fp, offset, SEEK_SET) == 0) {
      fwrite(blockDataPtr, blockSize, 1, fileDetails.fp);
    } else {
      printf("ERROR: fseek not allowed!\n");
      exit(EXIT_FAILURE);
    }
    pthread_mutex_lock(&task_lock);  // protect concurrent access
    fileDetails.flitArray[flitSeq].rsBlockStatus[i] = 1; // mark as done
    if (fileDetails.flitArray[flitSeq].badBlocks > 0) {
      fileDetails.flitArray[flitSeq].badBlocks -= 1; // adjust 
      updated = 1;
    }
    pthread_mutex_unlock(&task_lock);  // protect concurrent access
  }  // end for 

  // --------------------
  // Do book-keeping: count down badflits, if all RS blocks are OK
  //  countdown packets/files if all flits are OK

  // updated is used to guard for usb task sending us the same flit twice.
  //   We must have just updated something in a flit to decrement badFlits.
  //   So an identical flit will get here with 0 badBlocks, but updated will be 0 too.
  if ( updated && (fileDetails.flitArray[flitSeq].badBlocks == 0)) {
    pthread_mutex_lock(&task_lock);  // protect concurrent access
    if (fileDetails.badFlits > 0)
      fileDetails.badFlits -= 1;
    if (fileDetails.badFlits == 0) {
      allDone = 1;
    }
    pthread_mutex_unlock(&task_lock);  // protect concurrent access
  }

  // Release memory of flitStorage and wrapper
  free(newFb->flitStorage);
  free(newFb);
}


int main(int argc, char **argv)
{
  uint8_t  filter;
  pthread_t usbTask;
  struct arguments arguments;

  allDone = 0; // We still got files to receive...
  pthread_mutex_init(&task_lock, NULL);

  rs = correct_reed_solomon_create(correct_rs_primitive_polynomial_8_4_3_2_0,
                                   1, 1, RS_PARITY_SIZE);
  if (rs == NULL) {
    printf("ERROR: Failed to initialize RS\n");
    exit(EXIT_FAILURE);
  }

  qtail = qhead = NULL;  // queue is empty
  queueSem = sem_open("queueSem", O_CREAT, 0600, 0);
  if (queueSem == SEM_FAILED) {
    printf("ERROR: sem_open failed %s\n",strerror(errno));
    exit(EXIT_FAILURE);
  }

  // Handle the command-line arguments
  arguments.verbose = 1;
  arguments.filter = 0;
  arguments.fname = "received";
  argp_parse (&argp, argc, argv, 0, 0, &arguments);

  // Allocate the first/meta packet entry
  fileDetails.fileName = arguments.fname;
  fileDetails.lastFlitNo = -1;
  fileDetails.flitArray = NULL;
  fileDetails.badFlits = -1;  // MUST BE SIGNED AND INITIALIZED TO -1. This may get incremented......

  fileDetails.fp = fopen(fileDetails.fileName, "wb");
  if (fileDetails.fp == NULL) {
    printf("ERROR: could not open file %s\n", fileDetails.fileName);
    exit(EXIT_FAILURE);
  }

  verbLevel = arguments.verbose;

  // Create the USB-reading thread
  pthread_create(&usbTask, NULL, usb_thread, &arguments.filter);
  queueWrapper   *newFb;

  while (!allDone) {
    newFb = queueWaitAndRemove();
    // Process the flit
    insertFlit(newFb);
  }
  // Truncate the file to its original size. This is usefull for odd-sized files which get 1 extra byte for VLC transmission...
  fflush(fileDetails.fp);
  if (ftruncate(fileno(fileDetails.fp), fileDetails.fileSize) == -1) {
        perror("Error truncating file");
  }
  fclose(fileDetails.fp);
  pthread_join(usbTask, NULL);
  exit(EXIT_SUCCESS);
}

