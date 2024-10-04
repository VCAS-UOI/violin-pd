#include <sys/time.h>

typedef struct queueWrapper {
  uint16_t  flitLen;    // The length of the flit. This includes header, parity etc.
  int32_t   flitSeq;    // normally unsigned, but need to be signed for a comparison to a signed number
  uint32_t  packetSize;  // the net data/payload/file size. Does NOT include headers, parity etc.
  uint8_t   isDuplicate; // Marked by the nucleo xmitter. Not used.
  uint8_t   *flitStorage; // The payload
  struct queueWrapper *next;
} queueWrapper;


typedef struct flitBuff {
  uint8_t         num_rs_blocks;  // Number of Reed-Solomon blocks in the flit
  uint16_t        lastBlockLen;   // The length of the last block
  int16_t         badBlocks;   // MUST BE SIGNED, SO I CAN HAVE -1 AS AN INVALID FLAG
  uint8_t         *rsBlockStatus;  // Array of the status of each RS block.
} flitBuff;

// A description of the file (to be) received
typedef struct {
  char     *fileName;
  FILE     *fp;
  uint32_t fileSize;     // Actual file size. does not include parity or header info
  int32_t  badFlits;     // number of remaining incomplete flits
  int32_t  lastFlitNo;   // MUST BE SIGNED, SO I CAN HAVE -1 AS AN INVALID FLAG
  flitBuff *flitArray;  // Array of flitBuffers
} fileInfo;

#define RS_DATA_BLOCK_SIZE 128
#define RS_PARITY_SIZE 32
// It it the sum of the above two
#define RS_BLOCK_SIZE 160
