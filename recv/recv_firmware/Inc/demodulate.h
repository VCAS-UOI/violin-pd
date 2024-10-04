
#include "main.h"


// This is the max packet size (without the header)  IT MUST HAVE ROOM FOR COBS encoding...
#define MAX_PACKET_SIZE ((uint32_t)  2048)

#define SYNC_BITS 24U

// this is the first bit of the SYNC part of the packet
#define START_BIT 1U 

#define HALF_BUFF_SIZE ((uint32_t)  512)
#define FLIT_Q_SIZE 8

// The timer operates at 240MHz - each counter tick is 4.16666ns.
// This was a define, hence the all caps name.
extern uint32_t IDLE_SAMPLE_THRES;


typedef enum
{
  MAN_IDLE              = 0x00U,   // No transmission. Line should be held at ~START_BIT
  MAN_SYNC              = 0x01U,   // sync - preample phase of packet for clock sync
  MAN_PACKET_SIZE       = 0x02U,   // after the preample, receiving the size of the payload
  MAN_PAYLOAD           = 0x03U,   // packet payload bits
  MAN_WAIT_FOR_STOP_BIT = 0x04U,   // if last bit of payload == START_BIT, wait for STOP_BIT (=~START_BIT)
  MAN_WAIT_FOR_TDP      = 0x05U,   // after enough FLP pulses, wait for start of TLP
  MAN_TDP               = 0x06U,   // in the TLP part of the preamble
  MAN_ERROR_RECOVERY    = 0xFFU    // Error in transmission, should dump current packet and wait for the next.
} ManStateTypeDef;

typedef enum
{
  MAN_ERR_OK                 = 0x00U, //  no error
  MAN_ERR_SYNC               = 0x01U, // too long pulse durring sync-preample
  MAN_ERR_SYNC_TOO_SHORT     = 0x02U, // too short pulse durring sync-preample
  MAN_ERR_TOO_LONG           = 0x11U, //  too long pulse durring data transmission
  MAN_ERR_TOO_SHORT          = 0x12U, // too short pulse during data transmission           
  MAN_ERR_EXPECTED_2ND_SHORT = 0x21U, // Was expecting 2nd short pulse but got a long one
  UNKNOWN_SYMBOL             = 0x400, // RLL decoder has found an undefined symbol
  MAN_ERR_DATA_OVERRUN       = 0xFFU, // manDecode called again before previous call has returned
  MAN_ERR_NOT_ENOUGH_SAMPLES = 0x40U // less than MIN_SAMPLES_PER_PERIOD taken
} ManErrorTypeDef;

typedef enum 
{
  EDGE_MAIN,
  EDGE_SECONDARY
} DemodEdgeTypeDef;

// Manchester decode handler struct
//   most of these are here because we may not get a full packet per manDecode() call,
//   so the state must persist.
typedef struct {
  // ---- Current state and error code for debugging
  ManStateTypeDef state;
  ManErrorTypeDef error;

  DemodEdgeTypeDef  edge;   // next edge is the main edge or the secondary

  uint32_t symbolPeriod;
  uint32_t rll_buff; // Space for receiving a RLL encoded symbol
  uint32_t rll_bits; // number of chips in rll_buf
  uint8_t  decodedByte; // the decoded byte before it is COBS encoded.
  uint8_t  code; // the COBS code variable

  uint8_t  firstEdge;

  uint32_t lo_pulse_p2d_thres,   // see initDemodulation() for explanation of these
           hi_pulse_p2d_thres,
           lo_pulse_h2l_thres,
           hi_pulse_h2l_thres,
           lo_pulse_l2h_thres,
           hi_pulse_l2h_thres,
           lo_pulse_hb_thres,
           hi_pulse_hb_thres,
           lo_pulse_lb_thres,
           hi_pulse_lb_thres,
           preamble_thres,
           tdp_long_thres;

  uint32_t main_pulse_thres,
           sec_pulse_thres;

  uint32_t total_pulse_duration,
           avg_pulse_width;

  uint32_t num_pulses;

  // stats
  uint16_t num_glitches;     // Numer of short pulses seen in this flit
  uint16_t num_badSymbols;   // Numer of symbols out of the 4B6B encoding

  // Dimming variables
  uint8_t  dim, dim_low, dim_high;
  uint8_t  rep, rep1;
  
  uint8_t  isNewFlit;  // Mark this as a new flit - the info comes from MSB of "size"

  uint8_t  tdp;  // counter of the current pulse in the TDP part of the preamble

  uint8_t  prev_bit;     // Previous bit (half bit in some cases) received
  uint16_t num_bits;     // Numer of bits received during sync. In data reception counts up to 8 (1 byte) at a time.
  int32_t  num_bytes;    // Numer of bytes in packet. Negative numbers may be used for error codes
  uint32_t payload_size; // remaining payload bits

  uint32_t idx;   // Index in the buffer. Usefull when a packet is complete in the middle of the samples buffer
  __IO uint32_t *buffer; // keeps the base of the buffer (where the samples are stored by DMA)


  uint8_t  *codep; // pointer for the "code" part of COBS
  uint8_t  *bp;    // current pointer in buffer of demodulated packet, either header or packet
  uint8_t  *pbuff;  // payload storage (temporarily also for header info)
} demodulationHandleDef;

void initDemodulation(demodulationHandleDef *, uint8_t *);
int32_t demodulate(__IO uint32_t *, demodulationHandleDef *);

