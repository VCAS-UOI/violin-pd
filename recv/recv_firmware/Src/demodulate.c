
#include "demodulate.h"
//#include "dwt.h"
#include "printf.h"

#define INVERT_BIT(a) \
   (~a & 0x01)

extern uint32_t
         cnt_inGaps,     // Number of inter-flit gaps dectected
         cnt_inFlits,    // Number of flit-starts: i.e. the FLP was complete
         cnt_tdpErrors,  // Unexpected pulse in TDP or from FLP to TDP. These flits are dropped
         cnt_badSymbols, // Number of bad 4B6B symbols. Some may be recoverable by RS decode in the app
         cnt_glitches, cnt_doubleGlitches;  // Not used.
//extern uint32_t glitchWidths[20];

uint32_t IDLE_SAMPLE_THRES;

uint32_t *p_glitchWidths;

int16_t   rll4b6b_decode(demodulationHandleDef *);
int16_t   demodGetSymbol(demodulationHandleDef *, uint8_t);
int8_t    demodSynchronize(demodulationHandleDef *, uint32_t);
void      demodTDP(demodulationHandleDef *, uint32_t);

void demodClearBuff(demodulationHandleDef *);
inline void demodError(demodulationHandleDef *, ManErrorTypeDef);

// Called when a new packet is expected
void initDemodulation(demodulationHandleDef *dh, uint8_t *pbuff)
{
  dh->num_bytes = 0;  // current packet size in bytes
  dh->bp = pbuff+1;     // set pointer to start of buffer. For COBS, it is +1 position
  dh->pbuff = pbuff;
  *pbuff = 0; // Clear first byte. The others are cleared just before they are filled
  dh->code = 1;
  dh->codep = pbuff;

  // state  and threshold are set outside this function
  // They are set for the first ever reception and then they are adjusted by
  //  the demodulate function.
  //  I could do this also for {min,max}_val, avg_samples_per_period, but the algorithm
  //  seems too weak if these change while in operation.
  dh->error = MAN_ERR_OK;

  dh->edge = EDGE_MAIN;
  dh->prev_bit = ~START_BIT & 0x01;  // The idle value
  dh->num_bits = 0;
  dh->payload_size = 0;

  dh->rll_buff = 0;
  dh->rll_bits = 0;

  dh->num_pulses = 0;
  dh->total_pulse_duration = 0;

  dh->num_glitches = 0;   // no glitches yet
  dh->num_badSymbols = 0; // no bad symbols yet
}


int32_t demodulate(__IO uint32_t *buffer, demodulationHandleDef *dh)
{
  uint8_t  new_bit, no_data;
  int16_t  lastChip;  // Must be signed! Negative values convey an error
  uint32_t pw;  // pulse width 

  if (buffer == NULL) {  // New packet is to be demodulated from the remaining samples
    buffer = dh->buffer;  // Continue in the buffer after full packet demodulation
  } else  { // new samples are for the current packet
    dh->idx = 0;  // start reading samples from the beginning of the adc buffer
  }

  for (uint32_t i = dh->idx; i < HALF_BUFF_SIZE; i++) {

    pw = *buffer++;// pulse width is what I read from the dma buffer

    if (dh->state == MAN_ERROR_RECOVERY) {
      // Stay in this state until I see a long sequence of IDLE == ~START values
      if (pw > IDLE_SAMPLE_THRES)  { // continuous samples at ~START, must be in idle phase
        initDemodulation(dh, dh->pbuff);  // reset my data to be ready for the next packet
        dh->state = MAN_SYNC;
        /* Keep count of start of flits
         */
        cnt_inGaps += 1;
        //printf("Gap cnt %d\n", cnt_inGaps);
      }
      continue; // go to next signal edge
    } else if (dh->state == MAN_SYNC) {  // In the SYNC- preamble part of the packet
      demodSynchronize(dh, pw);
    } else if (dh->state == MAN_WAIT_FOR_TDP) {  // sync is done, waiting for FLP part of preamble to finish
      if (dh->edge == EDGE_MAIN) {
        if (pw > dh->preamble_thres) {  // "long" pulse
          dh->state = MAN_TDP;
          dh->tdp = 0;
          continue;
        }
        dh->edge = EDGE_SECONDARY;
      } else {
        if (pw > dh->preamble_thres) {  // "long" pulse
          dh->state = MAN_ERROR_RECOVERY;
          dh->tdp = 0;
          cnt_tdpErrors += 1;
          continue;
        }
        dh->edge = EDGE_MAIN;
      }
    } else if (dh->state == MAN_TDP) {  // In the TDP part of preamble
      demodTDP(dh, pw);
    } else {  // any State other than SYNC, ERROR_RECOVERY, WAIT_FOR_STOP_BIT, WAIT_FOR_TDP, TDP
      
      /*
      // IMPORTANT: THIS MULT TAKES A HUGE TIME! (does the compiler emulate FP?)
	  //  If I need this, do the calculation once in the demodSynchronize function
	  //  and store it....
      if (pw <  0.2*dh->avg_pulse_width) {
        dh->num_glitches++;
        cnt_glitches++;
        if (cnt_glitches < 20)
          *p_glitchWidths++ = pw;
        if (*buffer < 0.7*dh->avg_pulse_width)
          cnt_doubleGlitches++;
        continue;  // ignore glitches
      }
      */
      no_data = 0;
      if (dh->edge == EDGE_MAIN) {
        if (pw > dh->main_pulse_thres) {  // "long" pulse
          new_bit = START_BIT;
        } else {
          new_bit = dh->prev_bit; // same as previous bit. WATCH OUT FOR END OF BUFFER!
        }
        dh->edge = EDGE_SECONDARY;
      } else {
        if (pw > dh->sec_pulse_thres) {  // "long" pulse
          new_bit = INVERT_BIT(START_BIT);
        } else {
          no_data = 1;  // no data in this pulse
        }
        dh->edge = EDGE_MAIN;
      }

      if (dh->firstEdge == 1) {  // Calculate pulse thresholds after transition from preamble to VPPM
        dh->firstEdge = 0;
        dh->main_pulse_thres = (START_BIT == 1)? dh->lo_pulse_lb_thres : dh->hi_pulse_lb_thres;
        dh->sec_pulse_thres  = (START_BIT == 1)? dh->hi_pulse_lb_thres : dh->lo_pulse_lb_thres;
      }

      if (no_data == 1)
        continue;  // no data in this pulse

      lastChip = demodGetSymbol(dh, new_bit);
      if (lastChip < 0) // Error or not a full symbol yet.
        continue;

      //if (dh->state == MAN_PAYLOAD && dh->payload_size == 24)
      //  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);

      // ------- we have a new bit/symbol --------
      // State specific code here
      if (dh->state == MAN_PACKET_SIZE) {  // determine the packet size from the header
        // TODO: parameterize this. 2 bytes for payload size, and subsequent calculation
        if (dh->num_bytes == 2) { // got all the bits of this field
          // The upper nibble of the MSB of the transmitted size, shows if it is a new flit or not.
          if ( (*(dh->bp-1) & 0xF0) != 0) { // Anything in the upper bits signifies a newFlit
            dh->isNewFlit = 1;
            *(dh->bp-1) &= 0x0F;  // clear the upper bits
            //printf("New flit seen in demod\n");
          } else {
            dh->isNewFlit = 0;
          }
          // payload_size is in symbols (bits for manchester, but nibles-4b for 4B6B)
          // WATCHOUT: IN DEBUGGING dh->payload_size gets its value much later! !!!!!
          dh->payload_size = (*(dh->bp-1) << 8) | *(dh->bp-2);
          // Temp for debugging
          //printf("FLit size %d\n", dh->payload_size);
          //dh->payload_size <<= 3; // This is in symbols. bits for Manchester, but nibbles for 4B6B (so << 1 in that case)
          dh->payload_size <<= 1; // This is in symbols. nibbles for 4B6B (so << 1 in that case)
          dh->state = MAN_PAYLOAD;
          demodClearBuff(dh);  // clear output buffer, so in the end it will contain just the data bits.
        }
      } else if (dh->state == MAN_PAYLOAD) {
        dh->payload_size--;
        if (dh->payload_size == 0) {
          *(dh->codep) = dh->code;  // Final COBS code value
          dh->num_bytes =  dh->bp - dh->pbuff;
          //printf("flit compl %d\n", cnt_inGaps);
          //printf("FLit complete %d %ld\n", dh->num_bytes, dh->bp - dh->pbuff);
          // --------- Deliver packet -----------
          dh->state = MAN_ERROR_RECOVERY;
          dh->buffer = buffer; // keep current buffer pointer in demod handler
          // and keep index in demod handler
          dh->idx = i+1; // if we were at the last item of the adc buffer, this will go over 2*HALF_BUFF_SIZE and at the next call will not enter the loop
          return dh->num_bytes;
        }
        // TODO: For a payload larger than the specified size in the header, I deliver the expected bytes and ignore the rest.
        //  For a shorter payload, I drop the packet. Is this what I should do?
      } // end if MAN_PAYLOAD
    } // end any state which gets bits (PAYLOAD, PACKET_SIZE)
  } // end of foreach sample loop
  // end of sampling data, still in the middle of a packet
  return 0;
}

inline void demodError(demodulationHandleDef *dh, ManErrorTypeDef eCode)
{
  dh->state = MAN_ERROR_RECOVERY;
  dh->error = eCode;
}

int8_t demodSynchronize(demodulationHandleDef *dh, uint32_t pw)
{
  /*
  static uint32_t syncWidths[47];
  uint32_t sum;
  int i; // for debugging

  // Use the preamble to syncronize: find number of ticks per signal edge
  syncWidths[dh->num_pulses] = pw;  // for debugging - store the widths
  */
  dh->total_pulse_duration += pw;
  dh->num_pulses += 1;
  dh->avg_pulse_width = dh->total_pulse_duration / dh->num_pulses;
  /*
  if ((pw > 1.3*dh->avg_pulse_width) || pw < 0.7*dh->avg_pulse_width) {
    // Strange pulse received, restart from the gap
    //printf("Unexpected pulse width of %d at sync phase (average so far %d)\n", pw, dh->avg_pulse_width);
    demodError(dh, MAN_ERR_SYNC);
    return 0;
  }
  */
  if (dh->num_pulses == 2*SYNC_BITS-1) {   
    /* forcing syncWidths to be kept by the compiler
    sum = 0;
    for (i=0; i < 48; i++)
        sum +=syncWidths[i];
    dh->avg_pulse_width = sum / dh->num_pulses; 
    */
    // done with the sync-preamble phase.
    //  the -1 here means we do not count the last pulse (low if START_BIT is 1) of the preamble
    dh->state = MAN_WAIT_FOR_TDP;
    demodClearBuff(dh);  // clear output buffer, to remove the header bytes and reset bit/byte counters
    dh->symbolPeriod = 2*dh->avg_pulse_width;
    dh->edge= EDGE_MAIN;  // posedge if START_BIT = 1, negedge if START_BIT = 0
    dh->firstEdge = 1;
    // In the following, the math looks a bit weird to avoid rounding errors
    //   The range of dim_low/high is 10-90, so division by 200 is meant to calculate half of s/g.
    //   Cortex M7 does have an FP unit, so I could do it all in FP and then convert to int?
    // preamble to data transition pulse thresholds
    dh->lo_pulse_p2d_thres = (uint32_t)
                             dh->symbolPeriod / 2   // the last low pulse of the preamble (fixed at 50% duty cycle)
                           + (100-dh->dim_low) * dh->symbolPeriod / 200; // plus half of the upcoming low-brightness low pulse of data (size)
    dh->hi_pulse_p2d_thres = (uint32_t)
                             dh->symbolPeriod / 2
                           + dh->dim_low * dh->symbolPeriod / 200;
    // transition from high brightness to low brightness dimming pulse thresholds
    dh->lo_pulse_h2l_thres = (uint32_t)  // Low pulse
                             (100-dh->dim_high) * dh->symbolPeriod / 100 // the full high-brightness low pulse
                           + (100-dh->dim_low)  * dh->symbolPeriod / 200; // plus half of the upcoming low-brightness low pulse
    dh->hi_pulse_h2l_thres = (uint32_t)  // High pulse
                             dh->dim_high * dh->symbolPeriod / 100
                           + dh->dim_low  * dh->symbolPeriod / 200;
    
    // transition from low brightness to high brightness dimming pulse thresholds
    dh->lo_pulse_l2h_thres = (uint32_t)
                             (100-dh->dim_low)  * dh->symbolPeriod / 100  // the full low-brightness low pulse
                           + (100-dh->dim_high) * dh->symbolPeriod / 200; // half of the upcoming high-brightness low pulse
    dh->hi_pulse_l2h_thres = (uint32_t)
                             dh->dim_low  * dh->symbolPeriod / 100
                           + dh->dim_high * dh->symbolPeriod / 200;
    // high brightness pulse thresholds
    dh->lo_pulse_hb_thres = (uint32_t) 3 * (100-dh->dim_high) * dh->symbolPeriod / 200;
    dh->hi_pulse_hb_thres = (uint32_t) 3 * dh->dim_high       * dh->symbolPeriod / 200;
    // low brightness pulse thresholds
    dh->lo_pulse_lb_thres = (uint32_t) 3 * (100-dh->dim_low) * dh->symbolPeriod / 200;
    dh->hi_pulse_lb_thres = (uint32_t) 3 * dh->dim_low       * dh->symbolPeriod / 200;

    dh->preamble_thres = (uint32_t) 3 * dh->symbolPeriod / 4;
    dh->tdp_long_thres = (uint32_t) 3 * dh->avg_pulse_width;  // TDP extra long pulses are 4 half periods, threshold is set at 3x half periods

    dh->main_pulse_thres =  (START_BIT == 1)? dh->lo_pulse_p2d_thres : dh->hi_pulse_p2d_thres;

    /* Keep count of start of flits
     */
    cnt_inFlits += 1;
    //printf("End of sync. pulse: %d\n", dh->avg_pulse_width);
    //p_glitchWidths = glitchWidths;
    cnt_glitches = 0;
    cnt_doubleGlitches = 0;
    dh->num_badSymbols = 0;
    dh->num_glitches = 0;
  } // end got all SYNC_BITS
  return 0;
}

// Doesn't let me inline this.....
inline int16_t demodGetSymbol(demodulationHandleDef *dh, uint8_t new_bit)
{
  // Shift in the new bit
  // Note: the IEEE standard uses LSB first and this is how our xmiter works
  //  Here we shift them in as received and function rll4b6b_decode restores the right order of bits
  dh->rll_buff <<= 1;
  dh->rll_buff |= (new_bit & 0x01);
  dh->rll_bits++;
  // Fine-grain, "fractional" dimming support.
  // It runs every VPPM symbol, not every 4B6B symbol !!!!!
  dh->rep++; // count VPPM symbols
 
  // Check if dim is to change and update 
  if (dh->rep == 100) {
    // Transition from from high to low brightness
    dh->main_pulse_thres = (START_BIT == 1)? dh->lo_pulse_h2l_thres : dh->hi_pulse_h2l_thres;
    dh->sec_pulse_thres  = (START_BIT == 1)? dh->hi_pulse_h2l_thres : dh->lo_pulse_h2l_thres;
    dh->dim = dh->dim_low;
    dh->rep = 0;
  } else if (dh->rep == 1) {              // when rep < rep1, but runs only once
    dh->main_pulse_thres = (START_BIT == 1)? dh->lo_pulse_lb_thres : dh->hi_pulse_lb_thres;
    dh->sec_pulse_thres  = (START_BIT == 1)? dh->hi_pulse_lb_thres : dh->lo_pulse_lb_thres;
  } else if (dh->rep == dh->rep1) {
    // Transition from from low to high brightness
    dh->main_pulse_thres = (START_BIT == 1)? dh->lo_pulse_l2h_thres : dh->hi_pulse_l2h_thres;
    dh->sec_pulse_thres  = (START_BIT == 1)? dh->hi_pulse_l2h_thres : dh->lo_pulse_l2h_thres;
    dh->dim = dh->dim_high;
  } else if (dh->rep == dh->rep1+1)  {  // when rep > rep1, but runs only once
    dh->main_pulse_thres = (START_BIT == 1)? dh->lo_pulse_hb_thres : dh->hi_pulse_hb_thres;
    dh->sec_pulse_thres  = (START_BIT == 1)? dh->hi_pulse_hb_thres : dh->lo_pulse_hb_thres;
  }
  dh->prev_bit = new_bit;   // prepare for next sample
  return rll4b6b_decode(dh);
}



void demodClearBuff(demodulationHandleDef *dh)
{
  dh->num_bits = 0;
  dh->num_bytes = 0;
  dh->code = 1;
  dh->decodedByte = 0;
  dh->codep = dh->pbuff;
  dh->bp = dh->pbuff+1;  // reset to start of buffer.
  *(dh->bp) = 0; // and clear first byte
}


/* 4B6B encoding. Original 4 bit nibble in hex, 4B6B encoding (hex), 4B6B encoding (binary)
   4B6B reverse bits (as received oldest is MSB), 4B6B reverse bits in hex, in VPPM chips (0 is 10, 1 is 01)
0, 0x0e,   001110    011100  0x1c    1001  0101  1010    0x95a
1, 0x0d,   001101    101100  0x2c    0110  0101  1010    0x65a
2, 0x13,   010011    110010  0x32    0101  1010  0110    0x5a6
3, 0x16,   010110    011010  0x1a    1001  0110  0110    0x966
4, 0x15,   010101    101010  0x2a    0110  0110  0110    0x666   <<------- !!!
5, 0x23,   100011    110001  0x31    0101  1010  1001    0x5a9
6, 0x26,   100110    011001  0x19    1001  0110  1001    0x969
7, 0x25,   100101    101001  0x29    0110  0110  1001    0x669
8, 0x19,   011001    100110  0x26    0110  1001  0110    0x696
9, 0x1a,   011010    010110  0x16    1001  1001  0110    0x996
a, 0x1c,   011100    001110  0x0e    1010  0101  0110    0xa56
b, 0x31,   110001    100011  0x23    0110  1010  0101    0x6a5
c, 0x32,   110010    010011  0x13    1001  1010  0101    0x9a5
d, 0x29,   101001    100101  0x25    0110  1001  1001    0x699
e, 0x2a,   101010    010101  0x15    1001  1001  1001    0x999
f, 0x2c    101100    001101  0x0d    1010  0101  1001    0xa59
*/

// Aris: The default changed from 0 to 1. This solves the
// problem in RS decoding when a number of bytes are misread and
// when they are all 0s, RS decoder considers it a valid block.
// Exception at 0x34 which is a special symbol in size field for duplicates
// You can tell the valid symbols, as they are in hex (0x..) form
uint8_t mapRllDecode[64] = {
  1,   1,   1, 1,   1,   1,   1,   1,   // 0x0 - 0x7
  1,   1,   1, 1,   1,   0xf, 0xa, 1,   // 0xa - 0xf
  1,   1,   1, 0xc, 1,   0xe, 0x9, 1,   // 0x10 - 0x17
  1, 0x6, 0x3, 1,   0x0, 1,   1,   1,   // 0x18 - 0x1f
  1,   1,   1, 0xb, 1,   0xd, 0x8, 1,   // 0x20 - 0x27
  1, 0x7, 0x4, 1,   0x1, 1,   1,   1,   // 0x28 - 0x2f
  1, 0x5, 0x2, 1,   0,   1,   1,   1,   // 0x30 - 0x37  -- THE 0 HERE IS NOT A MISTAKE !!!!!
  1,   1,   1, 1,   1,   1,   1,   1};

// Follows the map above. 0 indicates an invalid symbol. Used for counting demodulation errors.
uint8_t RllValidSymbol[64] = {
  0, 0, 0, 0, 0, 0, 0, 0,   // 0x0 - 0x7
  0, 0, 0, 0, 0, 1, 1, 0,   // 0xa - 0xf
  0, 0, 0, 1, 0, 1, 1, 0,   // 0x10 - 0x17
  0, 1, 1, 0, 1, 0, 0, 0,   // 0x18 - 0x1f
  0, 0, 0, 1, 0, 1, 1, 0,   // 0x20 - 0x27
  0, 1, 1, 0, 1, 0, 0, 0,   // 0x28 - 0x2f
  0, 1, 1, 0, 1, 0, 0, 0,   // 0x30 - 0x37
  0, 0, 0, 0, 0, 0, 0, 0};


inline int16_t rll4b6b_decode(demodulationHandleDef *dh)
{
  int16_t lastChip;

  if (dh->rll_bits < 6) {  // Need to receive 6 bits before decoding
    return -1;
  }

  lastChip = dh->rll_buff & 0x1;
  // Do it with look-up table to compare speed
  // I loose the bad symbol info, unless I use special codes in the table and 
  //  check the value after the lookup...
  // RESULT: I get tiny improvement of 4% from the lookup. From ~78us to 75us for a run of demod
  //*(dh->bp) |= (mapRllDecode[dh->rll_buff] << dh->num_bits);
  dh->decodedByte |= (mapRllDecode[dh->rll_buff] << dh->num_bits);
  // Count the "bad symbols"
  if (RllValidSymbol[dh->rll_buff] == 0) {
    cnt_badSymbols += 1;
  }
  /*
  //  Note: the oldest received chip is the MSB here, 
  //   but in the output it is the LSB!
  switch (dh->rll_buff) {
  case 0x1c:  // 0000
    dh->decodedByte |= (0 << dh->num_bits);
    break;
  case 0x2c:  // 0001
    dh->decodedByte |= (1 << dh->num_bits);
    break;
  case 0x32:  // 0010
    dh->decodedByte |= (2 << dh->num_bits);
    break;
  case 0x1a:  // 0011
    dh->decodedByte |= (3 << dh->num_bits);
    break;
  case 0x2a:  // 0100
    dh->decodedByte |= (4 << dh->num_bits);
    break;
  case 0x31:  // 0101
    dh->decodedByte |= (5 << dh->num_bits);
    break;
  case 0x19:  // 0110
    dh->decodedByte |= (6 << dh->num_bits);
    break;
  case 0x29:  // 0111
    dh->decodedByte |= (7 << dh->num_bits);
    break;
  case 0x26:  // 1000
    dh->decodedByte |= (8 << dh->num_bits);
    break;
  case 0x16:  // 1001
    dh->decodedByte |= (9 << dh->num_bits);
    break;
  case 0x0e:  // 1010
    dh->decodedByte |= (0xa << dh->num_bits);
    break;
  case 0x23:  // 1011
    dh->decodedByte |= (0xb << dh->num_bits);
    break;
  case 0x13:  // 1100
    dh->decodedByte |= (0xc << dh->num_bits);
    break;
  case 0x25:  // 1101
    dh->decodedByte |= (0xd << dh->num_bits);
    break;
  case 0x15:  // 1110
    dh->decodedByte |= (0xe << dh->num_bits);
    break;
  case 0x0d:  // 1111
    dh->decodedByte |= (0xf << dh->num_bits);
    break;
  case 0x34:  // special symbol in size field, duplicate flit
    dh->decodedByte |= (0 << dh->num_bits);
    //printf("Special symbol received\n");
    break;
  default:  // UKNOWN SYMBOL
    dh->decodedByte |= (0 << dh->num_bits);
    // TODO: With Reed-Solomon, I can mark these as erasures and continue with the packet...
    //printf("Unknown symbol received %x @ byte: %d bit %d\n", dh->rll_buff, dh->num_bytes, dh->num_bits);
    //demodError(dh, UNKNOWN_SYMBOL);
    //lastChip = -1;
    dh->num_badSymbols += 1;
    cnt_badSymbols += 1;
    break;
  }
  */
  dh->num_bits += 4;
  if (dh->num_bits == 8)  { // When a full byte is read
    if (dh->state == MAN_PACKET_SIZE) {
      *(dh->bp) = dh->decodedByte;
      dh->bp++; // Increment pointer to next byte in buffer
      dh->num_bytes++;// Increment byte counter
    } else {
	  // Encode new byte using COBS for payload only.
      if (dh->decodedByte) {
        *(dh->bp) = dh->decodedByte;
	    dh->bp += 1;
        dh->code += 1;
	  }
      if (!dh->decodedByte || dh->code == 0xff) {
        *(dh->codep) = dh->code;
        dh->code = 1;
        dh->codep = dh->bp;
	    dh->bp += 1;
      }
    }
    dh->decodedByte = 0; // Clear previous stuff.
    dh->num_bits = 0;// Clear bit counter
  }
  // Clear RLL buffer and bit counter
  dh->rll_bits = 0;
  dh->rll_buff = 0;
  return lastChip;
}

// Handle TDP and ~TDP. Just go down the expected pulse widths. Any diff, go to ERROR_RECOVERY.
void demodTDP(demodulationHandleDef *dh, uint32_t pw)
{
  // Ignore pulse polarity for now. It shouldn't matter
  //   I can determine it from LSB of dh->tdp: odd numbers are at MAIN_EDGE (Rising)
  //      even numbers are at SECONDARY
  switch (dh->tdp) {
  case 0: // short high
  case 4: // short high
  case 6: // short high
  case 12: // short high
  case 16: // short high  -- last pulse
  case 3: // short low
  case 7: // short low
  case 9: // short low
  case 13: // short low
  case 15: // short low
    if (pw > dh->preamble_thres) {  // "long" pulse
      // TDP fault. Skip to next interframe spacer
      cnt_tdpErrors += 1;
      dh->state = MAN_ERROR_RECOVERY;
	} else if (dh->tdp != 16) {
      dh->tdp += 1;
    } else {   // last pulse of TDP (there is short low too, but this is handled later)
      dh->state = MAN_PACKET_SIZE;      
      dh->edge= EDGE_MAIN;  // posedge if START_BIT = 1, negedge if START_BIT = 0
      dh->firstEdge = 1;
	} 
    break;
  case 1: // very long low  > 2.5 x half period
  case 10: // very long high  > 2.5 x half period
    if (pw < dh->tdp_long_thres) {  // anything but the extra long pulse
      // is a TDP fault. Skip to next interframe spacer
      cnt_tdpErrors += 1;
      dh->state = MAN_ERROR_RECOVERY;
	} else {
      dh->tdp += 1;
    }
    break;
  case 2: // long high
  case 8: // long high
  case 14: // long high
  case 5: // long low
  case 11: // long low
    if (pw < dh->preamble_thres) {  // short pulse
      // TDP fault. Skip to next interframe spacer
      cnt_tdpErrors += 1;
      dh->state = MAN_ERROR_RECOVERY;
	} else if (pw >= dh->tdp_long_thres) {  // too long pulse
      dh->state = MAN_ERROR_RECOVERY;
      cnt_tdpErrors += 1;
    } else {
      dh->tdp += 1;
    }
    break;
  default:  // should never happer
    dh->state = MAN_ERROR_RECOVERY;
    cnt_tdpErrors += 1;
  }
  return;
}
