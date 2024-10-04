
#include "main.h"
#include "string.h"

extern const uint16_t dutyCycleReduction;  // reduces the high pulse by this amount. Was used for testing. It is not fixed to 0



/*
  Convert data bits (after 4B6B encoding) to timer triplets which define output PWM signals: period (ARR), repetitions(REP), high pulse width (CCR)
  Returns the number of bits consumed, usually 1, but in some cases 2.
*/
static uint8_t chip2TimerTriplet(uint8_t *newWaves,  // returns 1 when a new timer triplet is produced
                          uint16_t *timPtr,   // pointer to store timer triplet
                          uint8_t data,       // input data: 3 LSBs are only used
                          uint8_t mergeWaves, // when 1 allows merging with previous timer triplet (incr. repetition of previous timer pulse)
                          uint16_t halfPeriod, uint32_t minPulse1, uint32_t minPulse2, uint32_t  minPulse3)
{
  uint16_t period, highPulse;
  uint8_t  bitsConsumed;

  bitsConsumed = 1;
  // Note: bits are examined from right to left: current bit, next bit, next-next (!) bit
  switch(data) {
  case 0x0: // x00: output a Short(high) - Short(low) pulses
  case 0x4:
    period = 2*halfPeriod;
    highPulse = minPulse1;
    break;
  case 0x2: // x10  - Short(high) - Long(low) pulses
  case 0x6:
    period = 4*halfPeriod-minPulse2;
    highPulse = minPulse1;
    break;
  case 0x3: // x11: output a Short(high) - Short(low) pulses - data '1'
  case 0x7:
    period = 2*halfPeriod+minPulse1-minPulse2;
    highPulse = minPulse1;
    break;
  case 0x5: // 101 - Long(high) - Long(low) pulses
    period = 4*halfPeriod+minPulse1-minPulse3;
    highPulse =  minPulse1+minPulse2;
    // This case consumes 2 bits.
    bitsConsumed = 2;
    break;
  case 0x1: // 001 - Long(high) - Short(low)
    period = 2*halfPeriod+minPulse1;
    highPulse = minPulse1+minPulse2;
    // This case consumes 2 bits.
    bitsConsumed = 2;
    break;
  default:
    // All cases are covered, but if we get rubish inputs, panic
    //printf("Got rubish input to modulate\n");
    Error_Handler();
    period = 0;
    highPulse = 0;
  } // end switch

  if ((*(timPtr-3) == period-1) && (*(timPtr-1) == highPulse) && mergeWaves) {
    // Previous timer triplet is identical, so just increase the repetition counter
    *(timPtr-2) += 1;
    *newWaves = 0;
  } else {
    *timPtr     = period-1;
    *(timPtr+1) = 0;
    *(timPtr+2) = highPulse;
    *newWaves = 1;
  }
  return bitsConsumed;
}

// Identical to chip2TimerTriplet with all minPulses equal.
static uint8_t mychip2TimerTriplet(uint8_t *newWaves, uint16_t *timPtr, uint8_t data, uint8_t mergeWaves,
                          uint16_t halfPeriod, uint32_t minPulse)
{
  return chip2TimerTriplet(newWaves, timPtr, data, mergeWaves,
                          halfPeriod, minPulse, minPulse, minPulse);
}

// Table storing pre-calculated timer triple ("waves") for all combinations of dimming fractions (2), all 6B symbols (6B - not all used), all possible 2bits from the following symbol
struct {
  uint16_t waves[3*6];
  uint8_t  len;     // The number of timer triples stored in waves
  uint8_t  extraBit; // Set to 1, when waves stores 1 bit from the following symbol
} mapWaves[512];

/*
 Creates a conversion table from 6B symbols to waves (timer triples). This is done once and then looked up to speed up the modulation task
 The algorithm scans the symbol bits from lsb to msb.
 Each generated wave depends on the current bit and the following 2 bits. So at the "end" of a 6B symbol, up to 2 LSBs of the next symbol are looked-up.
 In most cases each bit of a 6B symbol produces a single wave, but there are bit combinations which consume 2 symbol bits to produce one wave.
 Therefore, a wave sequence for a symbol may consume a bit from the following symbol. This is recorded in the extraBit field of the output structure.
 To support VPPM, there are 2 parts of the table, one for the low dimming fraction (k1) and one for the high (k2).
 So in total the table has 2 (dimming) * 2^2 (next bits) * 2^6 (6B symbols) entries.
 TODO: Not all combinations of 6B symbols are used, so many/most of the generated waves are never used!
 */
void createModTable(uint16_t macDim, uint16_t halfPeriod)
{
  uint8_t newWaves;
  uint16_t rllData;
  uint16_t *timBuffer;
  uint32_t minPulse;
  uint16_t index;
  int16_t  k1, k2;
  uint8_t  bitsConsumed;


  k1 = (uint8_t) floorf(macDim / 100.0);
  k2 = (uint8_t) ceilf(macDim / 100.0);

  for (uint8_t k = 0; k < 2; k++) {  // For each of the two brightness fractions (k1, k2)
    minPulse = 2*halfPeriod * ((k==0)? k1:k2) / 10;
    for (uint8_t i = 0; i < 64; i++) {  // For each valid nibble -> 6B symbols, so 2^6=64 combinations
      for (uint8_t j = 0; j < 4; j++) {  // all possible next 2 bit values (from the following 6B symbol)
        // The index of the table is {k[0], j[5:0], i[1:0]}, i.e. 9 bits, 512 entries in total
        index = (k<<8) | (j<<6) | i;
        timBuffer = (uint16_t *) &(mapWaves[index].waves);
        mapWaves[index].len = 0;
        mapWaves[index].extraBit = 0;
        for (uint8_t bitCounter = 0; bitCounter < 6; ) {
          rllData = (    ((j << 6) | (i & 0x3F))  // Generate the combination of next 2 bits (j) and 6B symbol (i)
                      >> bitCounter              ) & 0x7;  // get the 3 bits needed to generate a wave (timer triple)
          bitsConsumed = mychip2TimerTriplet(&newWaves, timBuffer, rllData,
                              (bitCounter != 0), // merge waves except for the 1st bit of a symbol.
                              halfPeriod, minPulse-dutyCycleReduction);
          if (newWaves == 1) {
            timBuffer += 3;
            mapWaves[index].len += 1;
          }
          if ( (bitsConsumed == 2) && (bitCounter == 5)) {
            // We consume a bit of the following symbol
            mapWaves[index].extraBit = 1;
          }
          bitCounter += bitsConsumed;
        }  // end bitCounter
      }
    }  // end i
  }  // end k
}

/*
 Get a 6B symbol and 2 bits from the next 6B symbol (in the returned byte MSBs), packed in the returned byte.
 When the end of the message is reached (byteCounter+1 < l_size), nextPacketByte is used as the next symbol 
*/
static inline uint8_t getSymbol(uint8_t *inBytes, int8_t bitCounter, int32_t byteCounter, uint16_t l_size, uint8_t nextPacketByte)
{
  if (byteCounter+1 < l_size)
    return ((( *(inBytes+1) << 8 ) | *inBytes) >> bitCounter) & 0xff;
  else
    return (((nextPacketByte << 8) | *inBytes) >> bitCounter) & 0xff;
}



static inline uint8_t  getBits(uint8_t *, int8_t, int32_t, uint16_t, uint8_t);
uint32_t rll4b6b_encode(uint8_t *, uint16_t, uint8_t *);


// Note: the period stored in "waves" is -1 because the counter counts up to that number
//  This skews the duty cycle towards the low pulse.
#define ADD_WAVE(period, highPulse, isNotHeader) do {\
    if ((*(timBuffer-3) == (period)-1) && (*(timBuffer-1) == (highPulse)) && (isNotHeader))\
      *(timBuffer-2) += 1;\
    else {\
      *timBuffer++ = (period)-1;\
      *timBuffer++ = 0;\
      *timBuffer++ = (highPulse);\
      waves += 1;\
    }\
  } while(0)

// Works with UP counting only.  PWM1 (high first), when IDLE_SIGNAL==0, PWM2 (low first) when IDLE_SIGNAL==1
uint32_t modulatePacket(
         uint8_t *payload,   // The payload (4B6B RLL is already done)
         uint32_t data_size, // The actual data size without RLL. This is used to generate the header
         uint32_t size,      // The payload size in bytes (includes RLL)
         uint16_t halfPeriod, uint16_t macDim,  // Period and dimming info
         uint16_t *timBuffer,     // Buffer where the waves are output
         uint16_t *altTimBuffer,  // alternate waves buffer where the modified header is stored. Used for retransmitions of the same frame
         uint16_t **size2TimBuff  // returns the pointer in timBuffer, where the alternate header will be written for retransmitions of the same frame
         )
{
  uint32_t waves, bitSize;
  uint32_t phase;
  uint32_t minPulse1, minPulse2, minPulse3;
  int16_t  rep, k1, k2, rep2, rep1;
  int16_t  altRep = 0;
  uint16_t *altTimBufferPtr = altTimBuffer;
  uint8_t nextPacketByte;
  uint8_t *inBytes;
  uint8_t data, bitsConsumed, newWaves, mergeWaves;
  uint8_t header[2], rllHeader[3];
  uint8_t k;
  uint16_t index;

  // This is just an estimate. Must do it properly....
  /*
  if ((size+5) * 8 > MAX_PULSES)
    return -1;
  */
  
  // Preample is already prepared.
  
  // The size is the original data + parity from RS. We do not count the extra added by rll, as the demodulator
  //   does not need them.
  if (((data_size >> 8) & 0xF0) != 0) {
    //printf("Got too much to modulate\n");
    Error_Handler();
  }

  // Plant the special code in upper nible of MSB of size.
  //  This bit signals to the receiver that this is not a re-transmission of the previous frame
  //  It is not used anymore, but could be usefull in the future.
  header[1] = ((data_size >>8) & 0x0F) | 0x10;
  header[0] = (data_size & 0xFF);
  // Encode header using RLL code
  rll4b6b_encode(header, 2, rllHeader);

// --------------------------------------------------------------------------------------------------------------
// 
// --------------------------------------------------------------------------------------------------------------
  // Dimming using the algorithm in clause 8.5.2.3 of IEEE std
  k1 = (uint8_t) floorf(macDim / 100.0);
  k2 = (uint8_t) ceilf(macDim / 100.0);
  rep2 = macDim - 100 * k1;
  rep1 = 100 - rep2;

  rep = 0;  // counts the number of bits for "fractional dimming"
  waves = 0;
  phase = 1;

  // Add the last "wave" of the preample as its period depends on the 1st bit
  //  However, don't increment the preample wave repetition counter!
  if ((rllHeader[0] & 0x1) == 0)
    ADD_WAVE(2*halfPeriod, halfPeriod-dutyCycleReduction, 0);
  else
    ADD_WAVE(4*halfPeriod-(2*halfPeriod * k1 / 10), halfPeriod-dutyCycleReduction, 0);
  
  while (phase != 0 ) {
    // Inelegant hack to start with header. May have to extend, if I add CRC at the end....
    if (phase == 1) {  // Modulate the header 
      inBytes = &rllHeader[0];
      bitSize = 3*8;   // with 4b6b RLL, this is now 3 bytes
      nextPacketByte = *payload; // 1st payload byte after the header
    } else if (phase == 2) {  // Modulate the payload
      inBytes = payload;
      bitSize = size * 8;
      nextPacketByte = ~(*(payload+size-1)); // Last byte of payload, inverted
    } else {  // Modulate extra symbols at the end of payload to complete the 100 bits per dimming fraction
      // These are dummy and will be ignored at the receiver.
      // For now, just start with the data again.
      inBytes = payload;
      nextPacketByte = ~(*(payload+size-1));
      bitSize = 100-rep;  // rep gets 1 extra inc at the loop exit
    }

    for (uint32_t bitCounter = 0; bitCounter < bitSize; ) {
      // Calculate the minimum high pulse duration based on k1, k2
      //  there are 3 copies because at the boundaries we need to know the pulse duration for the following 2 periods.
      minPulse1 = minPulse2 = minPulse3 = 2*halfPeriod * k1 / 10;
      if ((rep >= rep1-2) && (rep < 98))
        minPulse3 = 2*halfPeriod * k2 / 10; // 3rd symbol uses 2ndary dimming
      if ((rep >= rep1-1) && (rep < 99))
        minPulse2 = 2*halfPeriod * k2 / 10; // 2nd symbol uses 2ndary dimming
      if (rep >= rep1)
        minPulse1 = 2*halfPeriod * k2 / 10; // this symbol uses 2ndary dimming
      
      // Decide when waves should merge and when not and keep info for alternate header
      mergeWaves = 1;
      if (phase == 1 && bitCounter >= 18 && bitCounter <= 21)  { // Upper nibble of size, corresponds to bits 18-23
        if (bitCounter == 18)  { 
          *size2TimBuff = timBuffer; // keep current timbuffer position.
          altRep = rep; // and current rep number
        }
        mergeWaves = 0; // force the non merging of pulses, so I can replace them later!
      }

      // Check if we may cross a boundary if we demodulate a whole symbol
      //  up to 7 bits may be consumed per 6b symbol (the 7th would be from the following symbol)
      if (  (phase == 1)   // go slow while modulating the header
         || ((rep >= rep1-7) && (rep <= rep1))  // may cross rep1
         || (rep >= 100-7)) { // may cross 100
         // Go bit by bit
        data = getBits(inBytes, bitCounter % 8, bitCounter / 8, bitSize/8, nextPacketByte);
        bitsConsumed = chip2TimerTriplet(&newWaves, timBuffer, data, mergeWaves,
                            halfPeriod, minPulse1-dutyCycleReduction, minPulse2-dutyCycleReduction, minPulse3-dutyCycleReduction);
        waves += newWaves;
        if (newWaves == 1)
          timBuffer += 3;
      } else {
        newWaves = 0;
        // get 6b symbol + 2 following bits.  find equivalent data symbol, lookup
        data = getSymbol(inBytes, bitCounter % 8, bitCounter / 8, bitSize/8, nextPacketByte);
        k = (rep >= rep1);
        //index = (k<<6) | (((data >> 6) & 0x3) << 6) | (data & 0x3f);
        index = (k<<8) | (data & 0xff);

        // Check first triplet. Could match previous -> increment ARR, else just copy it.
        if ((*(timBuffer-3) == mapWaves[index].waves[0]) && (*(timBuffer-1) == mapWaves[index].waves[2]) && mergeWaves) {
          *(timBuffer-2) += mapWaves[index].waves[1] + 1;
        } else {
          *timBuffer     = mapWaves[index].waves[0];
          *(timBuffer+1) = mapWaves[index].waves[1];
          *(timBuffer+2) = mapWaves[index].waves[2];
          timBuffer += 3;
          newWaves += 1;
        }
        if (mapWaves[index].len > 1) {
          // copy any remaining waves
          memcpy(timBuffer, &(mapWaves[index].waves[3]), 3*(mapWaves[index].len-1) * sizeof(uint16_t));
          newWaves += (mapWaves[index].len-1);
          timBuffer += (mapWaves[index].len-1)*3;
        }
        waves += newWaves;
        bitsConsumed = 6 + mapWaves[index].extraBit;
      }

      // I haven't see this advance 2 bytes, so 
      if ((bitCounter % 8) + bitsConsumed >= 8)
        inBytes++;
      rep += bitsConsumed;
      
      if (rep >= 100) {  // restart the dimming counter
        rep -= 100;
      }
      bitCounter += bitsConsumed;
    } // endfor bitCounter
    phase += 1;
    if (phase == 4) // no extra dimming symbols to send
      phase = 0;
  } // endfor phase
  // --------------------------------------------------------------------------------------------------------------
  // Convert the series of pulses (always alternating polarity) into PWM info: (ARR, RCR, CCR)
  //  -- ARR (Autoreload Register) sets the period of the signal
  //  -- RCR (Repetition Count Register) sets the number of times this part of the waveform repeats
  //  -- CCR (Counter Compare Register) sets the "duty cycle", 
  //      the time the signal is high (PWM1), when IDLE_SIGNAL is 0
  //      or the time the signal is low (PWM2), when IDLE_SIGNAL is 1
  //  Each "period" must have a high and then a low pulse (IDLE 0)
  //  The pulse duration (expressed as a number which is eventually stored in CCR) for 50% duty cycle
  //   is passed in halfPeriod. So a "short"-"short" signal has a period of 2xhalfPeriod and the duty cycle
  //   is 50%, so ARR = 2*halfPeriod and CCR=halfPeriod. Similarly all other combinations are formed.
  // --------------------------------------------------------------------------------------------------------------

  // Add two dummy long pulses at the end. I don't use the repetition feature here
  //  so that we get DMA finished interrupt, when the real data have been transmitted
  for (uint8_t i=0; i<2; i++) {
    *timBuffer++ = 4*halfPeriod;
    *timBuffer++ = 0;
    *timBuffer++ = 0;  // force signal to 1, when PWM2-SIGNAL=1, or to 0 when PWM1/IDLE=0 
  }

  // Prepare alternate header for retransmitions.
  // Instead of doing this on pre-RLL coded data which would require re-modulating all the frame
  //  we replace a specific symbol for the upper nibble of the header (size).
  rep = altRep;
  rllHeader[2] &= 0x3; // Clear 6B code of upper nible of MSB of size
  rllHeader[2] |= 0x2c;  // Plant the special 6B code for "repeat flit"  (001011 - shifted 2 bits left)
  for (uint32_t bitCounter = 18; bitCounter < 21; ) {
    minPulse1 = minPulse2 = minPulse3 = 2*halfPeriod * k1 / 10;
    if ((rep >= rep1-2) && (rep < 98))
      minPulse3 = 2*halfPeriod * k2 / 10; // 3rd symbol uses 2ndary dimming
    if ((rep >= rep1-1) && (rep < 99))
      minPulse2 = 2*halfPeriod * k2 / 10; // 2nd symbol uses 2ndary dimming
    if (rep >= rep1)
      minPulse1 = 2*halfPeriod * k2 / 10; // this symbol uses 2ndary dimming

    data = getBits(&rllHeader[2], bitCounter % 8, bitCounter / 8, 3, *payload);
    bitsConsumed = chip2TimerTriplet(&newWaves, altTimBufferPtr, data, 0,
                        halfPeriod, minPulse1-dutyCycleReduction, minPulse2-dutyCycleReduction, minPulse3-dutyCycleReduction);
    // I know we won't move to another byte of the data
    rep += bitsConsumed;
    if (newWaves == 1)
        altTimBufferPtr += 3;
    
    if (rep >= 100) {
      rep -= 100;
    }
    bitCounter += bitsConsumed;
  }

  /*
  if ((waves+3)*3 > MAX_PULSES)  // It's too late here, as I would have buggered the other waveform too!
      return -1;
  */
  // return the number of uint16_t produced 
  return (waves+2) * 3;
}

/*
  Returns 3 bits from *inBytes, from bit position bitCounter upwards.
  When the bits from *inBytes are not enough to form 3 bits, they are filled with bits from the next byte *(inBytes+1)
  and when the array is exhausted (byteCounter+1 < l_size), they are filled from nextPacketByte
  bitCounter range is 0-7
*/
static inline uint8_t getBits(uint8_t *inBytes, int8_t bitCounter, int32_t byteCounter, uint16_t l_size, uint8_t nextPacketByte)
{
  if (byteCounter+1 < l_size)
    return (((*(inBytes+1) << 8) | *inBytes) >> bitCounter) & 0x7;
  else
    return (((nextPacketByte << 8) | *inBytes) >> bitCounter) & 0x7;
}

uint8_t map4B6B[16] = {0x0e, 0x0d, 0x13, 0x16, 0x15, 0x23, 0x26, 0x25, 0x19, 0x1a, 0x1c, 0x31, 0x32, 0x29, 0x2a, 0x2c};

// byteLen must be a multiple of 2. Otherwise I'd have to decode a 6B symbol at the decoder and use only part of it...
/*
 Encode an array of bytes (in) using 4B6B encoding.
 The size of in must be a multiple of 2, as each pair of input bytes, produces 4 6B symbols,
 which get packed into 3 bytes.
 Returns the size of the output: 3/2 * size of input (in byteLen)

The following diagram shows the output format.
 ----in[a+1]-----,  -----in[a]------
 nibbleD, nibbleC,  nibbleB, nibbleA
- convert each 4 bit nibble to 6 bits, using lookup table map4B6B,
  producing  D C B A, each 6 bits ([5:0]) long
D[5:0]|C[5:4], C[3:0]|B[5:2], B[1:0]|A[5:0]
--out[a+2]---, --out[a+1]---, --out[a+0]---
*/
uint32_t rll4b6b_encode(uint8_t *in, uint16_t byteLen, uint8_t *out)
{
  uint8_t  nibble;
  uint8_t  temp;

  for (uint32_t nibbleCounter = 0; nibbleCounter < byteLen*2; nibbleCounter++) {
    nibble = (*in >> (4 * (nibbleCounter % 2))) & 0x0F;
    temp = map4B6B[nibble];
    switch (nibbleCounter % 4) {
    case 0:
      *out = temp;
      break;
    case 1:
      *out++ |= (temp << 6);
      *out = ((temp >> 2) & 0x0F);
      break;
    case 2:
      *out++ |= (temp << 4);
      *out = ((temp >> 4) & 0x03);
      break;
    case 3:
      *out++ |= temp << 2;
      break;
    }
    if (nibbleCounter % 2 == 1)
        in++;
  }
  return (3*byteLen) / 2;
}

