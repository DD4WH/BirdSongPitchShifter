/*
 * Pitch shifting in the time domain
 * 
 * implements pitch shifting in the time domain and is based on an idea by Lang Elliott & Herb Susmann for the "Hear birds again"-project,
 * specifically for the now deprecated SongFinder pitch shifter units. 
 * The algorithm can shift the audio down by a factor of two (one octave), three (1.5 octaves) or four (two octaves). 
 * Find a graph showing the implementation for the case of downshifting by 4 here: https://github.com/DD4WH/BirdSongPitchShifter#readme
 * 
 * Many thanks go to Harold Mills & Lang Elliott for explaining this algorithm to me and answering my questions ! :-) 
 * https://hearbirdsagain.org/
 * 
 * (c) Frank DD4WH 2022-10-05
 * 
 * uses Teensy 4.0 and external ADC / DAC connected on perf board with ground plane
 * 
 *  PCM5102A DAC module
    VCC = Vin
    3.3v = NC
    GND = GND
    FLT = GND
    SCL = 23 / MCLK via series 100 Ohm
    BCK = BCLK (21)
    DIN = TX (7)
    LCK = LRCLK (20)
    FMT = GND
    XMT = 3.3V (HIGH)
    
    PCM1808 ADC module:    
    FMT = GND
    MD1 = GND
    MD0 = GND
    GND = GND
    3.3V = 3.3V --> ADC needs both: 5V AND 3V3
    5V = VIN
    BCK = BCLK (21) via series 100 Ohm
    OUT = RX (8)
    LRC = LRCLK (20) via series 100 Ohm
    SCK = MCLK (23) via series 100 Ohm
    GND = GND
    3.3V = 3.3V
 *  
 * MIT license
 */

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <arm_math.h>
#include <arm_const_structs.h> // in the Teensy 4.0 audio library, the ARM CMSIS DSP lib is already a newer version
#include <utility/imxrt_hw.h>

#define DEBUG
#define PIH 1.5707963267948966192313216916398f
#define FOURPI  (2.0 * TWO_PI)
#define SIXPI   (3.0 * TWO_PI)
int32_t sum;
int idx_t = 0;
float32_t mean;
int16_t *sp_L;
int16_t *sp_R;
double SAMPLE_RATE = 44100;
float32_t audio_gain = 4.0f; 

// this audio comes from the codec by I2S
AudioInputI2S            i2s_in;
AudioRecordQueue         Q_in_L;
AudioRecordQueue         Q_in_R;
AudioMixer4              mixleft;
AudioMixer4              mixright;
AudioPlayQueue           Q_out_L;
AudioPlayQueue           Q_out_R;
AudioOutputI2S           i2s_out;
//AudioOutputUSB           i2s_out;

AudioConnection          patchCord1(i2s_in, 0, Q_in_L, 0);
AudioConnection          patchCord2(i2s_in, 1, Q_in_R, 0);

AudioConnection          patchCord3(Q_out_L, 0, mixleft, 0);
AudioConnection          patchCord4(Q_out_R, 0, mixright, 0);
AudioConnection          patchCord9(mixleft, 0,  i2s_out, 1);
AudioConnection          patchCord10(mixright, 0, i2s_out, 0);

int shift = 4; 
#define BLOCK_SIZE 128
const int N_BLOCKS = 6; //9; // 6 blocks á 128 samples  = 768 samples. No. of samples has to be dividable by 2 AND by 3 AND by 4 // 6 blocks of 128 samples == 768 samples = 17.4ms
const int WINDOW_LENGTH = N_BLOCKS * BLOCK_SIZE;
const int WINDOW_LENGTH_D_2 = WINDOW_LENGTH / 2;
const int IN_BUFFER_SIZE = WINDOW_LENGTH;
#define HOPSIZE_4 WINDOW_LENGTH/4
#define HOPSIZE_3 WINDOW_LENGTH/3
float32_t in_buffer_L[4][IN_BUFFER_SIZE];
float32_t in_buffer_R[4][IN_BUFFER_SIZE];
float32_t out_buffer_L[IN_BUFFER_SIZE];
float32_t out_buffer_R[IN_BUFFER_SIZE];
float32_t add_buffer_L[IN_BUFFER_SIZE / 2];
float32_t add_buffer_R[IN_BUFFER_SIZE / 2];
float32_t window[N_BLOCKS * BLOCK_SIZE];
int buffer_idx = 0;

const float32_t n_att = 90.0; // desired stopband attenuation
const int num_taps = 36; // can be divided by 2, 3 and 4 
// interpolation-by-N
// num_taps has to be a multiple integer of interpolation factor L
// pState is of length (numTaps/L)+blockSize-1 words where blockSize is the number of input samples processed by each call
arm_fir_interpolate_instance_f32 interpolation_R;
float32_t DMAMEM interpolation_R_state [num_taps / 2 + N_BLOCKS * BLOCK_SIZE / 2]; 
arm_fir_interpolate_instance_f32 interpolation_L;
float32_t DMAMEM interpolation_L_state [num_taps / 2 + N_BLOCKS * BLOCK_SIZE / 2]; 
float32_t DMAMEM interpolation_coeffs[num_taps];
int hop0 = 0;
int hop1 = 1;
int hop2 = 2;
int hop3 = 3;

void setup() {
  Serial.begin(115200);
  delay(100);

  AudioMemory(60); // must be high enough to deliver 16 stereo blocks = 32 * 128 samples at any point in time! 
  delay(100);

  /****************************************************************************************
     Audio Shield Setup
  ****************************************************************************************/
  mixleft.gain(0, 1.0);
  mixright.gain(0, 1.0);

  /****************************************************************************************
     begin to queue the audio from the audio library
  ****************************************************************************************/
//  setI2SFreq(SAMPLE_RATE);
  
  Serial.println("DD4WH time domain pitch shifter following Hear Birds Again by Harolds Mills");

  for(unsigned idx=0; idx < WINDOW_LENGTH; idx++)
  { // von Hann window
     window[idx] = 0.5f * (1.0f - cosf(TWO_PI * (float)idx / ((float)(WINDOW_LENGTH - 1))));  
    // Blackman-Nuttall
    //window[idx] = 0.3635819f - 0.4891775f*cosf(2.0*M_PI*(float)idx/((float)(WINDOW_LENGTH-1))) + 0.1365995*cosf(FOURPI*(float)idx/((float)(WINDOW_LENGTH-1))) - 0.0106411f*cosf(SIXPI*(float)idx/((float)(WINDOW_LENGTH-1)));  
  }

  // Interpolation filter
  // the interpolation filter is AFTER the upsampling, so it has to be in the target sample rate!
  calc_FIR_coeffs (interpolation_coeffs, num_taps, (float32_t)4000, n_att, 0, 0.0, SAMPLE_RATE);
  if (arm_fir_interpolate_init_f32(&interpolation_R, (uint8_t)shift, num_taps, interpolation_coeffs, interpolation_R_state, BLOCK_SIZE * N_BLOCKS / (uint32_t)shift)) 
  {
    Serial.println("Init of interpolation failed");
    while(1);
  }
  if (arm_fir_interpolate_init_f32(&interpolation_L, (uint8_t)shift, num_taps , interpolation_coeffs, interpolation_L_state, BLOCK_SIZE * N_BLOCKS / (uint32_t)shift)) 
  {
    Serial.println("Init of interpolation failed");
    while(1);
  }
  delay(100);
  Q_in_L.begin();
  Q_in_R.begin();

} // END OF SETUP


void loop() {
  elapsedMicros usec = 0;
  // are there at least N_BLOCKS buffers in each channel available ?
    if (Q_in_L.available() > N_BLOCKS + 0 && Q_in_R.available() > N_BLOCKS + 0)
    {
      // get audio samples from the audio  buffers and convert them to float
      for (unsigned i = 0; i < N_BLOCKS; i++)
      {
        sp_L = Q_in_L.readBuffer();
        sp_R = Q_in_R.readBuffer();

        // convert to float one buffer_size
        // float_buffer samples are now standardized from > -1.0 to < 1.0
        arm_q15_to_float (sp_L, &in_buffer_L[buffer_idx][BLOCK_SIZE * i], BLOCK_SIZE); // convert int_buffer to float 32bit
        arm_q15_to_float (sp_R, &in_buffer_R[buffer_idx][BLOCK_SIZE * i], BLOCK_SIZE); // convert int_buffer to float 32bit
        Q_in_L.freeBuffer();
        Q_in_R.freeBuffer();
      }
 
      /**********************************************************************************
          Time Domain pitch shift algorithm "OLA" by Harald Mills "Hear birds again" 
       **********************************************************************************/

      /**********************************************************************************
          1 Windowing 
       **********************************************************************************/

//             we apply the second half of the window to the first half of the input buffer
//             works for N==2 
        if(shift == 2)
        {
            for (unsigned i = 0; i < WINDOW_LENGTH_D_2; i++)
            {
              in_buffer_L[0][i] = in_buffer_L[0][i] * window[i + WINDOW_LENGTH_D_2];
              in_buffer_R[0][i] = in_buffer_R[0][i] * window[i + WINDOW_LENGTH_D_2];
            }
//             we apply the first half of the window to the second half of the input buffer
    
            for (unsigned i = 0; i < WINDOW_LENGTH_D_2; i++)
            {
              in_buffer_L[0][i + WINDOW_LENGTH_D_2] = in_buffer_L[0][i + WINDOW_LENGTH_D_2] * window[i];
              in_buffer_R[0][i + WINDOW_LENGTH_D_2] = in_buffer_R[0][i + WINDOW_LENGTH_D_2] * window[i];
            }
        }
        else if(shift == 3)
        {
           for (unsigned i = 0; i < WINDOW_LENGTH; i++)
           {
              in_buffer_L[buffer_idx][i] = in_buffer_L[buffer_idx][i] * window[i];
              in_buffer_R[buffer_idx][i] = in_buffer_R[buffer_idx][i] * window[i];
           }          
        }
        else if(shift == 4)
        {
           for (unsigned i = 0; i < WINDOW_LENGTH; i++)
           {
              in_buffer_L[buffer_idx][i] = in_buffer_L[buffer_idx][i] * window[i];
              in_buffer_R[buffer_idx][i] = in_buffer_R[buffer_idx][i] * window[i];
           }
        } // end shift == 4

    
      /**********************************************************************************
          2 Overlap & Add 
       **********************************************************************************/
        if(shift == 2)
        {
            for (unsigned i = 0; i < WINDOW_LENGTH_D_2; i++)
            {
              add_buffer_L[i] = in_buffer_L[0][i] + in_buffer_L[0][i + WINDOW_LENGTH_D_2];
              add_buffer_R[i] = in_buffer_R[0][i] + in_buffer_R[0][i + WINDOW_LENGTH_D_2];
            }
        }
        else if(shift == 3)
        {   // index of in_buffer    [0][x]  [1][x]  [2][x]  
            if(buffer_idx==2)       {hop0=2, hop1=1, hop2=0;}
            else if(buffer_idx==1)  {hop0=1, hop1=0, hop2=2;}
            else if(buffer_idx==0)  {hop0=0, hop1=2, hop2=1;}
            hop0=hop0*HOPSIZE_3;
            hop1=hop1*HOPSIZE_3;
            hop2=hop2*HOPSIZE_3;
            for (unsigned i = 0; i < HOPSIZE_3; i++)
            {
                add_buffer_L[i] = in_buffer_L[0][i + hop0] + in_buffer_L[1][i + hop1] + in_buffer_L[2][i + hop2]; 
                add_buffer_R[i] = in_buffer_R[0][i + hop0] + in_buffer_R[1][i + hop1] + in_buffer_R[2][i + hop2]; 
            }

            buffer_idx = buffer_idx + 1;  // increment
            if (buffer_idx >=3) {buffer_idx = 0;} // flip-over           
        }
        else if(shift == 4)
        {   // index of in_buffer    [0][x]  [1][x]  [2][x]  [3][x]
            if(buffer_idx == 3)     {hop0=3, hop1=2, hop2=1, hop3=0;}
            else if(buffer_idx==2)  {hop0=2, hop1=1, hop2=0, hop3=3;}
            else if(buffer_idx==1)  {hop0=1, hop1=0, hop2=3, hop3=2;}
            else if(buffer_idx==0)  {hop0=0, hop1=3, hop2=2, hop3=1;}
            hop0=hop0*HOPSIZE_4;
            hop1=hop1*HOPSIZE_4;
            hop2=hop2*HOPSIZE_4;
            hop3=hop3*HOPSIZE_4;
            for (unsigned i = 0; i < HOPSIZE_4; i++)
            {
                add_buffer_L[i] = in_buffer_L[0][i + hop0] + in_buffer_L[1][i + hop1] + in_buffer_L[2][i + hop2] + in_buffer_L[3][i + hop3]; 
                add_buffer_R[i] = in_buffer_R[0][i + hop0] + in_buffer_R[1][i + hop1] + in_buffer_R[2][i + hop2] + in_buffer_R[3][i + hop3]; 
            }

            buffer_idx = buffer_idx + 1;  // increment
            if (buffer_idx >=4) {buffer_idx = 0;} // flip-over  
        }

      /**********************************************************************************
          3 Interpolate 
       **********************************************************************************/

      // receives 512 samples and makes 1024 samples out of it
      // interpolation-by-2
      // interpolation-in-place does not work
      // blocksize is BEFORE zero stuffing
      // recycle in_buffer as temporary buffer
      if(shift == 2)
      {
          arm_fir_interpolate_f32(&interpolation_L, add_buffer_L, in_buffer_L[0], BLOCK_SIZE * N_BLOCKS / (uint32_t)(shift));
          arm_fir_interpolate_f32(&interpolation_R, add_buffer_R, in_buffer_R[0], BLOCK_SIZE * N_BLOCKS / (uint32_t)(shift));
          // do some scaling / gain application after the interpolation
          arm_scale_f32(in_buffer_L[0], audio_gain, out_buffer_L, BLOCK_SIZE * N_BLOCKS);
          arm_scale_f32(in_buffer_R[0], audio_gain, out_buffer_R, BLOCK_SIZE * N_BLOCKS);

      }
      else if(shift == 3)
      {
          arm_fir_interpolate_f32(&interpolation_L, add_buffer_L, in_buffer_L[buffer_idx], BLOCK_SIZE * N_BLOCKS / (uint32_t)(shift));
          arm_fir_interpolate_f32(&interpolation_R, add_buffer_R, in_buffer_R[buffer_idx], BLOCK_SIZE * N_BLOCKS / (uint32_t)(shift));
          // do some scaling / gain application after the interpolation
          arm_scale_f32(in_buffer_L[buffer_idx], audio_gain, out_buffer_L, BLOCK_SIZE * N_BLOCKS);
          arm_scale_f32(in_buffer_R[buffer_idx], audio_gain, out_buffer_R, BLOCK_SIZE * N_BLOCKS);
      }
      else if(shift == 4)
      {
          arm_fir_interpolate_f32(&interpolation_L, add_buffer_L, in_buffer_L[buffer_idx], BLOCK_SIZE * N_BLOCKS / (uint32_t)(shift));
          arm_fir_interpolate_f32(&interpolation_R, add_buffer_R, in_buffer_R[buffer_idx], BLOCK_SIZE * N_BLOCKS / (uint32_t)(shift));
          // do some scaling / gain application after the interpolation
          arm_scale_f32(in_buffer_L[buffer_idx], audio_gain, out_buffer_L, BLOCK_SIZE * N_BLOCKS);
          arm_scale_f32(in_buffer_R[buffer_idx], audio_gain, out_buffer_R, BLOCK_SIZE * N_BLOCKS);
      }

      /**********************************************************************************
           
       **********************************************************************************/

      /**********************************************************************************
          Just copy input into output buffer for testing
       **********************************************************************************/
/*        for (unsigned i = 0; i < WINDOW_LENGTH; i++)
        {
          out_buffer_L[i] = in_buffer_L[i] * audio_gain;
          out_buffer_R[i] = in_buffer_R[i] * audio_gain;
        } */

       /**********************************************************************
          CONVERT TO INTEGER AND PLAY AUDIO - Push audio into I2S audio chain
       **********************************************************************/
      for (unsigned i = 0; i < N_BLOCKS; i++)
        {
          sp_L = Q_out_L.getBuffer();    
          sp_R = Q_out_R.getBuffer();
          arm_float_to_q15 (&out_buffer_L[BLOCK_SIZE * i], sp_L, BLOCK_SIZE); 
          arm_float_to_q15 (&out_buffer_R[BLOCK_SIZE * i], sp_R, BLOCK_SIZE);
          Q_out_L.playBuffer(); // play it !  
          Q_out_R.playBuffer(); // play it !
        }

       /**********************************************************************************
          PRINT ROUTINE FOR ELAPSED MICROSECONDS
       **********************************************************************************/
#ifdef DEBUG
      sum = sum + usec;
      idx_t++;
      if (idx_t > 40) {
        mean = sum / idx_t;
        if (mean / 29.00 / N_BLOCKS * SAMPLE_RATE / AUDIO_SAMPLE_RATE_EXACT < 100.0)
        {
          Serial.print("processor load:  ");
          Serial.print (mean / 29.00 / N_BLOCKS * SAMPLE_RATE / AUDIO_SAMPLE_RATE_EXACT);
          Serial.println("%");
        }
        else
        {
          Serial.println("100%");
        }
        Serial.print (mean);
        Serial.print (" microsec for ");
        Serial.print (N_BLOCKS);
        Serial.print ("  stereo blocks    ");
        //Serial.print("FFT-length = "); Serial.print(FFT_length);
        //Serial.print(";   FIR filter length = "); Serial.println(m_NumTaps);

        idx_t = 0;
        sum = 0;
      }
#endif
    } // end of audio process loop
    
      /**********************************************************************************
          Add button check etc. here
       **********************************************************************************/

} // end loop

void calc_FIR_coeffs (float * coeffs_I, int numCoeffs, float32_t fc, float32_t Astop, int type, float dfc, float Fsamprate)
// pointer to coefficients variable, no. of coefficients to calculate, frequency where it happens, stopband attenuation in dB,
// filter type, half-filter bandwidth (only for bandpass and notch)
{ // modified by WMXZ and DD4WH after
  // Wheatley, M. (2011): CuteSDR Technical Manual. www.metronix.com, pages 118 - 120, FIR with Kaiser-Bessel Window
  // assess required number of coefficients by
  //     numCoeffs = (Astop - 8.0) / (2.285 * TPI * normFtrans);
  // selecting high-pass, numCoeffs is forced to an even number for better frequency response

  float32_t Beta;
  float32_t izb;
  float fcf = fc;
  int nc = numCoeffs;
  fc = fc / Fsamprate;
  dfc = dfc / Fsamprate;
  // calculate Kaiser-Bessel window shape factor beta from stop-band attenuation
  if (Astop < 20.96)
    Beta = 0.0;
  else if (Astop >= 50.0)
    Beta = 0.1102 * (Astop - 8.71);
  else
    Beta = 0.5842 * powf((Astop - 20.96), 0.4) + 0.07886 * (Astop - 20.96);

  for (int i = 0; i < numCoeffs; i++) //zero pad entire coefficient buffer, important for variables from DMAMEM
  {
    coeffs_I[i] = 0.0;
  }

  izb = Izero (Beta);
  if (type == 0) // low pass filter
    //     {  fcf = fc;
  { fcf = fc * 2.0;
    nc =  numCoeffs;
  }
  else if (type == 1) // high-pass filter
  { fcf = -fc;
    nc =  2 * (numCoeffs / 2);
  }
  else if ((type == 2) || (type == 3)) // band-pass filter
  {
    fcf = dfc;
    nc =  2 * (numCoeffs / 2); // maybe not needed
  }
  else if (type == 4) // Hilbert transform
  {
    nc =  2 * (numCoeffs / 2);
    // clear coefficients
    for (int ii = 0; ii < 2 * (nc - 1); ii++) coeffs_I[ii] = 0;
    // set real delay
    coeffs_I[nc] = 1;

    // set imaginary Hilbert coefficients
    for (int ii = 1; ii < (nc + 1); ii += 2)
    {
      if (2 * ii == nc) continue;
      float x = (float)(2 * ii - nc) / (float)nc;
      float w = Izero(Beta * sqrtf(1.0f - x * x)) / izb; // Kaiser window
      coeffs_I[2 * ii + 1] = 1.0f / (PIH * (float)(ii - nc / 2)) * w ;
    }
    return;
  }

  for (int ii = - nc, jj = 0; ii < nc; ii += 2, jj++)
  {
    float x = (float)ii / (float)nc;
    float w = Izero(Beta * sqrtf(1.0f - x * x)) / izb; // Kaiser window
    coeffs_I[jj] = fcf * m_sinc(ii, fcf) * w;

  }

  if (type == 1)
  {
    coeffs_I[nc / 2] += 1;
  }
  else if (type == 2)
  {
    for (int jj = 0; jj < nc + 1; jj++) coeffs_I[jj] *= 2.0f * cosf(PIH * (2 * jj - nc) * fc);
  }
  else if (type == 3)
  {
    for (int jj = 0; jj < nc + 1; jj++) coeffs_I[jj] *= -2.0f * cosf(PIH * (2 * jj - nc) * fc);
    coeffs_I[nc / 2] += 1;
  }

} // END calc_FIR_coeffs

float32_t Izero (float32_t x)
{
  float32_t x2 = x / 2.0;
  float32_t summe = 1.0;
  float32_t ds = 1.0;
  float32_t di = 1.0;
  float32_t errorlimit = 1e-9;
  float32_t tmp;
  do
  {
    tmp = x2 / di;
    tmp *= tmp;
    ds *= tmp;
    summe += ds;
    di += 1.0;
  }   while (ds >= errorlimit * summe);
  return (summe);
}  // END Izero

float m_sinc(int m, float fc)
{ // fc is f_cut/(Fsamp/2)
  // m is between -M and M step 2
  //
  float x = m * PIH;
  if (m == 0)
    return 1.0f;
  else
    return sinf(x * fc) / (fc * x);
}
