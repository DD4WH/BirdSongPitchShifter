/*  
 *  PITCH SHIFTING, FORMANT SHIFTING AND FREQUENCY SHIFTING OF BIRD SOUNDS
 *  (c) Frank DD4WH
 *  
 *  Experiments with the Teensy 4.0 & ADC PCM1808 & DAC PCM5102
 *    
 *  to aid people with hearing loss to hear birds (again)
 *  
 *  inspired & motivated by Lang Elliott
 *  https://hearbirdsagain.org/  
 *  
 *  using the following code libraries:
 *  
 *  Teensy Audio Library: Paul Stoffregen
 *  
 *  Pitch Shifting using FFT: Stephan Bernsee & duff2013
 * 
 *  OpenAudioLibrary: Chip Audette & Bob Larkin 
 * 
 *  experimental platform
 *  all licenses remain those of the original code snippets that I used,
 *  mostly MIT license, but also some use GNU GPLv3 license
 *  
 *  July, 27th 2022
 */
 
/*
 *  Pitch Shifting:
 *  1.) advanced algorithm (smbPitchShift) modified by duff2013
 *  2.) frequency division (BatEffectGranular) modified by CorBee (from the TeensyBat project) --> I found this to distort the audio very much, so this was not implemented into this code
 * 
 *  Heterodyning:
 *  3.) simple mixing of Audio signal with sine oscillator
 *  
 *  Granular Effect:
 *  4.) from Audio Library
 *  
 *  Frequency Shifting: 
 *  4.) Quadrature Modulator: designed for Software Defined Radios by Bob Larkin
 *  
 *  Formant shifting:
 *  5.) Tympan library by Chip Audette, version taken from OpenAudio lib
*/

/*
 * Only for Teensy 4.0 or Teensy 4.1 !!! This does not work with Teensy 3.2/3.5/3.6
 * Use the latest Arduino 1.8.19 and Teensyduino version 1.57
 * 
 * In order for this code to compile correctly, please install these libraries into your Arduino folder: 
 * 
 * 1.) https://github.com/duff2013/AudioEffectPhaseVocoder
 *     you have to correct line 26 of this file: effect_phaseVocoder.cpp --> should be: "#include "effect_phaseVocoder.h"" 
 * 2.) https://github.com/chipaudette/OpenAudio_ArduinoLibrary
 *     this is the source for Formant Shift and Quadrature modulation-based frequency shift
 * 3.) copy the file "hilbert251A.h" from folder OpenAudio_ArduinoLibrary/examples/FineFreqShift_OA to your sketch folder   
 * 
 */

/*  
Audio MEM INT16   Cur/Peak: 20/24
Audio MEM Float32 Cur/Peak: 24/26
CPU INT16 Cur/Peak: 60.39%/62.68%   
CPU F32   Cur/Peak: 15.60%/16.17% 
 *  
Memory Usage on Teensy 4.0:
   FLASH: code:98420, data:81032, headers:8960   free for files:1843204
   RAM1: variables:227104, code:95816, padding:2488   free for local variables:198880
   RAM2: variables:23808  free for malloc/new:500480

 */

#include "AudioStream_F32.h"
#include "OpenAudio_ArduinoLibrary.h"
#include "AudioEffectFormantShiftFD_OA_F32.h"  //the local file holding your custom function
#include <Audio.h>
#include <Bounce.h>
#include "effect_phaseVocoder.h"
//#include <Wire.h>
//#include <SPI.h>
//#include <SD.h>
//#include <SerialFlash.h>
// Filter for AudioFilter90Deg_F32 hilbert1
#include "hilbert251A.h"

// this is only needed for F32 OpenAudio lib !
//set the sample rate and block size
const float sample_rate_Hz = 44100.f; ; // other frequencies in the table in AudioOutputI2S_F32 for T3.x only
const int audio_block_samples = 128;     //for freq domain processing, a power of 2: 16, 32, 64, 128
AudioSettings_F32 audio_settings(sample_rate_Hz, audio_block_samples);

// GUItool: begin automatically generated code
AudioInputI2S            i2s1;           //xy=153,323
AudioAmplifier           ampL;           //xy=327,398
AudioAmplifier           ampR;           //xy=335,254
AudioFilterBiquad        highpassL;        //xy=479,350
AudioFilterBiquad        highpassR;        //xy=480,296
AudioMixer4              HPswitchR;         //xy=639,241
AudioMixer4              HPswitchL;         //xy=639,421
AudioSynthWaveformSine   sine1;          //xy=645,325
AudioEffectPhaseVocoder  VocoderR;      //xy=855,117
AudioEffectPhaseVocoder  VocoderL;      //xy=858,174
AudioEffectMultiply      multiplyR;      //xy=859,287
AudioEffectMultiply      multiplyL;      //xy=859,363
AudioEffectGranular      granularR;      //xy=504,155
AudioEffectGranular      granularL;      //xy=504,155
AudioMixer4              switchR;         //xy=1042,202
AudioMixer4              switchL;         //xy=1046,362
AudioFilterBiquad        lowpassR;        //xy=1195,230
AudioFilterBiquad        lowpassL;        //xy=1199,331
AudioOutputI2S           i2s2;           //xy=1338,285

// Formant Shifter
AudioConvert_I16toF32    convertInR;  // Convert to float
AudioConvert_I16toF32    convertInL;  // Convert to float
AudioEffectFormantShiftFD_OA_F32 formantShiftR(audio_settings); // the frequency-domain processing block
AudioEffectFormantShiftFD_OA_F32 formantShiftL(audio_settings); // the frequency-domain processing block
AudioEffectGain_F32      formantGainR; //Applies digital gain to audio data.
AudioEffectGain_F32      formantGainL; //Applies digital gain to audio data.
AudioConvert_F32toI16    convertOutR;
AudioConvert_F32toI16    convertOutL;

AudioMixer4_F32          MixerR;  
AudioMixer4_F32          MixerL;

AudioConnection          patchCord31(HPswitchR, 0, convertInR, 0);
AudioConnection          patchCord32(HPswitchL, 0, convertInL, 0);
AudioConnection_F32      patchCord33(convertInR, 0, formantShiftR, 0);
AudioConnection_F32      patchCord34(convertInL, 0, formantShiftL, 0);
AudioConnection_F32      patchCord35(formantShiftR, 0, MixerR, 2);
AudioConnection_F32      patchCord36(formantShiftL, 0, MixerL, 2);

AudioConnection_F32      patchCord39(MixerR, 0, convertOutR, 0);
AudioConnection_F32      patchCord40(MixerL, 0, convertOutL, 0);
AudioConnection          patchCord37(convertOutR, 0, switchR, 2);
AudioConnection          patchCord38(convertOutL, 0, switchL, 2);


// Frequency Mixer
RadioIQMixer_F32         IQMixerR;
RadioIQMixer_F32         IQMixerL;
AudioFilter90Deg_F32     HilbertR;
AudioFilter90Deg_F32     HilbertL;
AudioConnection_F32      patchCord51(convertInR, 0, IQMixerR, 0);
AudioConnection_F32      patchCord52(convertInR, 0, IQMixerR, 1);
AudioConnection_F32      patchCord53(convertInL, 0, IQMixerL, 0);
AudioConnection_F32      patchCord54(convertInL, 0, IQMixerL, 1);
AudioConnection_F32      patchCord55(IQMixerR, 0, HilbertR, 0);
AudioConnection_F32      patchCord56(IQMixerR, 1, HilbertR, 1);
AudioConnection_F32      patchCord57(IQMixerL, 0, HilbertL, 0);
AudioConnection_F32      patchCord58(IQMixerL, 1, HilbertL, 1);
AudioConnection_F32      patchCord59(HilbertR, 0, MixerR, 0);
AudioConnection_F32      patchCord60(HilbertR, 1, MixerR, 1);
AudioConnection_F32      patchCord61(HilbertL, 0, MixerL, 0);
AudioConnection_F32      patchCord62(HilbertL, 1, MixerL, 1);


AudioConnection          patchCord1(i2s1, 0, ampR, 0);
AudioConnection          patchCord2(i2s1, 1, ampL, 0);
AudioConnection          patchCord3(ampL, highpassL);
AudioConnection          patchCord4(ampL, 0, HPswitchL, 0);
AudioConnection          patchCord5(ampR, highpassR);
AudioConnection          patchCord6(ampR, 0, HPswitchR, 0);
AudioConnection          patchCord7(highpassL, 0, HPswitchL, 1);
AudioConnection          patchCord8(highpassR, 0, HPswitchR, 1);
AudioConnection          patchCord9(HPswitchR, VocoderR);
AudioConnection          patchCord10(HPswitchR, 0, multiplyR, 0);
AudioConnection          patchCord11(HPswitchL, VocoderL);
AudioConnection          patchCord12(HPswitchL, 0, multiplyL, 1);
AudioConnection          patchCord13(sine1, 0, multiplyR, 1);
AudioConnection          patchCord14(sine1, 0, multiplyL, 0);
AudioConnection          patchCord15(VocoderR, 0, switchR, 0);
AudioConnection          patchCord16(VocoderL, 0, switchL, 0);
AudioConnection          patchCord17(multiplyR, 0, switchR, 1);
AudioConnection          patchCord18(multiplyL, 0, switchL, 1);
AudioConnection          patchCord19(switchR, lowpassR);
AudioConnection          patchCord20(switchL, lowpassL);

AudioConnection          patchCord70(HPswitchR, granularR);
AudioConnection          patchCord71(HPswitchL, granularL);
AudioConnection          patchCord72(granularR, 0, switchR, 3);
AudioConnection          patchCord73(granularL, 0, switchL, 3);

AudioConnection          patchCord21(lowpassR, 0, i2s2, 0);
AudioConnection          patchCord22(lowpassL, 0, i2s2, 1);
AudioControlSGTL5000     sgtl5000_1;     //xy=154,532
// GUItool: end automatically generated code


elapsedMillis count;

int shift = -12; // 12 semitones == one octave downshift
int heterodyne_freq = 3600;
float formant_scaling = 0.5f; // 1.0f == no shift
int formant_overlap_factor = 4;
float quadrature_freq = 2500.0f;
float delayms = 1.0f;
int highPass_freq = 2500;
int lowPass_freq = 3500;
uint8_t HPfilter_ON = 0; // 0 = OFF, 1 = ON
#define GRANULAR_MEMORY_SIZE 12800  // enough for 290 ms at 44.1 kHz
int16_t granularMemoryR[GRANULAR_MEMORY_SIZE];
int16_t granularMemoryL[GRANULAR_MEMORY_SIZE];
float granular_ratio = 0.5f;
float granular_ms = 30.0f;

#define MODE_MIN          0
#define MODE_HETERODYNE   0
#define MODE_VOCODER      1
#define MODE_FORMANT      2
#define MODE_QUADRATURE   3
#define MODE_GRANULAR     4
#define MODE_MAX          4

int mode = MODE_HETERODYNE;

#define HETERODYNE_STANDARD_FREQ  2500

Bounce button0 = Bounce(26, 20); // UP              Decrease the pitch of the sound
Bounce button1 = Bounce(24, 20); // DOWN            Increase the pitch of the sound
Bounce button2 = Bounce(28, 20); // Klick middle    Restore shift parameter to STANDARD value (= zero shift)  
Bounce button3 = Bounce(32, 20); // RIGHT           Toggle different "modes"= shifting methods
Bounce button4 = Bounce(30, 20); // LEFT            Toggle HighPass Filter ON/OFF

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  AudioMemory(30);
  AudioMemory_F32(200, audio_settings);
  pinMode(26, INPUT_PULLUP);
  pinMode(24, INPUT_PULLUP);
  pinMode(28, INPUT_PULLUP);
  pinMode(32, INPUT_PULLUP);
  pinMode(30, INPUT_PULLUP);

  sgtl5000_1.enable();
  sgtl5000_1.volume(0.9);

  ampR.gain(2.5f);
  ampL.gain(2.5f);

    // Configure the frequency-domain algorithm for Formant Shift
  //int overlap_factor = 4;  //set to 4 or 8 or either 75% overlap (4x) or 87.5% overlap (8x)
  int N_FFT = audio_block_samples * formant_overlap_factor;
  formantShiftR.setup(audio_settings, N_FFT); //do after AudioMemory_F32();
  formantShiftL.setup(audio_settings, N_FFT); //do after AudioMemory_F32();

  Set_Formant(formant_scaling);

  Set_Vocoder(shift);

  Set_Heterodyne(heterodyne_freq, lowPass_freq, highPass_freq);

  Set_Quadrature(quadrature_freq);
  HilbertR.begin(hilbert251A, 251);  // Set the Hilbert transform FIR filter
  HilbertL.begin(hilbert251A, 251);  // Set the Hilbert transform FIR filter

  Set_HighPass (HPfilter_ON, highPass_freq);
  
  Switch_Mode(mode);

  // the Granular effect requires memory to operate
  granularR.begin(granularMemoryR, GRANULAR_MEMORY_SIZE);
  granularL.begin(granularMemoryL, GRANULAR_MEMORY_SIZE);

  Set_Granular(granular_ratio, granular_ms);
}

void loop() {
  // put your main code here, to run repeatedly:
  button0.update();
  button1.update();
  button2.update();
  button3.update();
  button4.update();
  
  if(count > 2000)
  {
    print_Info();
    count = 0;
  }

    if(mode == MODE_VOCODER)
    {
        if (button0.fallingEdge()) 
        {
          shift += 2;
          Set_Vocoder(shift);
        }
        if (button1.fallingEdge()) 
        {
          shift -= 2;
          Set_Vocoder(shift);
        }
        if (button2.fallingEdge()) 
        {
          shift = 0;
          Set_Vocoder(shift);
        }
    }
    else if(mode == MODE_HETERODYNE)
    {
        if (button0.fallingEdge()) 
        {
          heterodyne_freq -= 200;
          Set_Heterodyne(heterodyne_freq, lowPass_freq, highPass_freq);
        }
        if (button1.fallingEdge()) 
        {
          heterodyne_freq += 200;
          Set_Heterodyne(heterodyne_freq, lowPass_freq, highPass_freq);
        }
        if (button2.fallingEdge()) 
        {
          heterodyne_freq = HETERODYNE_STANDARD_FREQ;
          Set_Heterodyne(heterodyne_freq, lowPass_freq, highPass_freq);
        }
    }
    else if(mode == MODE_FORMANT)
    {
        if (button0.fallingEdge()) 
        {
          formant_scaling -= 0.05f;
          Set_Formant(formant_scaling);
        }
        if (button1.fallingEdge()) 
        {
          formant_scaling += 0.05f;
          Set_Formant(formant_scaling);
        }
        if (button2.fallingEdge()) 
        {
          formant_scaling = 1.0f;
          Set_Formant(formant_scaling);
        }
    }
    else if(mode == MODE_QUADRATURE)
    {
        if (button0.fallingEdge()) 
        {
          quadrature_freq -= 200.0f;
          Set_Quadrature(quadrature_freq);
        }
        if (button1.fallingEdge()) 
        {
          quadrature_freq += 200.0f;
          Set_Quadrature(quadrature_freq);
        }
        if (button2.fallingEdge()) 
        {
          quadrature_freq = 2500.0f;
          Set_Quadrature(quadrature_freq);
        }
    }
    else if(mode == MODE_GRANULAR)
    {
        if (button0.fallingEdge()) 
        {
          granular_ratio -= 0.05f;
          Set_Granular(granular_ratio, granular_ms);
        }
        if (button1.fallingEdge()) 
        {
          granular_ratio += 0.05f;
          Set_Granular(granular_ratio, granular_ms);
        }
        if (button2.fallingEdge()) 
        {
          granular_ratio = 0.5f;
          Set_Granular(granular_ratio, granular_ms);
        }
    }    
    // this is for mode changing    
        if (button3.fallingEdge()) 
        {
          mode += 1;
          mode = clamp_turnover(mode,MODE_MIN,MODE_MAX);
          Switch_Mode(mode);
        }
        if (button4.fallingEdge()) 
        {
          if(HPfilter_ON == 1)
          {
            HPfilter_ON = 0;
          }
          else
          {
            HPfilter_ON = 1;
          }
          Set_HighPass(HPfilter_ON, highPass_freq);
        }
}

void Switch_Mode(int mode)
{
  if(mode == MODE_VOCODER)
  {
    Serial.println("MODE Vocoder was selected");
    ampR.gain(3.5f);
    ampL.gain(3.5f);
    // switch on Vocoder, switch off everything else
    switchR.gain(0,1.0f);
    switchR.gain(1,0.0f);
    switchR.gain(2,0.0f);
    switchR.gain(3,0.0f);
    switchL.gain(0,1.0f);
    switchL.gain(1,0.0f);
    switchL.gain(2,0.0f);
    switchL.gain(3,0.0f);
    // lowpass 
    lowpassR.setLowpass(0,lowPass_freq,0.54);
    lowpassR.setLowpass(1,lowPass_freq,1.3);
    lowpassR.setLowpass(2,lowPass_freq,0.54);
    lowpassR.setLowpass(3,lowPass_freq,1.3);
    lowpassL.setLowpass(0,lowPass_freq,0.54);
    lowpassL.setLowpass(1,lowPass_freq,1.3);
    lowpassL.setLowpass(2,lowPass_freq,0.54);
    lowpassL.setLowpass(3,lowPass_freq,1.3);
        
  }
  else if(mode == MODE_HETERODYNE)
  {
    sine1.amplitude(1.0f);
    sine1.frequency(heterodyne_freq);
    // switch on Heterodyne, switch off everything else
    switchR.gain(0,0.0f);
    switchR.gain(1,1.3f);
    switchR.gain(2,0.0f);
    switchR.gain(3,0.0f);
    switchL.gain(0,0.0f);
    switchL.gain(1,1.3f);
    switchL.gain(2,0.0f);
    switchL.gain(3,0.0f);
    // lowpass
    lowpassR.setLowpass(0,lowPass_freq,0.54);
    lowpassR.setLowpass(1,lowPass_freq,1.3);
    lowpassR.setLowpass(2,lowPass_freq,0.54);
    lowpassR.setLowpass(3,lowPass_freq,1.3);
    lowpassL.setLowpass(0,lowPass_freq,0.54);
    lowpassL.setLowpass(1,lowPass_freq,1.3);
    lowpassL.setLowpass(2,lowPass_freq,0.54);
    lowpassL.setLowpass(3,lowPass_freq,1.3);
    Serial.println("MODE Heterodyne was selected");
  }
  else if(mode == MODE_FORMANT)
  {
    // switch on Formant, switch off everything else
    switchR.gain(0,0.0f);
    switchR.gain(1,0.0f);
    switchR.gain(2,1.0f);
    switchR.gain(3,0.0f);
    switchL.gain(0,0.0f);
    switchL.gain(1,0.0f);
    switchL.gain(2,1.0f);
    switchL.gain(3,0.0f);
    // lowpass
    lowpassR.setLowpass(0,lowPass_freq,0.54);
    lowpassR.setLowpass(1,lowPass_freq,1.3);
    lowpassR.setLowpass(2,lowPass_freq,0.54);
    lowpassR.setLowpass(3,lowPass_freq,1.3);
    lowpassL.setLowpass(0,lowPass_freq,0.54);
    lowpassL.setLowpass(1,lowPass_freq,1.3);
    lowpassL.setLowpass(2,lowPass_freq,0.54);
    lowpassL.setLowpass(3,lowPass_freq,1.3);
    MixerR.gain(0,0.0f);
    MixerR.gain(1,0.0f);
    MixerR.gain(2,1.0f);
    MixerR.gain(3,0.0f);
    MixerL.gain(0,0.0f);
    MixerL.gain(1,0.0f);
    MixerL.gain(2,1.0f);
    MixerL.gain(3,0.0f);
    Serial.println("MODE Formant Shift was selected");  
  }
  else if(mode == MODE_QUADRATURE)
  {
    // switch on Formant or Freq shifting, switch off everything else
    switchR.gain(0,0.0f);
    switchR.gain(1,0.0f);
    switchR.gain(2,1.0f);
    switchR.gain(3,0.0f);
    switchL.gain(0,0.0f);
    switchL.gain(1,0.0f);
    switchL.gain(2,1.0f);
    switchL.gain(3,0.0f);
    // lowpass
    lowpassR.setLowpass(0,lowPass_freq,0.54);
    lowpassR.setLowpass(1,lowPass_freq,1.3);
    lowpassR.setLowpass(2,lowPass_freq,0.54);
    lowpassR.setLowpass(3,lowPass_freq,1.3);
    lowpassL.setLowpass(0,lowPass_freq,0.54);
    lowpassL.setLowpass(1,lowPass_freq,1.3);
    lowpassL.setLowpass(2,lowPass_freq,0.54);
    lowpassL.setLowpass(3,lowPass_freq,1.3);
    MixerR.gain(0,0.5f);
    MixerR.gain(1,0.5f);
    MixerR.gain(2,0.0f);
    MixerR.gain(3,0.0f);
    MixerL.gain(0,0.5f);
    MixerL.gain(1,0.5f);
    MixerL.gain(2,0.0f);
    MixerL.gain(3,0.0f);
    Serial.println("MODE Quadrature was selected");  
  }
  else if(mode == MODE_GRANULAR)
  {
    // switch on Formant or Freq shifting, switch off everything else
    switchR.gain(0,0.0f);
    switchR.gain(1,0.0f);
    switchR.gain(2,0.0f);
    switchR.gain(3,1.0f);
    switchL.gain(0,0.0f);
    switchL.gain(1,0.0f);
    switchL.gain(2,0.0f);
    switchL.gain(3,1.0f);
    // lowpass
    lowpassR.setLowpass(0,lowPass_freq,0.54);
    lowpassR.setLowpass(1,lowPass_freq,1.3);
    lowpassR.setLowpass(2,lowPass_freq,0.54);
    lowpassR.setLowpass(3,lowPass_freq,1.3);
    lowpassL.setLowpass(0,lowPass_freq,0.54);
    lowpassL.setLowpass(1,lowPass_freq,1.3);
    lowpassL.setLowpass(2,lowPass_freq,0.54);
    lowpassL.setLowpass(3,lowPass_freq,1.3);
    Serial.println("MODE Granular was selected");  
  }
  else
  {
    Serial.println("ERROR: no sensible mode selected !");
  }
} // END "Switch_Mode"

void Set_Vocoder(int shift)
{
  VocoderR.setPitchShift(shift);
  VocoderL.setPitchShift(shift);  
  Serial.print(" PITCH SHIFT IN SEMITONES: "); Serial.println(shift);
  Serial.println();
}

void Set_Heterodyne(int freq, int lowpass_freq, int highpass_freq)
{
  heterodyne_freq = clamp(freq,0,10000);
  sine1.frequency(heterodyne_freq);
  Serial.print(" HETERODYNE SHIFT IN HZ: "); Serial.println(heterodyne_freq);
  Serial.println();
}

void Set_Formant(float scale_factor)
{
//  formantShiftR.setScaleFactor(1.587401f); // up-shift !  1.0 is no formant shifting.
//  formantShiftL.setScaleFactor(1.587401f); // up-shift !  1.0 is no formant shifting.
  formant_scaling = clamp_f(scale_factor,0.1f,2.0f);
  formantShiftR.setScaleFactor(formant_scaling); //1.0 is no formant shifting.
  formantShiftL.setScaleFactor(formant_scaling); //1.0 is no formant shifting.
}

void Set_Quadrature(float frequencyLO)
{
  IQMixerR.frequency(frequencyLO);   // Frequency shift, Hz
  IQMixerL.frequency(frequencyLO);   // Frequency shift, Hz
}

void Set_Granular(float ratio, float ms)
{
    granular_ratio = clamp_f(ratio,0.2f, 3.0f);
    granular_ms = clamp_f(ms, 25, 2500);
    granularR.beginPitchShift(granular_ms); // ??? maybe 25 to 2500 ?
    granularR.setSpeed(ratio); // 0.5 to 2.0
    granularL.beginPitchShift(granular_ms); // ??? maybe 25 to 2500 ?
    granularL.setSpeed(ratio); // 0.5 to 2.0
    Serial.print(" GRANULAR RATIO is: "); Serial.print(granular_ratio);
    Serial.println();
}

void Set_HighPass (uint8_t on, int freq)
{
    if(on == 1)
    {
      HPswitchR.gain(0,0.0f);  
      HPswitchR.gain(1,1.0f);
      HPswitchL.gain(0,0.0f);
      HPswitchL.gain(1,1.0f);
    }
    else
    {
      HPswitchR.gain(0,1.0f);  
      HPswitchR.gain(1,0.0f);
      HPswitchL.gain(0,1.0f);
      HPswitchL.gain(1,0.0f);
    }

    HPswitchL.gain(2,0.0f);
    HPswitchL.gain(3,0.0f);
    HPswitchR.gain(2,0.0f);
    HPswitchR.gain(3,0.0f);

    highpassR.setHighpass(0, freq, 0.54);
    highpassR.setHighpass(1, freq, 1.3);
    highpassR.setHighpass(2, freq, 0.54);
    highpassR.setHighpass(3, freq, 1.3);
    highpassL.setHighpass(0, freq, 0.54);
    highpassL.setHighpass(1, freq, 1.3);
    highpassL.setHighpass(2, freq, 0.54);
    highpassL.setHighpass(3, freq, 1.3);  
}

int clamp_turnover(int var, int Min, int Max)
{
  if(var > Max) var = Min;
  if(var < Min) var = Max;
  return var;
}

int clamp(int var, int Min, int Max)
{
  if(var > Max) var = Max;
  if(var < Min) var = Min;
  return var;
}

float clamp_f(float var, float Min, float Max)
{
  if(var > Max) var = Max;
  if(var < Min) var = Min;
  return var;
}

void print_Info(void)
{
    Serial.println();
    Serial.print("We are in ");
    //Serial.println(mode);
    if(mode == MODE_VOCODER) 
    {
      Serial.println(" VOCODOER mode");
      Serial.print(" PITCH SHIFT IN SEMITONES: "); Serial.println(shift);
   }
    else if (mode == MODE_HETERODYNE) 
    {
      Serial.println(" HETERODYNE mode");
      Serial.print(" HETERODYNE FREQUENCY is: "); Serial.print(heterodyne_freq); Serial.println(" Hz"); 
    }
    else if(mode == MODE_QUADRATURE)
    {
      Serial.println("QUADRATURE/FREQUENCY SHIFT mode");
      Serial.print(" FREQUENCY SHIFT is: "); Serial.print(quadrature_freq); Serial.println(" Hz"); 
    }
    else if(mode == MODE_FORMANT)
    {
      Serial.println("FORMANT SHIFT mode");
      Serial.print(" Formant scaling factor is: "); Serial.println(formant_scaling); 
    }    
    else if(mode == MODE_GRANULAR)
    {
      Serial.println("GRANULAR mode");
      Serial.print(" Granular ratio is: "); Serial.println(granular_ratio); 
    }    
    else
    {
      Serial.println(" no sensible mode, something wrong here . . .");
    }
    Serial.print("Audio Input HighPass Filter is ");
    if(HPfilter_ON == 1) Serial.println("ON");
    else Serial.println("OFF"); 
    Serial.print("Audio MEM INT16   Cur/Peak: ");
    Serial.print(AudioMemoryUsage());
    Serial.print("/");
    Serial.println(AudioMemoryUsageMax());
    Serial.print("Audio MEM Float32 Cur/Peak: ");
    Serial.print(AudioMemoryUsage_F32());
    Serial.print("/");
    Serial.println(AudioMemoryUsageMax_F32());    
    
    Serial.print("CPU INT16 Cur/Peak: "); 
    Serial.print (AudioProcessorUsage());
    Serial.print("%/"); 
    Serial.print (AudioProcessorUsageMax());
    Serial.println("%,   ");
    Serial.print("CPU F32   Cur/Peak: ");
    Serial.print(audio_settings.processorUsage());
    Serial.print("%/");
    Serial.print(audio_settings.processorUsageMax());
    Serial.print("%,   ");
    Serial.println();

}
