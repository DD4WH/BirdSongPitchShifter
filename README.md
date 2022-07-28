# BirdSongPitchShifter
Experimental code for Teensy 4 to evaluate different pitch shift algorithms

Many people suffer from high frequency hearing loss, especially people above the age of 50. Lang Elliott had the idea of lowering the pitch of bird songs with a special device in order to help bird enthusiasts with high frequency hearing loss to hear those higher-pitched bird songs [https://hearbirdsagain.org/]. These higher-pitched songs would normally be outside of their hearing ranges. His idea motivated me to try this on the Teensy 4.0. The plan is to experiment with different pitch shifting algorithms. I want to evaluate their specific suitability for pitch shifting of bird songs. [Bird songs differ a lot from the normal audio application range, i.e. speech or music !]

My scan of the Teensy forum revealed several existing libraries/algorithms for altering the pitch of audio sounds:


* Granular effect - in the Teensy Audio library
* Heterodyning, i.e. multiplying the audio with a sine wave of given frequency - implementable with the Audio Library
* Vocoder / pitch shifting in the frequency domain - sophisticated algorithm by Stephan Bernsee, implemented by Duff2013 for the Teensy
* Formant shifting in the frequency domain - sophisticated algorithm by Chip Audette, implemented in the Tympan library and later by Bob Larkin in the OpenAudio_Arduino_lib
* Frequency shifting by Quadrature Modulation - implemented by Bob Larkin in the OpenAudio library, originally intended for Software Defined Radio 



So, after finding those implementations, I wanted to experiment with them in realtime by sending bird song audio from my smartphone to the Teensy and switch between those algorithms and tweak their parametres in order to find the best sounding solution for altering the pitch of bird songs by ear.

I use a Teensy 4.0 with an ADC PCM1808 and DAC5102a, but it should also run with a Teensy 4 and the Teensy audio board by using the Stereo line inputs.

The following sketch contains a real-time implementation of all the above mentioned methods, so I can switch and tweak them. Audio comes from the 3.5mm audio plug of my smartphone into the ADC and I can hear the result in my headphones. I implemented all methods in Stereo. Five buttons are used to toggle ON/OFF an input 2.5kHz highpass filter, to switch between the algorithms and to increase/decrease the amount of pitch shift of the specific method.

