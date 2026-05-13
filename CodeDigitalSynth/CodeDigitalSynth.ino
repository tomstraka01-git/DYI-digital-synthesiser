#include <I2S.h>
#include <SPI.h>
#include "MCP_ADC.h"
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>

#define LRCK_PIN  28   
#define BCK_PIN   27   
#define DIN_PIN   26   



#define SAMPLE_RATE 44100
#define AMPLITUDE   32767 // maximum, later i will apply volume multiplication


#define MCP_DOUT  16   
#define MCP_CS    17   
#define MCP_DIN   18   
#define MCP_CLK   19

// Buttons
#define BUTTON1 20 // Waweform changing button
#define BUTTON2 21 // Sound ON/OF
#define BUTTON3 22
#define BUTTON4 13


Adafruit_USBD_MIDI usb_midi;
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);


// Main mode variables
float volume = 1.0f;
float detune = 0.0f;
float portamento = 0.0f;


float baseFrequency = 440.0f;  
int midiNote = 69; 


// Portamento values
float currentFrequency = 440.0f; // A4 note
float targetFrequency  = 440.0f; 

// FX mode variables
float filterState = 0.0f;
float cutoff = 0.0f;
float resonance = 0.0f;
float delayTime = 0.0f;
float delayMix = 0.0f;

#define DELAY_MAX_SAMPLES (SAMPLE_RATE )  // 1 second
int16_t delayBuffer[DELAY_MAX_SAMPLES] = {0};
int delayWritePos = 0;


// LFO mode variables

float lfoRate = 1.0f;
float lfoDepth = 0.0f;
float lfoTarget = 0;
float lfoAttack = 0.5f;


/*
Modes
MAIN mode: Volume, Detune, Frequency, Portamento

ADSR mode: AttackRate, DecayRate, Sustain Level, ReleaseRate

FX mode: Cutoff, Resonance, DelayTime, DelayMix

LFO mode: LFORate, LFODepth,  LFOTarget, LFO Attack


*/


enum Mode { MAIN, ADSR, FX, LFO };
Mode currentMode = MAIN;


int waveform = 0; // 0 for sine, 1 for square, 2 for saw, 3 for triangle
I2S i2s(OUTPUT);

MCP3208 mcp(&SPI); 


enum ADSRState { ATTACK, DECAY, SUSTAIN, RELEASE, OFF };
ADSRState envState = OFF;


float attackRate = 0.015f; // max 0.0000075f 
float decayRate = 0.003f; // max 0.0000113f
float sustainLevel = 0.6f; // 0 to 1
float releaseRate = 0.003f; // max 0.0000045f

float envLevel = 0.0f;  // current envelope volume (0.0 – 1.0)


void noteOn()  { envState = ATTACK; }


void noteOff() { envState = RELEASE; }





// uses log to make the potenciometer feel nonlinear.
float analogReadLog(uint8_t ch, float minT, float maxT) {
  float pot = mcp.read(ch) / 4095.0f;
  return minT * powf(maxT / minT, pot);
}


void updateButtons() {
  static bool last1 = HIGH, last2 = HIGH, last3 = HIGH, last4 = HIGH;
  static unsigned long lastTime1 = 0, lastTime2 = 0, lastTime3 = 0, lastTime4 = 0;

  bool b1 = digitalRead(BUTTON1);
  bool b2 = digitalRead(BUTTON2);
  bool b3 = digitalRead(BUTTON3);
  bool b4 = digitalRead(BUTTON4);

  // wavedform button
  if (last1 == HIGH && b1 == LOW && millis() - lastTime1 > 50) {
    waveform = (waveform + 1) % 4;
    lastTime1 = millis();
  }

  // on/off button
  if (last2 == HIGH && b2 == LOW) noteOn();
  if (last2 == LOW  && b2 == HIGH) noteOff();

  // cycle mode button
  if (last3 == HIGH && b3 == LOW && millis() - lastTime3 > 50) {
    currentMode = (Mode)((currentMode + 1) % 4);
    lastTime3 = millis();
  }

  // free for now
  if (last4 == HIGH && b4 == LOW && millis() - lastTime4 > 50) {
    lastTime4 = millis();
  }

  last1 = b1; last2 = b2; last3 = b3; last4 = b4;
}


void updateKnobs() {
  switch (currentMode) {
    case MAIN:
      
      volume = mcp.read(0) / 4095.0f;
      float detuneAmount = (mcp.read(1) / 2047.5f) - 1.0f;
      baseFrequency = analogReadLog(2, 20.0f, 2000.0f);
      targetFrequency = baseFrequency * powf(2.0f, (midiNote - 69.0f) / 12.0f);
      detune = targetFrequency * (powf(2.0f, detuneAmount * 50.0f / 1200.0f) - 1.0f);
      portamento = mcp.read(3) / 4095.0f;
      break;

    case ADSR:

      attackRate = 1.0f / (analogReadLog(0, 0.001f, 3.0f) * SAMPLE_RATE);
      decayRate = 1.0f / (analogReadLog(1, 0.005f, 2.0f) * SAMPLE_RATE);
      sustainLevel = mcp.read(2) / 4095.0f;
      releaseRate = 1.0f / (analogReadLog(3, 0.005f, 5.0f) * SAMPLE_RATE);
      break;

    case FX:

      cutoff = analogReadLog(0, 200.0f, 8000.0f);
      resonance = mcp.read(1) / 4095.0f;
      delayTime = mcp.read(2) / 4095.0f;
      delayMix = mcp.read(3) / 4095.0f;
      break;

    case LFO:
      
      lfoRate = analogReadLog(0, 0.1f, 20.0f);
      lfoDepth = mcp.read(1) / 4095.0f;
      lfoTarget = map(mcp.read(2), 0, 4095, 0, 2);
      lfoAttack = analogReadLog(3, 0.05f, 5.0f);
      break;
  
  }
}


void readMidi() {
  if (MIDI.read()) {
    if (MIDI.getType() == midi::NoteOn && MIDI.getData2() > 0) {
      midiNote = MIDI.getData1();  
      targetFrequency = baseFrequency * powf(2.0f, (midiNote - 69.0f) / 12.0f);
      noteOn();
    }
    if (MIDI.getType() == midi::NoteOff || 
      (MIDI.getType() == midi::NoteOn && MIDI.getData2() == 0)) {
      noteOff();
    }
  }

}
 

void setup() {
  
  USBDevice.begin();

  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);
  pinMode(BUTTON4, INPUT_PULLUP);

  SPI.setRX(MCP_DOUT);
  SPI.setTX(MCP_DIN);
  SPI.setSCK(MCP_CLK);
  SPI.begin();
  mcp.begin(MCP_CS);
  
  i2s.setBCLK(BCK_PIN);   
  i2s.setDATA(DIN_PIN);  
  i2s.setLRCLK(LRCK_PIN);
  i2s.setBitsPerSample(16);
  i2s.begin(SAMPLE_RATE);

  usb_midi.begin();
  MIDI.begin(MIDI_CHANNEL_OMNI);
}

void loop() {
  static float phase = 0.0f;
  
  float portaCoeff = 1.0f - portamento * 0.9997f;
  currentFrequency += (targetFrequency - currentFrequency) * portaCoeff;
  
  
  const float inc = 2.0f * M_PI * (currentFrequency + detune) / SAMPLE_RATE;

  int16_t sample = 0;
  
  static unsigned long lastControlUpdate = 0;
  
  readMidi();
  
  if (millis() - lastControlUpdate > 5) { // every 5ms
    updateButtons();
    updateKnobs();
    lastControlUpdate = millis();
  }


  switch (envState) {
    case ATTACK:
      envLevel += attackRate;
      if (envLevel >= 1.0f) { envLevel = 1.0f; envState = DECAY; }
      break;
    case DECAY:
      envLevel -= decayRate;
      if (envLevel <= sustainLevel) { envLevel = sustainLevel; envState = SUSTAIN; }
      break;
    case SUSTAIN:
      envLevel = sustainLevel; 
      break;
    case RELEASE:
      envLevel -= releaseRate;
      if (envLevel <= 0.0f) { envLevel = 0.0f; envState = OFF; }
      break;
    case OFF:
      envLevel = 0.0f;
      break;
  }

  switch (waveform) {
    case 0:
      sample = (int16_t)(sinf(phase) * AMPLITUDE);
      break;
    case 1:
      sample = (int16_t)((phase < M_PI ? 1.0f : -1.0f) * AMPLITUDE);
      break;
    case 2:
      sample = (int16_t)((1.0f - phase / M_PI) * AMPLITUDE);
      break;
    case 3:
      sample = (int16_t)((phase < M_PI ? (phase / M_PI * 2.0f - 1.0f) : (3.0f - phase / M_PI * 2.0f)) * AMPLITUDE);
      break;
    default: 
      sample = 0; 
      break;
  } 
  
  int32_t s = (int32_t)sample * envLevel * volume;
  if (s >  32767) s =  32767;
  if (s < -32768) s = -32768;
  sample = (int16_t)s;

  
  float alpha = (2.0f * M_PI * cutoff) /
              (2.0f * M_PI * cutoff + SAMPLE_RATE);

  // low-pass filter
  filterState += alpha * ((float)sample - filterState);

  sample = (int16_t)filterState;


  // Compute delay read position
  int delaySamples = (int)(delayTime * (DELAY_MAX_SAMPLES - 1));
  int delayReadPos = delayWritePos - delaySamples;
  if (delayReadPos < 0) delayReadPos += DELAY_MAX_SAMPLES;

  // Read from buffer, write dry signal in
  int16_t delaySig = delayBuffer[delayReadPos];
  delayBuffer[delayWritePos] = sample + (int16_t)(delaySig * 0.5f); // 0.5f = feedback amount
  delayWritePos = (delayWritePos + 1) % DELAY_MAX_SAMPLES;

  
  int32_t mixed = (int32_t)(sample * (1.0f - delayMix)) + (int32_t)(delaySig * delayMix);
  mixed = constrain(mixed, -32768, 32767);

  i2s.write16((int16_t)mixed, (int16_t)mixed);

  phase += inc;
  if (phase >= 2.0f * M_PI) phase -= 2.0f * M_PI;
}



