#include <I2S.h>
#include <SPI.h>
#include "MCP_ADC.h"
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>
#include "pico/multicore.h"

#define LRCK_PIN  28   
#define BCK_PIN   27   
#define DIN_PIN   26   

#define SAMPLE_RATE 44100
#define AMPLITUDE   32767

#define MCP_DOUT  16   
#define MCP_CS    17   
#define MCP_DIN   18   
#define MCP_CLK   19

#define BUTTON1 20 // OSC1 waveform cycle
#define BUTTON2 21 // Sound ON/OFF
#define BUTTON3 22 // Mode cycle
#define BUTTON4 13 // OSC2 waveform cycle  ← now used

Adafruit_USBD_MIDI usb_midi;
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

// Main mode variables
float volume = 1.0f;
float detune = 0.0f;
float portamento = 0.0f;
float osc2Level = 0.5f;  // 0 = OSC1 only, 1 = OSC2 only, 0.5 = equal
float osc2SemiOffset = 0.0f;  // -24 to +24 semitones

float baseFrequency = 440.0f;  
int   midiNote = 69; 

float currentFrequency = 440.0f;
float targetFrequency = 440.0f; 

// FX mode variables
float cutoff = 1000.0f;
float resonance = 0.0f;
float delayTime = 0.0f;
float delayMix  = 0.0f;

#define DELAY_MAX_SAMPLES (SAMPLE_RATE)
int16_t delayBuffer[DELAY_MAX_SAMPLES] = {0};
int delayWritePos = 0;

// LFO mode variables
float lfoEnvelope = 0.0f;
float lfoRate = 1.0f;
float lfoDepth = 0.0f;
float lfoTarget = 0;
float lfoAttack = 0.5f;

enum Mode { MAIN, ADSR, FX, LFO };
Mode currentMode = MAIN;

int waveform1 = 0;
int waveform2 = 1; // default OSC2 to square for a different texture

I2S i2s(OUTPUT);
MCP3208 mcp(&SPI); 

enum ADSRState { ATTACK, DECAY, SUSTAIN, RELEASE, OFF };
ADSRState envState = OFF;

float attackRate = 0.015f;
float decayRate = 0.003f;
float sustainLevel = 0.6f;
float releaseRate = 0.003f;
float envLevel = 0.0f;

void noteOn()  { envState = ATTACK; lfoEnvelope = 0.0f; }
void noteOff() { envState = RELEASE; }

float analogReadLog(uint8_t ch, float minT, float maxT) {
  float pot = mcp.read(ch) / 4095.0f;
  return minT * powf(maxT / minT, pot);
}

void updateButtons() {
  static bool last1 = HIGH, last2 = HIGH, last3 = HIGH, last4 = HIGH;
  static unsigned long lastTime1 = 0, lastTime3 = 0, lastTime4 = 0;

  bool b1 = digitalRead(BUTTON1);
  bool b2 = digitalRead(BUTTON2);
  bool b3 = digitalRead(BUTTON3);
  bool b4 = digitalRead(BUTTON4);

  // OSC1 waveform cycle
  if (last1 == HIGH && b1 == LOW && millis() - lastTime1 > 50) {
    waveform1 = (waveform1 + 1) % 4;
    lastTime1 = millis();
  }

  // note on/off
  if (last2 == HIGH && b2 == LOW)  noteOn();
  if (last2 == LOW  && b2 == HIGH) noteOff();

  // mode cycle
  if (last3 == HIGH && b3 == LOW && millis() - lastTime3 > 50) {
    currentMode = (Mode)((currentMode + 1) % 4);
    lastTime3 = millis();
  }

  // OSC2 waveform cycle  
  if (last4 == HIGH && b4 == LOW && millis() - lastTime4 > 50) {
    waveform2 = (waveform2 + 1) % 4;
    lastTime4 = millis();
  }

  last1 = b1; last2 = b2; last3 = b3; last4 = b4;
}

void updateKnobs() {
  switch (currentMode) {
    case MAIN:
      volume = mcp.read(0) / 4095.0f;
      {
      float detuneAmount = (mcp.read(1) / 2047.5f) - 1.0f;
      baseFrequency = analogReadLog(2, 20.0f, 2000.0f);
      targetFrequency = baseFrequency * powf(2.0f, (midiNote - 69.0f) / 12.0f);
      detune = targetFrequency * (powf(2.0f, detuneAmount * 50.0f / 1200.0f) - 1.0f);
      }
    portamento = mcp.read(3) / 4095.0f; 
    if (mcp.read(3) < 10) portamento = 0.0f;
    osc2SemiOffset = (mcp.read(4) / 4095.0f) * 48.0f - 24.0f;  // -24 to +24 semitones
    break;

    case ADSR:
      attackRate = 1.0f / (analogReadLog(0, 0.001f, 3.0f)  * SAMPLE_RATE);
      decayRate = 1.0f / (analogReadLog(1, 0.005f, 2.0f)  * SAMPLE_RATE);
      sustainLevel = mcp.read(2) / 4095.0f;
      releaseRate = 1.0f / (analogReadLog(3, 0.005f, 5.0f)  * SAMPLE_RATE);
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
      lfoTarget = (float)(mcp.read(2) * 2 / 4095);
      lfoAttack = analogReadLog(3, 0.05f, 5.0f);   
      osc2Level = mcp.read(4) / 4095.0f;          
      break;
  }
}

void readMidi() {
  if (MIDI.read()) {
    if (MIDI.getType() == midi::NoteOn && MIDI.getData2() > 0) {
      midiNote        = MIDI.getData1();  
      targetFrequency = baseFrequency * powf(2.0f, (midiNote - 69.0f) / 12.0f);
      noteOn();
    }
    if (MIDI.getType() == midi::NoteOff || 
       (MIDI.getType() == midi::NoteOn && MIDI.getData2() == 0)) {
      noteOff();
    }
  }
}

int16_t applyFilter(int16_t input, float fc) {
  static float svf_low = 0.0f;
  static float svf_band = 0.0f;

  float f = 2.0f * sinf(M_PI * fc / SAMPLE_RATE);
  if (f > 0.95f) f = 0.95f;

  float q  = 2.0f - (resonance * 1.9f);
  float in = (float)input;

  svf_low  = svf_low + f * svf_band;
  float svf_high = in - svf_low - q * svf_band;
  svf_band = f * svf_high + svf_band;

  return (int16_t)constrain((int32_t)svf_low, -32768, 32767);
}

float applyLFO() {
  static float lfoPhase = 0.0f;

  float ar = 1.0f / (lfoAttack * SAMPLE_RATE);
  lfoEnvelope += ar;
  if (lfoEnvelope > 1.0f) lfoEnvelope = 1.0f;

  float lfoWave = sinf(lfoPhase);

  lfoPhase += 2.0f * M_PI * lfoRate / SAMPLE_RATE;
  if (lfoPhase >= 2.0f * M_PI) lfoPhase -= 2.0f * M_PI;

  return lfoWave * lfoDepth * lfoEnvelope;
}


inline int16_t generateSample(int wf, float ph) {
  switch (wf) {
    case 0: return (int16_t)(sinf(ph) * AMPLITUDE);
    case 1: return (int16_t)((ph < M_PI ? 1.0f : -1.0f) * AMPLITUDE);
    case 2: return (int16_t)((1.0f - ph / M_PI) * AMPLITUDE);
    case 3: return (int16_t)((ph < M_PI
              ? (ph / M_PI * 2.0f - 1.0f)
              : (3.0f - ph / M_PI * 2.0f)) * AMPLITUDE);
    default: return 0;
  }
}

void controlLoop() {
  while (true) {
    updateButtons();
    updateKnobs();
    delay(10);
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

  multicore_launch_core1(controlLoop);
}



void loop() {
  static float phase1 = 0.0f;
  static float phase2 = 0.0f;

  // Portamento
  float portaCoeff = 1.0f - portamento * 0.9997f;
  currentFrequency += (targetFrequency - currentFrequency) * portaCoeff;

  readMidi();



  // ADSR envelope
  switch (envState) {
    case ATTACK:
      envLevel += attackRate;
      if (envLevel >= 1.0f) { envLevel = 1.0f; envState = DECAY; }
      break;
    case DECAY:
      envLevel -= decayRate;
      if (envLevel <= sustainLevel) { envLevel = sustainLevel; envState = SUSTAIN; }
      break;
    case SUSTAIN: envLevel = sustainLevel; break;
    case RELEASE:
      envLevel -= releaseRate;
      if (envLevel <= 0.0f) { envLevel = 0.0f; envState = OFF; }
      break;
    case OFF: envLevel = 0.0f; break;
  }

  // --- LFO ---
  float lfoValue    = applyLFO();
  float frequencyMod = currentFrequency;
  float cutoffMod   = cutoff;

  switch ((int)lfoTarget) {
    case 0: frequencyMod += currentFrequency * lfoValue * 0.05f; break;
    case 2:
      cutoffMod *= (1.0f + lfoValue);
      cutoffMod  = constrain(cutoffMod, 20.0f, 20000.0f);
      break;
   
  }




  const float inc1 = 2.0f * M_PI * (frequencyMod + detune) / SAMPLE_RATE;

  float osc2Freq = frequencyMod * powf(2.0f, osc2SemiOffset / 12.0f);
  const float inc2 = 2.0f * M_PI * osc2Freq / SAMPLE_RATE;
  
  // Generate both oscillators
  int16_t s1 = generateSample(waveform1, phase1);
  int16_t s2 = generateSample(waveform2, phase2);

  // Mix oscillators
  int32_t mixed_osc = (int32_t)(s1 * (1.0f - osc2Level))
                    + (int32_t)(s2 * osc2Level);


  // Apply envelope + volume 
  int32_t s = (int32_t)(mixed_osc * envLevel * volume);
  s = constrain(s, -32768, 32767);

  // Volume LFO (applied after envelope) 
  if ((int)lfoTarget == 1) {
    s = (int32_t)(s * (1.0f + lfoValue * 0.5f));
    s = constrain(s, -32768, 32767);
  }

  int16_t sample = (int16_t)s;

  // Filter 
  sample = applyFilter(sample, cutoffMod);

  // Delay 
  int delaySamples = (int)(delayTime * (DELAY_MAX_SAMPLES - 1));
  int delayReadPos = delayWritePos - delaySamples;
  if (delayReadPos < 0) delayReadPos += DELAY_MAX_SAMPLES;

  int16_t delaySig = delayBuffer[delayReadPos];
  delayBuffer[delayWritePos] = sample + (int16_t)(delaySig * 0.5f);
  delayWritePos = (delayWritePos + 1) % DELAY_MAX_SAMPLES;

  int32_t out = (int32_t)(sample * (1.0f - delayMix))
              + (int32_t)(delaySig * delayMix);
  out = constrain(out, -32768, 32767);

  i2s.write16((int16_t)out, (int16_t)out);

  // Advance phases
  phase1 += inc1;
  if (phase1 >= 2.0f * M_PI) phase1 -= 2.0f * M_PI;

  phase2 += inc2;
  if (phase2 >= 2.0f * M_PI) phase2 -= 2.0f * M_PI;
}