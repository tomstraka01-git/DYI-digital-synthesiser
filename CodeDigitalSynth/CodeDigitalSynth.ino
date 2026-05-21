#include <I2S.h>
#include <SPI.h>
#include "MCP_ADC.h"
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>
#include "pico/multicore.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#include "hardware/sync.h"

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


#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


Adafruit_USBD_MIDI usb_midi;
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

// Chorus variables
#define CHORUS_SAMPLES 2000
int16_t chorusBuffer[CHORUS_SAMPLES];

float chorus_rate = 0.0f;
float chorus_depth = 0.0f;
int chorusWritePos = 0;
float chorusMix = 0.0f;


enum ADSRState { ATTACK, DECAY, SUSTAIN, RELEASE, OFF };
ADSRState envState = OFF;

#define WAVE_SAMPLES 128
volatile int16_t waveBuffer[WAVE_SAMPLES];
volatile int waveIndex = 0;

// Polyphony
#define MAX_VOICES 4

struct Voice {
  float phase1, phase2;
  float currentFreq, targetFreq;
  float envLevel;
  ADSRState envState;
  int midiNote;
  bool active;
};

Voice voices[MAX_VOICES];


struct SVF { float low, band; };
SVF svf[MAX_VOICES] = {};

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

enum Mode { MAIN, ADSR, FX, FX2, LFO };
Mode currentMode = MAIN;

int waveform1 = 0;
int waveform2 = 1;

I2S i2s(OUTPUT);
MCP3208 mcp(&SPI); 



float attackRate = 0.015f;
float decayRate = 0.003f;
float sustainLevel = 0.6f;
float releaseRate = 0.003f;
float envLevel = 0.0f;

int allocateVoice(int note) {
  // First: reuse same note if already playing
  for (int i = 0; i < MAX_VOICES; i++)
    if (voices[i].midiNote == note && voices[i].envState != OFF)
      return i;

  // Second: find a silent voice
  for (int i = 0; i < MAX_VOICES; i++)
    if (voices[i].envState == OFF) return i;

  // Third: steal the voice with lowest envelope level
  int steal = 0;
  for (int i = 1; i < MAX_VOICES; i++)
    if (voices[i].envLevel < voices[steal].envLevel) steal = i;
  return steal;
}

void voiceNoteOn(int note) {
  float freq = baseFrequency * powf(2.0f, (note - 69.0f) / 12.0f);
  int v = allocateVoice(note);
  voices[v].midiNote   = note;
  voices[v].targetFreq = freq;
  if (voices[v].envState == OFF)
    voices[v].currentFreq = freq;  // snap freq if voice was silent
  voices[v].envState = ATTACK;
  voices[v].envLevel = 0.0f;
  voices[v].active   = true;
  lfoEnvelope = 0.0f;
}

void voiceNoteOff(int note) {
  for (int i = 0; i < MAX_VOICES; i++)
    if (voices[i].midiNote == note &&
        voices[i].envState != OFF &&
        voices[i].envState != RELEASE)
      voices[i].envState = RELEASE;
}

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
  if (last2 == HIGH && b2 == LOW)  voiceNoteOn(69);
  if (last2 == LOW  && b2 == HIGH) voiceNoteOff(69);

  // mode cycle
  if (last3 == HIGH && b3 == LOW && millis() - lastTime3 > 50) {
    currentMode = (Mode)((currentMode + 1) % 5);
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
    
  
    float a4Freq = 300.0f + (mcp.read(2) / 4095.0f) * 180.0f;
    baseFrequency = a4Freq;  // baseFrequency now means "what is A4"

    detune = a4Freq * (powf(2.0f, detuneAmount * 50.0f / 1200.0f) - 1.0f);
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
      cutoff    = analogReadLog(0, 200.0f, 8000.0f);
      resonance = mcp.read(1) / 4095.0f;
      delayTime = mcp.read(2) / 4095.0f;
      delayMix  = mcp.read(3) / 4095.0f;
      break;

    case FX2:
      chorus_rate  = mcp.read(0) / 4095.0f * 3.0f;
      chorus_depth = mcp.read(1) / 4095.0f * 20.0f;
      chorusMix    = mcp.read(2) / 4095.0f;
      break;

    case LFO:

      lfoRate = analogReadLog(0, 0.1f, 20.0f);
      lfoDepth = mcp.read(1) / 4095.0f;
      lfoTarget = (float)(mcp.read(2) * 3 / 4096);
      lfoAttack = analogReadLog(3, 0.05f, 5.0f);   
      osc2Level = mcp.read(4) / 4095.0f;          
      break;
  }
}

void readMidi() {
  if (MIDI.read()) {
    byte type = MIDI.getType();
    byte note = MIDI.getData1();
    byte vel  = MIDI.getData2();

    if (type == midi::NoteOn && vel > 0) {
      voiceNoteOn(note);
    } else if (type == midi::NoteOff ||
              (type == midi::NoteOn && vel == 0)) {
      voiceNoteOff(note);
    }
  }
}

int16_t applyFilter(int v, int16_t input, float fc) {
  float f = 2.0f * sinf(M_PI * fc / SAMPLE_RATE);
  if (f > 0.95f) f = 0.95f;
  float q  = 2.0f - (resonance * 1.9f);
  float in = (float)input;
  svf[v].low  += f * svf[v].band;
  float high   = in - svf[v].low - q * svf[v].band;
  svf[v].band  = f * high + svf[v].band;
  return (int16_t)constrain((int32_t)svf[v].low, -32768, 32767);
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

const char* waveName(int wf) {
  switch(wf) {
    case 0: return "SIN";
    case 1: return "SQR";
    case 2: return "SAW";
    case 3: return "TRI";
    default: return "???";
  }
}

const char* modeName(Mode m) {
  switch(m) {
    case MAIN: return "MAIN";
    case ADSR: return "ADSR";
    case FX:   return "FX1 ";
    case FX2:  return "FX2 ";
    case LFO:  return "LFO ";
    default:   return "????";
  }
}

void updateOled(int16_t sample) {
  display.clearDisplay();

  // Waveform part (top 18px) 
  int16_t localBuf[WAVE_SAMPLES];
  uint32_t savedIdx;

  
  uint32_t irq = save_and_disable_interrupts();
  memcpy((void*)localBuf, (void*)waveBuffer, sizeof(localBuf));
  savedIdx = waveIndex;
  restore_interrupts(irq);

  for (int x = 0; x < 128; x++) {
    int idx = (savedIdx + x) % WAVE_SAMPLES;
    int y = map((int)localBuf[idx], -32768, 32767, 17, 0);
    display.drawPixel(x, y, SSD1306_WHITE);
  }

  // Divider line 
  display.drawFastHLine(0, 19, 128, SSD1306_WHITE);

  // Mode indicator (top right) 
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(98, 21);
  display.print(modeName(currentMode));

  
  switch (currentMode) {

    case MAIN:

      display.setCursor(0, 21);
      display.print("O1:"); display.print(waveName(waveform1));
      display.print(" O2:"); display.print(waveName(waveform2));

      display.setCursor(0, 31);
      display.print("A4:"); display.print((int)baseFrequency); display.print("Hz"); // changed Fr: to A4:

      display.setCursor(0, 41);
      display.print("Vl:"); display.print((int)(volume * 100)); display.print("%");
      display.print(" Dt:"); display.print((int)(detune * 10) / 10.0f);

      display.setCursor(0, 51);
      display.print("PT:"); display.print((int)(portamento * 100)); display.print("%");
      display.print(" O2:"); display.print((int)(osc2SemiOffset)); display.print("st");
      break;

    case ADSR: {
      // Draw ADSR shape (right side, 50px wide)
      // A
      display.drawLine(78, 62, 88, 22, SSD1306_WHITE);
      // D
      int dY = 22 + (int)((1.0f - sustainLevel) * 25);
      display.drawLine(88, 22, 98, dY, SSD1306_WHITE);
      // S
      display.drawLine(98, dY, 108, dY, SSD1306_WHITE);
      // R
      display.drawLine(108, dY, 118, 62, SSD1306_WHITE);

      // Text values (left side)
      display.setCursor(0, 21);
      display.print("A:"); 
      display.print(1.0f / (attackRate * SAMPLE_RATE), 2);
      display.print("s");

      display.setCursor(0, 31);
      display.print("D:");
      display.print(1.0f / (decayRate  * SAMPLE_RATE), 2);
      display.print("s");

      display.setCursor(0, 41);
      display.print("S:");
      display.print((int)(sustainLevel * 100));
      display.print("%");

      display.setCursor(0, 51);
      display.print("R:");
      display.print(1.0f / (releaseRate * SAMPLE_RATE), 2);
      display.print("s");
      break;
    }

    case FX:
      display.setCursor(0, 21);
      display.print("Cut:"); display.print((int)cutoff); display.print("Hz");

      display.setCursor(0, 31);
      display.print("Res:"); display.print((int)(resonance * 100)); display.print("%");

      display.setCursor(0, 41);
      display.print("Dly:"); display.print((int)(delayTime * 1000)); display.print("ms");
      display.print(" Mx:"); display.print((int)(delayMix * 100)); display.print("%");

      display.setCursor(0, 51);
      display.print("Chr:"); display.print(chorus_rate, 1); display.print("Hz");
      display.print(" Mx:"); display.print((int)(chorusMix * 100)); display.print("%");
      break;
    
    case FX2:
      display.setCursor(0, 21);
      display.print("CRt:"); display.print(chorus_rate, 1); display.print("Hz");

      display.setCursor(0, 31);
      display.print("CDp:"); display.print((int)chorus_depth); display.print("ms");

      display.setCursor(0, 41);
      display.print("CMx:"); display.print((int)(chorusMix * 100)); display.print("%");
      break;
      
    case LFO: {
      const char* targets[] = { "PITCH", "VOL  ", "FILT " };
      display.setCursor(0, 21);
      display.print("Rt:"); display.print(lfoRate, 1); display.print("Hz");

      display.setCursor(0, 31);
      display.print("Dp:"); display.print((int)(lfoDepth * 100)); display.print("%");

      display.setCursor(0, 41);
      display.print("Tg:"); display.print(targets[(int)lfoTarget]);

      display.setCursor(0, 51);
      display.print("Atk:"); display.print(lfoAttack, 1); display.print("s");
      break;
    }
  }

  // MIDI note + env state (bottom right) 

  ADSRState displayState = OFF;
  for (int i = 0; i < MAX_VOICES; i++)
    if (voices[i].envState != OFF) { displayState = voices[i].envState; break; }

  const char* envNames[] = { "ATK","DEC","SUS","REL","OFF" };
  display.setCursor(98, 51);
  display.print(envNames[displayState]);

  display.display();
}

void controlLoop() {
  while (true) {
    updateButtons();
    updateKnobs();
    updateOled(0);
    delay(50);
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

  
  Wire.setSDA(4);  // GP4
  Wire.setSCL(5);  // GP5
  Wire.begin();

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); 
  display.clearDisplay();

  multicore_launch_core1(controlLoop);
}


void loop() {
  readMidi();

  float lfoValue  = applyLFO();
  float cutoffMod = cutoff;
  if ((int)lfoTarget == 2) {
    cutoffMod *= (1.0f + lfoValue);
    cutoffMod  = constrain(cutoffMod, 20.0f, 20000.0f);
  }

  float portaCoeff = 1.0f - portamento * 0.9997f;
  int32_t mixedOut = 0;

  for (int v = 0; v < MAX_VOICES; v++) {
    Voice& vx = voices[v];
    if (vx.envState == OFF) continue;

    // Portamento
    vx.currentFreq += (vx.targetFreq - vx.currentFreq) * portaCoeff;

    // ADSR
    switch (vx.envState) {
      case ATTACK:
        vx.envLevel += attackRate;
        if (vx.envLevel >= 1.0f) { vx.envLevel = 1.0f; vx.envState = DECAY; }
        break;
      case DECAY:
        vx.envLevel -= decayRate;
        if (vx.envLevel <= sustainLevel) { vx.envLevel = sustainLevel; vx.envState = SUSTAIN; }
        break;
      case SUSTAIN:
        vx.envLevel = sustainLevel;
        break;
      case RELEASE:
        vx.envLevel -= releaseRate;
        if (vx.envLevel <= 0.0f) {
          vx.envLevel = 0.0f;
          vx.envState = OFF;
          vx.active   = false;
          continue;
        }
        break;
      default: break;
    }

    // Frequency + LFO pitch mod
    float freqMod = vx.currentFreq;
    if ((int)lfoTarget == 0)
      freqMod += vx.currentFreq * lfoValue * 0.05f;

    float inc1 = 2.0f * M_PI * (freqMod + detune) / SAMPLE_RATE;
    float osc2Freq = freqMod * powf(2.0f, osc2SemiOffset / 12.0f);
    float inc2 = 2.0f * M_PI * osc2Freq / SAMPLE_RATE;

    // Oscillators
    int16_t s1 = generateSample(waveform1, vx.phase1);
    int16_t s2 = generateSample(waveform2, vx.phase2);
    int32_t osc = (int32_t)(s1 * (1.0f - osc2Level))
                + (int32_t)(s2 * osc2Level);

    // Envelope + volume
    int32_t s = (int32_t)(osc * vx.envLevel * volume);

    // Volume LFO
    if ((int)lfoTarget == 1)
      s = (int32_t)(s * (1.0f + lfoValue * 0.5f));

    s = constrain(s, -32768, 32767);

    // Per-voice filter
    int16_t filtered = applyFilter(v, (int16_t)s, cutoffMod);

    // Mix (divide by MAX_VOICES to prevent clipping)
    mixedOut += filtered / MAX_VOICES;

    // Advance phases
    vx.phase1 += inc1;
    if (vx.phase1 >= 2.0f * M_PI) vx.phase1 -= 2.0f * M_PI;
    vx.phase2 += inc2;
    if (vx.phase2 >= 2.0f * M_PI) vx.phase2 -= 2.0f * M_PI;
  }

  // Delay
  mixedOut = constrain(mixedOut, -32768, 32767);
  int16_t preDly = (int16_t)mixedOut;

  int delaySamples = (int)(delayTime * (DELAY_MAX_SAMPLES - 1));
  int delayReadPos = delayWritePos - delaySamples;
  if (delayReadPos < 0) delayReadPos += DELAY_MAX_SAMPLES;

  int16_t delaySig = delayBuffer[delayReadPos];
  delayBuffer[delayWritePos] = preDly + (int16_t)(delaySig * 0.5f);
  delayWritePos = (delayWritePos + 1) % DELAY_MAX_SAMPLES;

  int32_t out = (int32_t)(preDly * (1.0f - delayMix))
              + (int32_t)(delaySig * delayMix);
  out = constrain(out, -32768, 32767);


// Chorus
  static float chorusLfoPhase = 0.0f;
  chorusBuffer[chorusWritePos] = (int16_t)out;

  float chorusLfo = sinf(chorusLfoPhase);
  chorusLfoPhase += 2.0f * M_PI * chorus_rate / SAMPLE_RATE;
  if (chorusLfoPhase >= 2.0f * M_PI) chorusLfoPhase -= 2.0f * M_PI;

  // Base delay ~10ms plus LFO modulation
  float delayMs = 10.0f + chorusLfo * chorus_depth;
  int chorusDelaySamples = (int)(delayMs * SAMPLE_RATE / 1000.0f);
  chorusDelaySamples = constrain(chorusDelaySamples, 1, CHORUS_SAMPLES - 1);

  int chorusReadPos = chorusWritePos - chorusDelaySamples;
  if (chorusReadPos < 0) chorusReadPos += CHORUS_SAMPLES;

  int16_t chorusSig = chorusBuffer[chorusReadPos];
  chorusWritePos = (chorusWritePos + 1) % CHORUS_SAMPLES;

  out = (int32_t)(out * (1.0f - chorusMix))
      + (int32_t)(chorusSig * chorusMix);
  out = constrain(out, -32768, 32767);

  
  waveBuffer[waveIndex % WAVE_SAMPLES] = preDly;
  waveIndex++;

  i2s.write16((int16_t)out, (int16_t)out);
}