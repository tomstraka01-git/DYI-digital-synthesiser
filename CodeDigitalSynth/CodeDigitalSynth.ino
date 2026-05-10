#include <I2S.h>
#include <SPI.h>
#include "MCP_ADC.h"

#define LRCK_PIN  28   
#define BCK_PIN   27   
#define DIN_PIN   26   



#define SAMPLE_RATE 44100
#define FREQUENCY   440.0f   // A4 note
#define AMPLITUDE   32767 // maximum, later i will apply volume multiplication


#define MCP_DOUT  16   
#define MCP_CS    17   
#define MCP_DIN   18   
#define MCP_CLK   19


int waveform = 0; // 0 for sine, 1 for square, 2 for saw, 3 for triangle
I2S i2s(OUTPUT);
MCP3208 mcp(&SPI); 


enum ADSRState { ATTACK, DECAY, SUSTAIN, RELEASE, OFF };
ADSRState envState = OFF;


float attackRate = 0.015f; // max 0.0000075f 
float decayRate = 0.003fl; // max 0.0000113f
float sustainLevel = 0.6f; // 0 to 1
float releaseRate = 0.003f; // max 0.0000045f

float envLevel = 0.0f;  // current envelope volume (0.0 – 1.0)

// Call this to start a note (e.g. button press)
void noteOn()  { envState = ATTACK; }

// Call this to release a note (e.g. button release)
void noteOff() { envState = RELEASE; }

// uses log to make the potenciometer feel nonlinear.
float analogReadLog(uint8_t ch, float minT, float maxT) {
  float pot = mcp.read(ch) / 4095.0f;
  return minT * powf(maxT / minT, pot);
}


void updateADSR() {
  attackRate = 1.0f / (analogReadLog(0, 0.001f, 3.0f) * SAMPLE_RATE);
  decayRate = 1.0f / (analogReadLog(1, 0.005f, 2.0f) * SAMPLE_RATE);
  sustainLevel = mcp.read(2) / 4095.0f;
  releaseRate = 1.0f / (analogReadLog(3, 0.005f, 5.0f) * SAMPLE_RATE);
}


void setup() {
  
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
}

void loop() {
  static float phase = 0.0f;
  const float inc = 2.0f * M_PI * FREQUENCY / SAMPLE_RATE;

  int16_t sample = 0;
  
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
  
  sample = (int16_t)(sample * envLevel);

  i2s.write16(sample, sample);  // Left, Right

  phase += inc;
  if (phase >= 2.0f * M_PI) phase -= 2.0f * M_PI;
}



