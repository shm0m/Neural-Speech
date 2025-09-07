#include <Arduino.h>
#include "arduinoMFCC.h"
#include "forward_pass.h"

#define GAIN 100.0
#define FILTER_ORDER 5
#define FRAME_SIZE 256
#define HOP_SIZE 128
#define N_FRAMES 48
#define SIGNAL_LENGTH (HOP_SIZE * (N_FRAMES - 1) + FRAME_SIZE)

const float b[] = {0.00327922, 0.01639608, 0.03279216, 0.03279216, 0.01639608, 0.00327922};

float input_buffer[FILTER_ORDER + 1] = {0};
int buffer_index = 0;
int downsample_counter = 0;

float signal_buffer[SIGNAL_LENGTH];
int signal_index = 0;

float frame_buffer[FRAME_SIZE];
int frame_index = 0;

volatile bool recording = true;
volatile bool buttonPressed = false;

arduinoMFCC mymfcc(13, 13, FRAME_SIZE, 8000);
float mfcc_output[N_FRAMES][13];


void predictFromMFCCs() {
  float input[624]; // 48 frames × 13 coefficients = 624 entrées
  float output[2] = {0};

  // On concatène les MFCCs frame par frame dans un seul vecteur
  int index = 0;
  for (int i = 0; i < N_FRAMES; i++) {
    for (int j = 0; j < 13; j++) {
      input[index++] = mfcc_output[i][j];
    }
  }

  // Passe une seule fois dans le réseau de neurones
  forward_pass(input, output);

  // Applique softmax pour convertir les scores en probabilités
  float confidence[2];
  softmax(output, confidence);

  int prediction = (confidence[0] > confidence[1]) ? 0 : 1;

    Serial.print("Prédiction : ");
    Serial.print(prediction == 0 ? "LINUX " : "MAC ");
    Serial.print(" | Confiance : ");
    Serial.print(confidence[prediction] * 100, 1);
    Serial.println(" %");

    if(prediction==0){
      digitalWrite(4,HIGH);
      delay(1000);
      digitalWrite(4,LOW);
  }
  else{
      digitalWrite(4,HIGH);
      delay(333);
      digitalWrite(4,LOW);
      delay(333);
      digitalWrite(4,HIGH);
      delay(333);
      digitalWrite(4,LOW);
  }
}


// ADC config 32kHz
void setupADC() {
  PMC->PMC_PCER1 |= PMC_PCER1_PID37;
  ADC->ADC_MR = ADC_MR_PRESCAL(0) | ADC_MR_STARTUP_SUT64 | ADC_MR_TRACKTIM(15) | ADC_MR_SETTLING_AST3;
  ADC->ADC_CHER = 0x80;
  PMC->PMC_PCER0 |= PMC_PCER0_PID27;
  TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_CPCTRG;
  TC0->TC_CHANNEL[0].TC_RC = 20; // 32kHz
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  NVIC_EnableIRQ(TC0_IRQn);
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
}

void TC0_Handler() {
  TC0->TC_CHANNEL[0].TC_SR;
  ADC->ADC_CR = ADC_CR_START;
}

float applyFIRFilter(float* buffer, int index) {
  float result = 0.0;
  int j = index;
  for (int i = 0; i <= FILTER_ORDER; i++) {
    result += b[i] * buffer[j];
    j = (j > 0) ? (j - 1) : FILTER_ORDER;
  }
  return result;
}

void startRecording() {
  if (!recording) {
    buttonPressed = true;
  }
}

void computeMFCCsFromBuffer() {
  mymfcc.create_hamming_window();
  mymfcc.create_mel_filter_bank();
  mymfcc.create_dct_matrix();

  for (int i = 0; i < N_FRAMES; i++) {
    int start = i * HOP_SIZE;
    float frame[FRAME_SIZE];

    for (int j = 0; j < FRAME_SIZE; j++) {
      frame[j] = signal_buffer[start + j];
    }

    float mfcc[13];
    mymfcc.compute(frame, mfcc);

    for (int k = 0; k < 13; k++) {
      mfcc_output[i][k] = mfcc[k];
    }
  }

  // Affichage des MFCCs
    //Serial.print("[");
  for (int i = 0; i < N_FRAMES; i++) {
    Serial.print("Frame ");
    Serial.print(i);
    Serial.print(": ");
    for (int j = 0; j < 13; j++) {
      Serial.print(mfcc_output[i][j], 4);
      Serial.print(", ");
    }
    Serial.println();
  }
    //Serial.println("],");

    digitalWrite(3,HIGH);
    delay(100);
    digitalWrite(3,LOW);
    delay(100);
    digitalWrite(3,HIGH);
    delay(100);
    digitalWrite(3,LOW);

}

void setup() {
  Serial.begin(460800);
  analogReadResolution(12);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  setupADC();
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), startRecording, FALLING);
  mymfcc.create_hamming_window();
  mymfcc.create_mel_filter_bank();
  mymfcc.create_dct_matrix();

  recording = true;
  buttonPressed = true;
  signal_index = 0;
}

void loop() {
  if (buttonPressed) {
    recording = true;
    signal_index = 0;
    buttonPressed = false;
    //Serial.println("Enregistrement...");
    digitalWrite(3,HIGH);
  }

  if (!recording || (ADC->ADC_ISR & 0x80) == 0) return;

  uint16_t value = ADC->ADC_CDR[7];
  input_buffer[buffer_index] = (float)value;
  buffer_index = (buffer_index + 1) % (FILTER_ORDER + 1);
  downsample_counter++;

  if (downsample_counter >= 4) {  // 32kHz → 8kHz
    downsample_counter = 0;

    float filtered = applyFIRFilter(input_buffer, buffer_index) * GAIN;
    int16_t centered = (int16_t)filtered - 2048;

    if (signal_index < SIGNAL_LENGTH) {
      signal_buffer[signal_index++] = (float)centered;
    }

    if (signal_index >= SIGNAL_LENGTH) {
      recording = false;
      //Serial.println("Enregistrement terminé.");

    digitalWrite(3,LOW);
      computeMFCCsFromBuffer();
      predictFromMFCCs();
    }
  }
}