/*#include "forward_pass.h"
#include "weights.h"
#include <math.h>

void forward_pass(float mfcc[13], float output[2]) {
  float hidden[50];

  for (int j = 0; j < 50; j++) {
    float sum = 0;
    for (int i = 0; i < 13; i++){
      sum += IW1[j][i] * mfcc[i];
      hidden[j] = tanh(sum + b1[j]);}
  }

  for (int k = 0; k < 2; k++) {
    float sum = 0;
    for (int j = 0; j < 50; j++){
      sum += LW2[k][j] * hidden[j];
      output[k] = sum + b2[k];}
  }
}

*/

#include "forward_pass.h"
#include "weights.h"
#include <math.h>

float sigmoid(float x) {
  return 1.0f / (1.0f + expf(-x));
}

void forward_pass(float input[624], float output[2]) {
  float layer1[50];
  float layer2[30];
  float layer3[2];

  // Couche 1 : ReLU( IW1 * input + b1 )
  for (int j = 0; j < 50; j++) {
    float sum = 0;
    for (int i = 0; i < 624; i++) {
      sum += IW1[j][i] * input[i];
    }
    layer1[j] = fmaxf(0.0f, sum + b1[j]);
  }

  // Couche 2 : ReLU( IW2 * layer1 + b2 )
  for (int j = 0; j < 30; j++) {
    float sum = 0;
    for (int i = 0; i < 50; i++) {
      sum += IW2[j][i] * layer1[i];
    }
    layer2[j] = fmaxf(0.0f, sum + b2[j]);
  }

  // Couche 3 : Sigmoid( IW3 * layer2 + b3 )
  for (int j = 0; j < 2; j++) {
    float sum = 0;
    for (int i = 0; i < 30; i++) {
      sum += IW3[j][i] * layer2[i];
    }
    layer3[j] = sigmoid(sum + b3[j]);
  }

  output[0] = layer3[0];
  output[1] = layer3[1];
}
