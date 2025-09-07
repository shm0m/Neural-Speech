# Neural-Speech

A project for word recognition using a neural network on Arduino.

## Features

- MFCC (Mel Frequency Cepstral Coefficient) extraction (`arduinoMFCC.h`)
- Energy peak detection and signal centering (`energy_centering.cpp`)
- Simple convolutional neural network (`cnn.cpp`, `cnn.h`)
- Final prediction with a dense neural network (`forward_pass.cpp`, `forward_pass.h`)
- Includes training and test data (`training_data.h`, `test_data.h`)

## File Structure

- **Neural-Speech.ino**: Arduino entry point, audio signal acquisition and processing
- **arduinoMFCC.cpp / .h**: MFCC extraction, normalization, DCT, windowing
- **cnn.cpp / .h**: Convolution, pooling, flattening for the CNN
- **forward_pass.cpp / .h**: Dense neural network for prediction
- **energy_centering.cpp / .h**: Energy peak detection and signal centering
- **weights.h**: Neural network weights
- **training_data.h / test_data.h**: MFCC data for training and testing
- **training_batch.txt / test_batch.txt**: Raw data batches
- **training_labels.txt / test_labels.txt**: Labels for the data
- **README.md**: This file

## Usage

1. **Prepare Arduino Environment**  
   Use an Arduino Due (code is designed for this platform).

2. **Acquisition and Processing**  
   The audio signal is acquired, centered on the energy peak, split into frames, and transformed into MFCCs.

3. **Prediction**  
   MFCCs are passed through the CNN and dense network for final prediction.

4. **Output**  
   Prediction and confidence are displayed via the serial port.

## Example

```cpp
void predictFromMFCCs() {
  float input[624]; // 48 frames × 13 coefficients
  float output[2] = {0};
  // Concatenate MFCCs
  // forward_pass(input, output);
  // softmax(output, confidence);
}
```

## Credits

Based on the MFCC library by [Foued DERRAZ](https://github.com/FouedDrz/arduinoMFCC), adapted by Clément
