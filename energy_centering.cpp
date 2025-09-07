int detect_energy_peak(const float* buffer, int length, float threshold) {
  const int window_size = 256;
  float max_energy = 0;
  int peak_index = -1;

  for (int i = 0; i <= length - window_size; i++) {
    float energy = 0;
    for (int j = 0; j < window_size; j++) {
      energy += buffer[i + j] * buffer[i + j];
    }

    if (energy > max_energy && energy > threshold) {
      max_energy = energy;
      peak_index = i + window_size / 2;
    }
  }

  return peak_index;  // -1 si aucun pic significatif
}

void extract_centered_segment(const float* input, int input_length, float* output, int output_length, int center_index) {
  int start = center_index - output_length / 2;
  if (start < 0) start = 0;
  if (start + output_length > input_length) start = input_length - output_length;

  for (int i = 0; i < output_length; i++) {
    output[i] = input[start + i];
  }
}
