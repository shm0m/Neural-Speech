#pragma once

int detect_energy_peak(const float* buffer, int length, float threshold);
void extract_centered_segment(const float* input, int input_length, float* output, int output_length, int center_index);
