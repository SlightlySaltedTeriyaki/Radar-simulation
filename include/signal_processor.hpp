#pragma once
#include "signal_generator.hpp"
#include <complex>
#include <vector>
#include <cstdint>

struct Detection {
    uint16_t range_bin;
    float    amplitude;
    float    velocity = 0.0f;  // estimated |velocity| in bins/CPI (from Doppler bin)
};

class SignalProcessor {
public:
    SignalProcessor(int guard_cells, int reference_cells, float threshold_factor);

    // Single-sweep 1D CFAR — used by benchmark and tests (unchanged).
    const std::vector<Detection>& process(const Sweep& sweep);

    // CPI Range-Doppler processing: FFT across pulses + CFAR on collapsed profile.
    const std::vector<Detection>& process_cpi(const CPI& cpi);

    // Pointer to the fftshifted Range-Doppler map for GUI display.
    // Layout: NUM_PULSES rows × RANGE_BINS cols, row 0 = most negative velocity.
    const float* rd_display() const { return rd_display_.data(); }

private:
    int   guard_;
    int   ref_;
    float threshold_factor_;

    std::vector<Detection>           result_buffer_;
    std::vector<std::complex<float>> fft_buf_;     // NUM_PULSES — reused across calls
    std::vector<float>               rd_map_;      // NUM_PULSES * RANGE_BINS
    std::vector<float>               rd_display_;  // fftshifted, for GUI
    std::vector<float>               range_profile_; // RANGE_BINS — max across Doppler
    std::vector<int>                 best_doppler_;  // RANGE_BINS — argmax Doppler bin

    // In-place Cooley-Tukey radix-2 FFT (size must be power of 2)
    static void fft_inplace(std::complex<float>* x, int N);
};
