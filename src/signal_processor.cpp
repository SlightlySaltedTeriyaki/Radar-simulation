#include "signal_processor.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
using namespace std;

static constexpr float PI = 3.14159265359f;

SignalProcessor::SignalProcessor(int guard_cells, int reference_cells, float threshold_factor)
    : guard_(guard_cells)
    , ref_(reference_cells)
    , threshold_factor_(threshold_factor)
{
    result_buffer_.reserve(64);
    fft_buf_.resize(NUM_PULSES);
    rd_map_.assign(NUM_PULSES * RANGE_BINS, 0.0f);
    rd_display_.assign(NUM_PULSES * RANGE_BINS, 0.0f);
    range_profile_.resize(RANGE_BINS, 0.0f);
    best_doppler_.resize(RANGE_BINS, 0);
}

const vector<Detection>& SignalProcessor::process(const Sweep& sweep) {
    result_buffer_.clear();

    const int half_window = guard_ + ref_;
    const int first = half_window;
    const int last  = RANGE_BINS - half_window - 1;

    float sum = 0;
    for (int j = first - half_window; j <= first - guard_ - 1; ++j)
        sum += std::abs(sweep[j]);
    for (int j = first + guard_ + 1; j <= first + half_window; ++j)
        sum += std::abs(sweep[j]);

    for (int i = first; i <= last; ++i) {
        if (i > first) {
            sum -= std::abs(sweep[i - half_window - 1]);
            sum += std::abs(sweep[i - guard_ - 1]);
            sum -= std::abs(sweep[i + guard_]);
            sum += std::abs(sweep[i + half_window]);
        }
        float threshold = (sum / (2 * ref_)) * threshold_factor_;
        if (std::abs(sweep[i]) > threshold)
            result_buffer_.push_back({static_cast<uint16_t>(i), sweep[i], 0.0f});
    }
    return result_buffer_;
}

void SignalProcessor::fft_inplace(complex<float>* x, int N) {
    // Bit-reversal permutation
    for (int i = 1, j = 0; i < N; ++i) {
        int bit = N >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) swap(x[i], x[j]);
    }
    // Cooley-Tukey butterfly stages
    for (int len = 2; len <= N; len <<= 1) {
        float ang = -2.0f * PI / float(len);
        complex<float> wlen(std::cos(ang), std::sin(ang));
        for (int i = 0; i < N; i += len) {
            complex<float> w(1.0f, 0.0f);
            for (int j = 0; j < len / 2; ++j) {
                auto u = x[i + j];
                auto t = w * x[i + j + len / 2];
                x[i + j]           = u + t;
                x[i + j + len / 2] = u - t;
                w *= wlen;
            }
        }
    }
}

const vector<Detection>& SignalProcessor::process_cpi(const CPI& cpi) {
    result_buffer_.clear();

    // FFT each range bin across pulses. Layout: rd_map_[doppler * RANGE_BINS + range].
    for (int r = 0; r < RANGE_BINS; ++r) {
        for (int n = 0; n < NUM_PULSES; ++n)
            fft_buf_[n] = cpi[n][r];
        fft_inplace(fft_buf_.data(), NUM_PULSES);
        for (int d = 0; d < NUM_PULSES; ++d)
            rd_map_[d * RANGE_BINS + r] = std::abs(fft_buf_[d]);
    }

    // fftshift for display: row d shows source bin (d + N/2) % N,
    // so row 0 = most negative velocity and row N/2 = zero velocity.
    for (int d = 0; d < NUM_PULSES; ++d) {
        int src = (d + NUM_PULSES / 2) % NUM_PULSES;
        std::memcpy(&rd_display_[d * RANGE_BINS],
                    &rd_map_[src * RANGE_BINS],
                    RANGE_BINS * sizeof(float));
    }

    // Collapse to 1D: max magnitude across Doppler bins per range bin.
    // The winning bin index is saved to convert to a velocity estimate later.
    for (int r = 0; r < RANGE_BINS; ++r) {
        float best = 0.0f;
        int   bidx = 0;
        for (int d = 0; d < NUM_PULSES; ++d) {
            float v = rd_map_[d * RANGE_BINS + r];
            if (v > best) { best = v; bidx = d; }
        }
        range_profile_[r] = best;
        best_doppler_[r]  = bidx;
    }

    // Sliding CA-CFAR on the collapsed range profile.
    const int half_window = guard_ + ref_;
    const int first       = half_window;
    const int last        = RANGE_BINS - half_window - 1;

    float sum = 0.0f;
    for (int j = first - half_window; j <= first - guard_ - 1; ++j)
        sum += range_profile_[j];
    for (int j = first + guard_ + 1; j <= first + half_window; ++j)
        sum += range_profile_[j];

    for (int i = first; i <= last; ++i) {
        if (i > first) {
            sum -= range_profile_[i - half_window - 1];
            sum += range_profile_[i - guard_ - 1];
            sum -= range_profile_[i + guard_];
            sum += range_profile_[i + half_window];
        }

        float threshold = (sum / (2 * ref_)) * threshold_factor_;
        if (range_profile_[i] > threshold) {
            // Real-valued (non-IQ) input means bins b and (N-b) are mirror images with
            // the same |velocity|. Fold to [0, N/2] to resolve the ambiguity.
            int   b    = best_doppler_[i];
            int   babs = std::min(b, NUM_PULSES - b);
            float vel  = float(babs) * 2.0f * MAX_VELOCITY / float(NUM_PULSES);
            result_buffer_.push_back({static_cast<uint16_t>(i), range_profile_[i], vel});
        }
    }

    return result_buffer_;
}
