#pragma once
#include <array>
#include <random>
#include <cstdint>

static constexpr int RANGE_BINS = 1024;  // počet vzdálenostních buněk v jednom sweepů

using Sweep = std::array<float, RANGE_BINS>;

// FÁZE 2: Generuje simulované radarové sweepy.
class SignalGenerator {
public:
    explicit SignalGenerator(uint32_t seed, int num_targets = 1, float noise_sigma = 1.0f);

    // Vrátí jeden sweep: šum + náhodně umístěné cíle.
    Sweep generate();

private:
    std::mt19937 rng_;
    std::normal_distribution<float> noise_;
    std::uniform_int_distribution<int> target_pos_;
    int num_targets_;
    float target_amplitude_;
};
