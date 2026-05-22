#include "signal_generator.hpp"

SignalGenerator::SignalGenerator(uint32_t seed, int num_targets, float noise_sigma)
    : rng_(seed)
    , noise_(0.0f, noise_sigma)
    , target_pos_(50, RANGE_BINS - 50)
    , target_amplitude_(10.0f)
    , num_targets_(num_targets)
{}

Sweep SignalGenerator::generate() {
    Sweep sweep;

    // TODO (Fáze 2): naplň sweep Gaussovým šumem, pak na náhodnou pozici
    // přidej cíl s amplitudou target_amplitude_
    // Příklad: sweep[i] = noise_(rng_);

    for (int i = 0; i < RANGE_BINS; ++i) {
        sweep[i] = noise_(rng_);
    }
    
    for (int i = 0; i < num_targets_; ++i) {
        sweep[target_pos_(rng_)] += target_amplitude_;
    }

    return sweep;  // move semantics — kopie se nevytvoří
}
