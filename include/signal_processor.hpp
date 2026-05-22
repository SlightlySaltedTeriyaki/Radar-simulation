#pragma once
#include "signal_generator.hpp"
#include <vector>
#include <cstdint>

struct Detection {
    uint16_t range_bin;
    float amplitude;
};

// FÁZE 3: CA-CFAR (Cell-Averaging Constant False Alarm Rate) detektor.
// Klasický radarový algoritmus — detekuje cíle adaptivním prahem nad lokálním šumem.
class SignalProcessor {
public:
    SignalProcessor(int guard_cells, int reference_cells, float threshold_factor);

    // Zpracuje jeden sweep, vrátí seznam detekcí.
    // POZOR: nesmí alokovat paměť uvnitř této funkce (pre-alokuj v konstruktoru).
    std::vector<Detection> process(const Sweep& sweep);

private:
    int guard_;
    int ref_;
    float threshold_factor_;

    std::vector<Detection> result_buffer_;  // pre-alokovaný výstupní buffer
};
