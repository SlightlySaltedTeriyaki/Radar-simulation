#pragma once
#include <array>
#include <cmath>
#include <random>
#include <vector>
#include <cstdint>

static constexpr int   RANGE_BINS   = 1024;
static constexpr int   NUM_PULSES   = 32;    // pulses per CPI (must be power of 2)
static constexpr float MAX_VELOCITY = 5.0f;  // bins/CPI — sets Doppler ambiguity limit

using Sweep = std::array<float, RANGE_BINS>;
using CPI   = std::array<Sweep, NUM_PULSES>;

struct SimTarget {
    float x, y;              // Cartesian position (bins from radar origin; x=East, y=North)
    float speed;             // magnitude of velocity (bins/frame), kept constant
    float heading;           // direction of travel (radians, 0=North CW)
    float turn_rate;         // current rate of heading change (rad/frame)
    float turn_rate_target;  // desired turn rate; turn_rate converges toward this smoothly
    int   behavior_timer;    // frames until the next turn_rate_target update
    bool  maneuvering;       // false = straight-line target; true = makes gradual turns
    float vr_doppler;        // radial velocity for Doppler phase (bins/CPI)

    float range()   const { return std::sqrt(x * x + y * y); }
    // Returns azimuth in (-π, π]: 0=North, clockwise positive.
    // Uses atan2(x, y) — not the usual atan2(y, x) — because x maps to East and y to North.
    float azimuth() const { return std::atan2(x, y); }
};

class SignalGenerator {
public:
    explicit SignalGenerator(uint32_t seed, int num_targets = 1, float noise_sigma = 1.0f);

    // Single sweep — used by the benchmark. angle=-1 bypasses beam gating.
    Sweep generate(float angle = -1.0f, float beamwidth = 0.087f);

    // Full CPI — used by the GUI for Range-Doppler processing.
    CPI generate_cpi(float angle = -1.0f, float beamwidth = 0.087f);

private:
    std::mt19937 rng_;
    std::normal_distribution<float> noise_;
    std::vector<SimTarget>          targets_;
    float                           target_amplitude_;

    // Place target at the radar edge with a fresh flight profile.
    void respawn(SimTarget& t);

    // Advance target one step. Returns false if not in beam or just respawned.
    bool advance_and_check(SimTarget& t, float angle, float beamwidth);
};
