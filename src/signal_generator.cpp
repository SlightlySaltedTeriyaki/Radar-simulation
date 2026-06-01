#include "signal_generator.hpp"
#include <cmath>

static constexpr float TWO_PI    = 6.28318530718f;
static constexpr float PI        = 3.14159265359f;
static constexpr float MIN_RANGE = 40.0f;
static constexpr float MAX_RANGE = RANGE_BINS - 40.0f;

SignalGenerator::SignalGenerator(uint32_t seed, int num_targets, float noise_sigma)
    : rng_(seed)
    , noise_(0.0f, noise_sigma)
    , target_amplitude_(30.0f)
{
    std::uniform_real_distribution<float> r_dist(MIN_RANGE * 2.0f, MAX_RANGE);
    std::uniform_real_distribution<float> az_dist(0.0f, TWO_PI);

    targets_.resize(num_targets);
    for (auto& t : targets_) {
        respawn(t);
        // Spread initial positions so targets appear immediately rather than all at the far edge.
        float r  = r_dist(rng_);
        float az = az_dist(rng_);
        t.x = r * std::sin(az);
        t.y = r * std::cos(az);
    }
}

void SignalGenerator::respawn(SimTarget& t) {
    std::uniform_real_distribution<float> az_dist(0.0f, TWO_PI);
    std::uniform_real_distribution<float> speed_dist(0.06f, 0.18f);
    std::uniform_real_distribution<float> dop_dist(1.5f, 4.0f);
    std::uniform_real_distribution<float> dev_dist(-0.4f, 0.4f);
    std::uniform_real_distribution<float> turn_dist(0.001f, 0.003f);
    std::uniform_int_distribution<int>    timer_dist(240, 720);
    std::bernoulli_distribution           coin(0.5);

    float az = az_dist(rng_);
    t.x = MAX_RANGE * std::sin(az);
    t.y = MAX_RANGE * std::cos(az);

    t.heading    = az + PI + dev_dist(rng_);  // roughly inward
    t.speed      = speed_dist(rng_);
    t.turn_rate  = 0.0f;
    t.maneuvering = coin(rng_);

    if (t.maneuvering) {
        float mag          = turn_dist(rng_);
        t.turn_rate_target = coin(rng_) ? mag : -mag;
    } else {
        t.turn_rate_target = 0.0f;
    }

    t.behavior_timer = timer_dist(rng_);
    // vr_doppler is independent of spatial velocity: slow spatial motion keeps targets
    // visible across many sweeps while a realistic Doppler shift drives the RD map.
    t.vr_doppler = dop_dist(rng_) * (coin(rng_) ? 1.0f : -1.0f);
}

bool SignalGenerator::advance_and_check(SimTarget& t, float angle, float beamwidth) {
    if (t.maneuvering) {
        // Smooth first-order convergence toward desired turn rate — avoids abrupt direction snaps.
        t.turn_rate += (t.turn_rate_target - t.turn_rate) * 0.008f;

        if (--t.behavior_timer <= 0) {
            std::uniform_real_distribution<float> turn_dist(0.001f, 0.003f);
            std::uniform_int_distribution<int>    timer_dist(240, 720);
            std::uniform_real_distribution<float> chance(0.0f, 1.0f);
            std::bernoulli_distribution           coin(0.5);

            if (chance(rng_) < 0.4f) {
                t.turn_rate_target = 0.0f;   // fly straight for a while
            } else {
                float mag          = turn_dist(rng_);
                t.turn_rate_target = coin(rng_) ? mag : -mag;
            }
            t.behavior_timer = timer_dist(rng_);
        }
    }

    t.heading += t.turn_rate;
    t.x       += t.speed * std::sin(t.heading);
    t.y       += t.speed * std::cos(t.heading);

    float r = t.range();
    if (r < MIN_RANGE || r > MAX_RANGE) {
        respawn(t);
        return false;
    }

    if (angle >= 0.0f) {
        float az   = t.azimuth();
        float diff = az - angle;
        if (diff >  PI) diff -= TWO_PI;
        if (diff < -PI) diff += TWO_PI;
        if (std::abs(diff) > beamwidth * 0.5f)
            return false;
    }
    return true;
}

Sweep SignalGenerator::generate(float angle, float beamwidth) {
    Sweep sweep;
    for (int i = 0; i < RANGE_BINS; ++i)
        sweep[i] = noise_(rng_);

    for (auto& t : targets_) {
        if (!advance_and_check(t, angle, beamwidth)) continue;

        float r         = t.range();
        float amplitude = target_amplitude_ * 256.0f / (r + 256.0f);
        int   center    = static_cast<int>(r);
        for (int k = -2; k <= 2; ++k) {
            int bin = center + k;
            if (bin >= 0 && bin < RANGE_BINS)
                sweep[bin] += amplitude * std::exp(-0.5f * float(k * k));
        }
    }
    return sweep;
}

CPI SignalGenerator::generate_cpi(float angle, float beamwidth) {
    CPI cpi;
    for (int n = 0; n < NUM_PULSES; ++n)
        for (int i = 0; i < RANGE_BINS; ++i)
            cpi[n][i] = noise_(rng_);

    for (auto& t : targets_) {
        if (!advance_and_check(t, angle, beamwidth)) continue;

        float r         = t.range();
        float amplitude = target_amplitude_ * 256.0f / (r + 256.0f);
        int   center    = static_cast<int>(r);
        // vr_doppler is used here rather than the spatial radial velocity so that
        // the Doppler signature is independent of (slow) PPI motion.
        float fd = t.vr_doppler / (2.0f * MAX_VELOCITY);

        for (int n = 0; n < NUM_PULSES; ++n) {
            float phase = TWO_PI * fd * float(n);
            float scale = amplitude * std::cos(phase);

            for (int k = -2; k <= 2; ++k) {
                int bin = center + k;
                if (bin >= 0 && bin < RANGE_BINS)
                    cpi[n][bin] += scale * std::exp(-0.5f * float(k * k));
            }
        }
    }
    return cpi;
}
