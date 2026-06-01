#include "target_tracker.hpp"
#include <algorithm>
#include <cmath>

static constexpr float PI = 3.14159265359f;

TargetTracker::TargetTracker(float max_range_delta, int miss_threshold)
    : max_range_delta_(max_range_delta)
    , miss_threshold_(miss_threshold)
{}

void TargetTracker::update(const std::vector<Detection>& detections,
                           float sweep_angle, float beamwidth, int current_frame)
{
    std::vector<bool> matched(detections.size(), false);

    for (Track& track : tracks_) {
        // Predict where this track should be now, based on elapsed frames and velocity.
        int   elapsed         = std::max(current_frame - track.frame_last_seen, 1);
        float predicted_range = track.range_bin + track.velocity_est * float(elapsed);

        int   best_idx   = -1;
        float best_delta = max_range_delta_;

        for (int i = 0; i < static_cast<int>(detections.size()); ++i) {
            if (matched[i]) continue;
            float delta = std::fabs(float(detections[i].range_bin) - predicted_range);
            if (delta < best_delta) { best_delta = delta; best_idx = i; }
        }

        if (best_idx >= 0) {
            matched[best_idx] = true;
            float new_range = float(detections[best_idx].range_bin);

            // velocity_est in bins/frame: how much range changed over the elapsed period
            constexpr float alpha = 0.3f;
            float range_rate = (new_range - track.range_bin) / float(elapsed);
            track.velocity_est    = alpha * range_rate + (1.0f - alpha) * track.velocity_est;
            track.range_bin       = new_range;
            track.azimuth         = sweep_angle;
            track.frame_last_seen = current_frame;
            ++track.age;
            track.missed_frames = 0;
        } else {
            // Count a miss only when the beam was actually near this track's azimuth.
            // With beamwidth = TWO_PI (default) every frame counts — preserves benchmark behaviour.
            float diff = sweep_angle - track.azimuth;
            if (diff >  PI) diff -= PI * 2.0f;
            if (diff < -PI) diff += PI * 2.0f;
            if (std::abs(diff) < beamwidth) {
                ++track.missed_frames;
            }
        }
    }

    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
        [&](const Track& t) { return t.missed_frames >= miss_threshold_; }),
        tracks_.end());

    for (int i = 0; i < static_cast<int>(detections.size()); ++i) {
        if (!matched[i])
            tracks_.push_back({next_id_++,
                               float(detections[i].range_bin),
                               sweep_angle,
                               0.0f,           // velocity_est starts neutral
                               1, 0,
                               current_frame});
    }
}
