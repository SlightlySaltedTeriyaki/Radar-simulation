#include "target_tracker.hpp"
#include <algorithm>
#include <cmath>

TargetTracker::TargetTracker(float max_range_delta, int miss_threshold)
    : max_range_delta_(max_range_delta)
    , miss_threshold_(miss_threshold)
{}

void TargetTracker::update(const std::vector<Detection>& detections) {
    std::vector<bool> matched(detections.size(), false);

    for (Track& track : tracks_) {
        int best_idx = -1;
        float best_delta = max_range_delta_;

        for (int i = 0; i < static_cast<int>(detections.size()); ++i) {
            if (matched[i]) continue;
            float delta = std::fabs(static_cast<float>(detections[i].range_bin) - track.range_bin);
            if (delta < best_delta) {
                best_delta = delta;
                best_idx = i;
            }
        }

        if (best_idx >= 0) {
            matched[best_idx] = true;
            constexpr float alpha = 0.3f;  // EMA váha nového měření vs. historie
            track.range_bin = alpha * static_cast<float>(detections[best_idx].range_bin)
                            + (1.0f - alpha) * track.range_bin;
            ++track.age;
            track.missed_frames = 0;
        } else {
            ++track.missed_frames;
        }
    }

    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
        [&](const Track& t) { return t.missed_frames >= miss_threshold_; }),
        tracks_.end());

    for (int i = 0; i < static_cast<int>(detections.size()); ++i) {
        if (!matched[i])
            tracks_.push_back({next_id_++, static_cast<float>(detections[i].range_bin), 1, 0});
    }
}
