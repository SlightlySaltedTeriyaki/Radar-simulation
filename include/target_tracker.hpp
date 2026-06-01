#pragma once
#include "signal_processor.hpp"
#include <vector>
#include <cstdint>

struct Track {
    uint32_t id;
    float range_bin;       // last measured range (EMA-smoothed)
    float azimuth;         // last-detected bearing (radians, 0=North CW)
    float velocity_est;    // signed range rate (bins/frame); used to predict range between sweeps
    int   age;
    int   missed_frames;
    int   frame_last_seen; // frame index when the track was last matched to a detection
};

class TargetTracker {
public:
    explicit TargetTracker(float max_range_delta = 5.0f, int miss_threshold = 3);

    // current_frame: monotonically increasing frame counter used for range prediction.
    // sweep_angle + beamwidth: beam-gated miss counting (only count a miss when the beam
    //   was actually pointing near the track). Pass beamwidth = TWO_PI to always count.
    void update(const std::vector<Detection>& detections,
                float sweep_angle   = 0.0f,
                float beamwidth     = 6.28318530718f,
                int   current_frame = 0);

    const std::vector<Track>& tracks() const { return tracks_; }

private:
    float    max_range_delta_;
    int      miss_threshold_;
    uint32_t next_id_ = 1;
    std::vector<Track> tracks_;
};
