#pragma once
#include "signal_processor.hpp"
#include <vector>
#include <cstdint>

struct Track {
    uint32_t id;
    float range_bin;    // průměrovaná poloha
    int age;            // kolik sweepů track existuje
    int missed_frames;  // kolik sweepů po sobě chyběl
};

// FÁZE 4: Jednoduchý tracker — asociuje detekce k existujícím stopám.
class TargetTracker {
public:
    explicit TargetTracker(float max_range_delta = 5.0f, int miss_threshold = 3);

    // Zpracuje detekce z jednoho sweepů, aktualizuje stopy.
    void update(const std::vector<Detection>& detections);

    const std::vector<Track>& tracks() const { return tracks_; }

private:
    float max_range_delta_;
    int miss_threshold_;
    uint32_t next_id_ = 1;
    std::vector<Track> tracks_;
};
