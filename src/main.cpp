#include <chrono>
#include <cstdio>
#include "ring_buffer.hpp"
#include "signal_generator.hpp"
#include "signal_processor.hpp"
#include "target_tracker.hpp"

static constexpr int NUM_SWEEPS = 10'000;

int main() {
    SignalGenerator generator(/*seed=*/42);
    RingBuffer<Sweep, 8> buffer;         // 8 sweepů ve frontě
    SignalProcessor cfar(/*guard=*/2, /*reference=*/16, /*threshold=*/5.0f);
    TargetTracker tracker(/*max_delta=*/5.0f, /*miss_threshold=*/3);

    auto t_start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < NUM_SWEEPS; ++i) {
        Sweep sweep = generator.generate();

        buffer.push(sweep);

        Sweep current;
        if (!buffer.pop(current)) continue;

        auto detections = cfar.process(current);
        tracker.update(detections);
    }

    auto t_end = std::chrono::high_resolution_clock::now();
    auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();

    std::printf("Sweepy:      %d\n", NUM_SWEEPS);
    std::printf("Cas celkem:  %ld us\n", elapsed_us);
    std::printf("Throughput:  %.0f sweepy/s\n", NUM_SWEEPS * 1e6 / elapsed_us);
    std::printf("Latence:     %.1f us/sweep\n", (double)elapsed_us / NUM_SWEEPS);
    std::printf("Aktivni tracky: %zu\n", tracker.tracks().size());

    return 0;
}
