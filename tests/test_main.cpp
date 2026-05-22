#include <cassert>
#include <cstdio>
#include "../include/ring_buffer.hpp"
#include "../include/signal_generator.hpp"
#include "../include/signal_processor.hpp"
#include "../include/target_tracker.hpp"

void test_cfar_detects_strong_target() {
    SignalProcessor cfar(2, 16, 5.0f);
    Sweep sweep{};  // all zeros

    sweep[512] = 100.0f;

    auto detections = cfar.process(sweep);
    assert(detections.size() == 1);
    assert(detections[0].range_bin == 512);
}

void test_cfar_ignores_pure_noise() {
    SignalProcessor cfar(2, 16, 5.0f);
    Sweep sweep{};  // all zeros, no target, no noise

    auto detections = cfar.process(sweep);
    assert(detections.empty());
}

void test_tracker_drops_missed_track() {
    SignalProcessor cfar(2, 16, 5.0f);
    TargetTracker tracker(5.0f, 3);

    // create a track by detecting a strong target for a few frames
    for (int i = 0; i < 5; ++i) {
        Sweep sweep{};
        sweep[512] = 100.0f;
        auto detections = cfar.process(sweep);
        tracker.update(detections);
    }

    assert(!tracker.tracks().empty());

    // send empty detections — track should disappear after miss_threshold misses
    for (int i = 0; i < 3; ++i) {
        tracker.update({});
    }

    assert(tracker.tracks().empty());
}

void test_ringbuffer_wraparound() {
    RingBuffer<int, 8> buffer;

    for (int i = 0; i < 8; ++i)
        assert(buffer.push(i));

    assert(buffer.full());

    for (int i = 0; i < 8; ++i) {
        int val;
        assert(buffer.pop(val));
        assert(val == i);
    }

    assert(buffer.empty());

    for (int i = 0; i < 8; ++i)
        assert(buffer.push(i + 10));

    assert(buffer.full());
}

int main() {
    test_cfar_detects_strong_target();
    test_cfar_ignores_pure_noise();
    test_tracker_drops_missed_track();
    test_ringbuffer_wraparound();

    std::printf("All tests passed.\n");
    return 0;
}
