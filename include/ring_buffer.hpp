#pragma once
#include <atomic>
#include <cstddef>

// Lock-free SPSC (single-producer, single-consumer) ring buffer with compile-time capacity.
// N must be a power of two so the index wrap can use bitwise AND instead of modulo.
template <typename T, std::size_t N>
class RingBuffer {
    static_assert((N & (N - 1)) == 0 && N > 0, "RingBuffer size N must be a power of two");
public:
    RingBuffer() = default;

    bool push(const T& value);
    bool pop(T& out);

    bool empty() const;
    bool full() const;
    std::size_t size() const;

private:
    T data_[N]{};
    std::atomic<std::size_t> head_{0};  // written only by producer
    std::atomic<std::size_t> tail_{0};  // written only by consumer
};

template <typename T, std::size_t N>
bool RingBuffer<T, N>::push(const T& value) {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t next_head = (head + 1) & (N - 1);
    if (next_head == tail_.load(std::memory_order_acquire))
        return false;
    data_[head] = value;
    head_.store(next_head, std::memory_order_release);
    return true;
}

template <typename T, std::size_t N>
bool RingBuffer<T, N>::pop(T& out) {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire))
        return false;
    out = data_[tail];
    tail_.store((tail + 1) & (N - 1), std::memory_order_release);
    return true;
}

template <typename T, std::size_t N>
bool RingBuffer<T, N>::empty() const {
    return tail_.load(std::memory_order_acquire) == head_.load(std::memory_order_acquire);
}

template <typename T, std::size_t N>
bool RingBuffer<T, N>::full() const {
    return ((head_.load(std::memory_order_acquire) + 1) & (N - 1)) == tail_.load(std::memory_order_acquire);
}

template <typename T, std::size_t N>
std::size_t RingBuffer<T, N>::size() const {
    const std::size_t head = head_.load(std::memory_order_acquire);
    const std::size_t tail = tail_.load(std::memory_order_acquire);
    return (head - tail) & (N - 1);
}
