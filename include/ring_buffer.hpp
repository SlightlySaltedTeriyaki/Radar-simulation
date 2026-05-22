#pragma once
#include <cstddef>

// FÁZE 1: Template ring buffer s compile-time velikostí (žádná heap alokace).
// N musí být mocnina 2 — proč? (hint: bitový AND místo modulo)
template <typename T, std::size_t N>
class RingBuffer {
public:
    RingBuffer() = default;

    // Vrátí false pokud je buffer plný (data jsou zahozena).
    bool push(const T& value);

    // Vrátí false pokud je buffer prázdný.
    bool pop(T& out);

    bool empty() const;
    bool full() const;
    std::size_t size() const;

private:
    T data_[N]{};
    std::size_t head_ = 0;
    std::size_t tail_ = 0;
    std::size_t count_ = 0;
};

// --- implementace (v .hpp protože je to template) ---

template <typename T, std::size_t N>
bool RingBuffer<T, N>::push(const T& value) {
    if (full()) return false;
    // TODO: ulož value do data_[head_], posuň head_, inkrementuj count_
    data_[head_] = value;
    head_ = (head_ + 1) & (N - 1);
    ++count_;
    return true;
}

template <typename T, std::size_t N>
bool RingBuffer<T, N>::pop(T& out) {
    if (empty()) return false;
    // TODO: přečti data_[tail_], posuň tail_, dekrementuj count_
    out = data_[tail_];
    tail_ = (tail_ + 1) & (N - 1);
    --count_;
    return true;
}

template <typename T, std::size_t N>
bool RingBuffer<T, N>::empty() const { return count_ == 0; }

template <typename T, std::size_t N>
bool RingBuffer<T, N>::full() const { return count_ == N; }

template <typename T, std::size_t N>
std::size_t RingBuffer<T, N>::size() const { return count_; }
