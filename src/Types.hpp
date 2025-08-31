#pragma once
#include <cstdint>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>

// A single polled sample from controllers
struct Sample {
    uint64_t timestamp_ms;
    uint64_t button_mask;   // arbitrary bitmask (user mapping)
    double left_x, left_y;  // [-1..1]
    double right_x, right_y;// [-1..1]
    double l2, r2;          // [0..1]
    // Motion
    double gyro_x_dps, gyro_y_dps, gyro_z_dps; // degrees/sec
    double accel_x_g, accel_y_g, accel_z_g;    // g
};

template<typename T>
class ThreadQueue {
    std::mutex m_;
    std::condition_variable cv_;
    std::deque<T> q_;
    std::atomic<bool> closed_{false};
public:
    void push(T item) {
        {
            std::lock_guard<std::mutex> lk(m_);
            q_.push_back(std::move(item));
        }
        cv_.notify_one();
    }
    bool wait_pop(T &out) {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait(lk, [&]{ return !q_.empty() || closed_.load(); });
        if (q_.empty()) return false;
        out = std::move(q_.front());
        q_.pop_front();
        return true;
    }
    void close() { closed_.store(true); cv_.notify_all(); }
    bool closed() const { return closed_.load(); }
};
