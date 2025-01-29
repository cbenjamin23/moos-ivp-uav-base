#pragma once

#include <mutex>
#include <shared_mutex>
#include <utility>

template <typename T>
class ThreadSafeVariable {
public:
    explicit ThreadSafeVariable(T initial_value = T()) : value_(std::move(initial_value)) {}

    // Get a copy
    T get() const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        return value_;
    }
    

    void set(const T& new_value) {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        value_ = new_value;
    }



    // Copy constructor
    ThreadSafeVariable(const ThreadSafeVariable& other) {
        std::unique_lock<std::shared_mutex> lock(other.mutex_);
        value_ = other.value_;
    }

    // Copy assignment operator
    ThreadSafeVariable& operator=(const ThreadSafeVariable& other) {
        if (this != &other) {
            std::lock(mutex_, other.mutex_);
            std::unique_lock<std::shared_mutex> self_lock(mutex_, std::adopt_lock);
            std::unique_lock<std::shared_mutex> other_lock(other.mutex_, std::adopt_lock);
            value_ = other.value_;
        }
        return *this;
    }

    ThreadSafeVariable& operator=(const T& new_value) {
        set(new_value);
        return *this;
    }

     // Implicit conversion to T for compatibility with logical operators
    operator T() const {
        return get();
    }


    // If bool
    template <typename U = T, typename = typename std::enable_if<std::is_same<U, bool>::value>::type>
    bool operator!() const {
        return !get();
    }


    // Proxy class for thread-safe member function access
    class Proxy {
    public:
        Proxy(T& value, std::shared_mutex& mutex) : value_(value), lock_(mutex) {}
        T* operator->() { return &value_; }

    private:
        T& value_;
        std::unique_lock<std::shared_mutex> lock_; // Automatically locks/unlocks the mutex
    };

    // Provide access to a proxy object
    Proxy operator->() {
        return Proxy(value_, mutex_);
    }

private:
    mutable std::shared_mutex mutex_;
    T value_;
};
