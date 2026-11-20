#pragma once

#include <atomic>
#include <cstring>
#include <memory>


template <typename T>
class APSA {
public:
  APSA()
    : c_mem(std::make_unique<T>()),    // Allocate communication buffer
      a_mem(std::make_unique<T>()),    // Allocate atomic/shared buffer
      p_mem(std::make_unique<T>()),    // Allocate performance buffer
      c_p(nullptr),                     // Communication pointer starts null
      a_p(nullptr),                     // Atomic pointer starts null
      p_p(nullptr)                      // Performance pointer starts null
  {
    // No initialization needed - pointers start null as per APSA spec
  }

  bool comm_write(const T& data) {
    // Copy new data into our communication buffer
    *c_mem = data;

    // Point c_p to the buffer we just wrote
    c_p.store(c_mem.get(), std::memory_order_release);

    T* desired = c_p.load(std::memory_order_acquire);

    // Perform the atomic swap
    T* old_a_p = a_p.exchange(desired, std::memory_order_acq_rel);

    // Update c_p to point to what a_p was pointing to
    c_p.store(old_a_p, std::memory_order_release);

    return true;
  }


  bool perf_read(T& data) {
    // Check if new data is available in atomic pointer
    // memory_order_acquire ensures we see the write from comm_write
    T* a_ptr = a_p.load(std::memory_order_acquire);

    if (a_ptr != nullptr) {
      // New data is available! Swap p_p with a_p to claim it
      T* old_p_p = p_p.exchange(a_p.load(std::memory_order_acquire),
                                 std::memory_order_acq_rel);

      // Update a_p atomically
      a_p.store(old_p_p, std::memory_order_release);

      // Copy the data from our performance buffer
      // p_p now points to the buffer with fresh data
      T* p_ptr = p_p.load(std::memory_order_acquire);
      if (p_ptr != nullptr) {
        data = *p_ptr;
        
        // This tells the communication thread we're done with this data
        p_p.store(nullptr, std::memory_order_release);

        return true;  // New data was read
      }
    }

    return false;  // No new data available
  }


  bool perf_write(const T& data) {
    // Copy data into our performance buffer
    *p_mem = data;

    // Point p_p to the buffer we just wrote
    p_p.store(p_mem.get(), std::memory_order_release);

    // Atomically swap p_p with a_p
    T* old_a_p = a_p.exchange(p_p.load(std::memory_order_acquire),
                              std::memory_order_acq_rel);

    // Update p_p to point to what a_p was pointing to
    p_p.store(old_a_p, std::memory_order_release);

    return true;
  }


  bool comm_read(T& data) {
    // Check if new data is available in atomic pointer
    T* a_ptr = a_p.load(std::memory_order_acquire);

    if (a_ptr != nullptr) {
      // New data is available! Swap c_p with a_p to claim it
      T* old_c_p = c_p.exchange(a_p.load(std::memory_order_acquire),
                                std::memory_order_acq_rel);

      // Update a_p atomically
      a_p.store(old_c_p, std::memory_order_release);

      // Copy the data from our communication buffer
      T* c_ptr = c_p.load(std::memory_order_acquire);
      if (c_ptr != nullptr) {
        data = *c_ptr;

        c_p.store(nullptr, std::memory_order_release);

        return true;  // New data was read
      }
    }

    return false;  // No new data available
  }

private:
  std::unique_ptr<T> c_mem;  // Communication thread's buffer (ROS side)
  std::unique_ptr<T> a_mem;  // Atomic/shared buffer (exchange point)
  std::unique_ptr<T> p_mem;  // Performance thread's buffer (EtherCAT side)

  std::atomic<T*> c_p;  // Communication pointer (ROS side access)
  std::atomic<T*> a_p;  // Atomic pointer (shared exchange point)
  std::atomic<T*> p_p;  // Performance pointer (EtherCAT side access)
};
