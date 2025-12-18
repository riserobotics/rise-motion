#pragma once
#include <atomic>

template <typename T> struct Snapshot {
  std::atomic<unsigned int> seq;
  T *data;

  void read(T &dst) {
    unsigned s1, s2;
    do {
      s1 = seq.load(std::memory_order_acquire);
      if (s1 & 1)
        continue; // writer active

      memcpy(&dst, data, sizeof(T));

      s2 = seq.load(std::memory_order_acquire);
    } while (s1 != s2);
  }

  void write(T &src) {
    // mark write start (odd)
    seq.fetch_add(1, std::memory_order_release);

    // prepare data
    memcpy(data, &src, sizeof(T));

    // publish (even)
    seq.fetch_add(1, std::memory_order_release);
  }
};
