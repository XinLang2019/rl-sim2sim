#include "loop.hpp"
#include <condition_variable>
#include <stdexcept>

int main() {
    std::mutex mutex;
    std::condition_variable changed;
    int count = 0;
    {
        LoopFunc loop("lifecycle", .002, [&] {
            std::lock_guard<std::mutex> guard(mutex);
            ++count;
            changed.notify_one();
        });
        loop.start();
        {
            std::unique_lock<std::mutex> lock(mutex);
            if (!changed.wait_for(lock, std::chrono::seconds(2), [&] { return count > 0; })) return 1;
        }
        bool rejected = false;
        try { loop.start(); } catch (const std::logic_error&) { rejected = true; }
        if (!rejected) return 2;
        loop.shutdown();
        int stopped = count;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (count != stopped) return 3;
        loop.shutdown(); // Idempotent.
        count = 0;
        loop.start();
        std::unique_lock<std::mutex> lock(mutex);
        if (!changed.wait_for(lock, std::chrono::seconds(2), [&] { return count > 0; })) return 4;
        // Destruction must stop and join the worker before captured state dies.
    }
    return 0;
}
