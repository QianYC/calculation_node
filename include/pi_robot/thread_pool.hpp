//
// Created by yc_qian on 20-4-16.
//

#ifndef PI_ROBOT_THREAD_POOL_HPP
#define PI_ROBOT_THREAD_POOL_HPP

#include <mutex>
#include <functional>
#include <queue>
#include <thread>
#include <condition_variable>

class ThreadPool {
public:
    ThreadPool(int n) {
        for (int i = 0; i < n; ++i) {
            std::thread worker([&](){
                std::unique_lock<std::mutex> lk(mtx);
                while (true) {
                    if (shutdown) {
                        break;
                    } else if (!tasks.empty()) {
                        auto f = tasks.front();
                        tasks.pop();
                        lk.unlock();
                        f();
                        lk.lock();
                    } else {
                        cv.wait(lk);
                    }
                }
            });
            worker.detach();
        }
    }

    void submit(std::function<void()> &&f) {
        {
            std::lock_guard<std::mutex> lk(mtx);
            tasks.push(f);
        }
        cv.notify_one();
    }

    ~ThreadPool() {
        {
            std::lock_guard<std::mutex> lk(mtx);
            shutdown = true;
        }
        cv.notify_all();
    }
private:
    std::mutex mtx;
    std::condition_variable cv;
    std::queue<std::function<void()>> tasks;
    bool shutdown = false;
};
#endif //PI_ROBOT_THREAD_POOL_HPP
