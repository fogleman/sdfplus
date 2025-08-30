#pragma once

std::function<void()> timed(const std::string &message) {
    const auto startTime = std::chrono::steady_clock::now();
    fprintf(stderr, "%s... ", message.c_str());
    fflush(stderr);
    return [message, startTime]() {
        const std::chrono::duration<double> elapsed =
            std::chrono::steady_clock::now() - startTime;
        const double seconds = elapsed.count();
        fprintf(stderr, "%fs\n", seconds);
    };
}

using WorkerFunc = std::function<void(const int, const int)>;

void RunWorkers(
    const WorkerFunc workerFunc,
    const int numWorkers = std::thread::hardware_concurrency())
{
    boost::asio::thread_pool pool(numWorkers);
    for (int i = 0; i < numWorkers; i++) {
        boost::asio::post(pool, [&, i] {
            workerFunc(i, numWorkers);
        });
    }
    pool.join();
}
