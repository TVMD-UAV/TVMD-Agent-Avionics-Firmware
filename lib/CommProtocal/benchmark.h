# ifndef BENCHMARK
# define BENCHMARK

# include <Arduino.h>

class Benchmark {
public:
    Benchmark(int num_sampling=100);

    void feed_data(int pid, uint32_t send_time, uint32_t receive_time);

    double get_fps() {return 1e6 * _fps;};

    double get_latency() {return 1e-6 * (double)_latency;};

    int packet_lost() {return _num_packet_lost;};

    ~Benchmark();

private:
    int _samples_idx;
    int _max_num_sampling;
    int _durations;
    double _fps;
    int _latency;
    bool _available;
    int _num_packet_lost;
    int _last_pid;

    uint32_t _last_update_time;
};


# endif
