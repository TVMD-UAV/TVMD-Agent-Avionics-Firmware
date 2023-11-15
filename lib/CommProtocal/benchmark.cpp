# include "benchmark.h"


Benchmark::Benchmark(int num_sampling){
    _samples_idx = 0;
    _max_num_sampling = num_sampling;
    // _durations = (int*) malloc(num_sampling * sizeof(int));
    _durations = 0;
    _last_update_time = micros();
}

Benchmark::~Benchmark(){
    // free(_durations);
}

void Benchmark::feed_data(int pid, uint32_t send_time, uint32_t receive_time){
    _durations += receive_time - send_time;
    
    if (pid != _last_pid + 1) _num_packet_lost += 1;

    if (_samples_idx == _max_num_sampling-1){
        // buffer full
        _latency = (double)_durations / _max_num_sampling;
        _fps = (double)_max_num_sampling / (receive_time - _last_update_time);

        // reset states
        _last_update_time = receive_time;
        _durations = 0;
        _available = true;
    }
    
    _samples_idx = (_samples_idx + 1) % _max_num_sampling;
    _last_pid = pid;
}

//TODO: Use semaphore to protect inner states read/write
void Benchmark::feed_data(uint32_t receive_time){
    if (_samples_idx == _max_num_sampling-1){
        _fps = (double)_max_num_sampling / (receive_time - _last_update_time);
        _last_update_time = receive_time;
        _available = true;
    }
    _samples_idx = (_samples_idx + 1) % _max_num_sampling;
}