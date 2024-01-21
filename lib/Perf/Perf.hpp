#pragma once

#include <Arduino.h>

class Perf {
public:
    Perf(int num_sampling=100, time_t timeout_ms=100) : 
        _max_num_sampling(num_sampling), _timeout(timeout_ms), 
        _samples_idx(0), _fps(0)
    {
        _last_update_time = micros();
    }

    void feed_data(uint32_t receive_time)
    {
        if (_samples_idx == _max_num_sampling - 1){
            _fps = (double)_max_num_sampling / (receive_time - _last_update_time);
            _last_update_time = receive_time;
        }
        _samples_idx = (_samples_idx + 1) % _max_num_sampling;
    };

    bool is_timeout() const {
        return (micros() - _last_update_time > _timeout * 1e3);
    };

    double get_fps() const {
        if ( is_timeout() )
            return -1;
        return 1e6 * _fps;
    };

    ~Perf() {};

private:
    int _samples_idx;
    int _max_num_sampling;
    double _fps;

    time_t _last_update_time;
    time_t _timeout;            // in ms
};
