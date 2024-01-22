#pragma once

#include <Arduino.h>
#include <semphr.h>

class Perf {
public:
    typedef unsigned long utime_t;
    Perf(int num_sampling=100, utime_t timeout_ms=200) : 
        _max_num_sampling(num_sampling), _timeout(timeout_ms), 
        _samples_idx(0), _fps(0)
    {
        _last_update_time = millis();
        _last_recv_time = millis();
        _mutex = xSemaphoreCreateMutex();
    }

    void feed_data(const utime_t receive_time)
    {
        const utime_t now = millis();
        if (xSemaphoreTake(_mutex, 0) == pdTRUE) {
            if (_samples_idx == _max_num_sampling - 1){
                _fps = (float)_max_num_sampling / (now - _last_recv_time);
                _last_recv_time = now;
            }
            _last_update_time = now;
            _samples_idx = (_samples_idx + 1) % _max_num_sampling;
            xSemaphoreGive(_mutex);
        }
    };

    bool is_timeout() const {
        const utime_t now = millis(); 
        if (now < _last_update_time)
            return (0xFFFFFFFF - _last_update_time + now);
        return (now - _last_update_time > _timeout);
    };

    float get_fps() const {
        if ( is_timeout() )
            return -1;
        return 1e3 * _fps;
    };

    ~Perf() {};

private:
    int _samples_idx;
    int _max_num_sampling;
    double _fps;
    SemaphoreHandle_t _mutex{nullptr};

    utime_t _last_update_time;
    utime_t _last_recv_time;
    utime_t _timeout;            // in ms
};
