#pragma once

#include <Arduino.h>
#include <semphr.h>

class Perf {
public:
    typedef unsigned long utime_t;

    /* 
     * @param num_sampling: Number of samples to calculate FPS
     * @param timeout_ms: Timeout in ms
     * @param dlpf: Digital low pass filter for timeout (counting)
     */
    Perf(int num_sampling=100, utime_t timeout_ms=200, size_t dlpf=3) : 
        _max_num_sampling(num_sampling), _timeout(timeout_ms), 
        _samples_idx(0), _fps(0), _dlpf(dlpf), _dlpf_counter(0)
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

    bool is_timeout() {
        const utime_t now = millis(); 
        const bool now_timeout = (now < _last_update_time) ? 
            ( (0xFFFFFFFF - _last_update_time + now) > _timeout ) : 
            (                now - _last_update_time > _timeout );
        
        if (now_timeout) {
            _dlpf_counter++;
        } else {
            _dlpf_counter = 0;
        }

        return _dlpf_counter >= _dlpf;
    };

    float get_fps() {
        if ( is_timeout() )
            return -1;
        return 1e3 * _fps;
    };

    ~Perf() {};

private:
    int _max_num_sampling;
    int _samples_idx;
    size_t _dlpf;               // Digital low pass filter for timeout
    size_t _dlpf_counter;
    float _fps;                 // Frames per second in Hz
    SemaphoreHandle_t _mutex{nullptr};

    utime_t _last_update_time;  // For timeout calculation
    utime_t _last_recv_time;    // For FPS calculation
    utime_t _timeout;           // in ms
};
