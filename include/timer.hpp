#include <ctime>

/* Unix */
class Timer
{
public:
    Timer() { clock_gettime(CLOCK_REALTIME, &beg_); }

    double elapsed() {
        clock_gettime(CLOCK_REALTIME, &end_);
        return (end_.tv_sec - beg_.tv_sec) * 1000 +
            (end_.tv_nsec - beg_.tv_nsec) / 1000000.;
    }
    
    void reset() { clock_gettime(CLOCK_REALTIME, &beg_); }
    
private:
    timespec beg_, end_;
};
