#ifndef LG_MAP_LOCK_TIMING_HPP_
#define LG_MAP_LOCK_TIMING_HPP_

#include "LG_Logging.hpp"
#include <algorithm>
#include <mutex>

struct typeTrackingMapLockTiming
{
    u64 Loop;
    const fp64& TimeStamp;
    bool Enabled = false;
    u64 Acquisitions = 0;
    fp64 WaitSeconds = 0.0;
    fp64 MaximumWaitSeconds = 0.0;

    ~typeTrackingMapLockTiming()
    {
        if(Enabled)
        {
            LG_LogData("[SLAMTrackingMapWait] loop=%llu frame_s=%.9f acquisitions=%llu "
                    "wait_ms=%.6f max_wait_ms=%.6f\n",
                    static_cast<unsigned long long>(Loop), TimeStamp,
                    static_cast<unsigned long long>(Acquisitions),
                    WaitSeconds * 1000.0, MaximumWaitSeconds * 1000.0);
        }
    }
};

// A Lockable adapter: std::scoped_lock still controls map/graph deadlock
// avoidance. Only calls into the map mutex are timed, never graph waits.
class typeLoggedMapMutex
{
public:
    typeLoggedMapMutex(std::mutex& Mutex, const char* Thread, const char* Stage,
            u64 Frame, typeTrackingMapLockTiming* Timing = nullptr)
        : Mutex(Mutex), Thread(Thread), Stage(Stage), Frame(Frame), Timing(Timing) {}

    void lock()
    {
        const auto Start = PantoClock::now();
        Mutex.lock();
        Acquired(Start);
    }

    bool try_lock()
    {
        const auto Start = PantoClock::now();
        if(!Mutex.try_lock())
        {
            AddWait(std::chrono::duration<fp64>(PantoClock::now() - Start).count());
            return false;
        }
        Acquired(Start);
        return true;
    }

    void unlock()
    {
        const auto ReleasedAt = PantoClock::now();
        Mutex.unlock();
        if(Thread != nullptr)
        {
            // Sample before releasing, then log after release. Other threads
            // may print first; wall_s gives the actual event ordering.
            LG_LogData("[SLAMMapLock] thread=%s iteration=%llu stage=%s event=unlocked "
                    "wall_s=%.9f held_ms=%.6f\n", Thread,
                    static_cast<unsigned long long>(Frame), Stage,
                    std::chrono::duration<fp64>(ReleasedAt.time_since_epoch()).count(),
                    std::chrono::duration<fp64>(ReleasedAt - AcquiredAt).count() * 1000.0);
        }
    }

private:
    void AddWait(fp64 Wait)
    {
        if(Timing != nullptr)
        {
            Timing->WaitSeconds += Wait;
            Timing->MaximumWaitSeconds = std::max(Timing->MaximumWaitSeconds, Wait);
        }
    }

    void Acquired(PantoClock::time_point Start)
    {
        AcquiredAt = PantoClock::now();
        const fp64 Wait = std::chrono::duration<fp64>(AcquiredAt - Start).count();
        AddWait(Wait);
        if(Timing != nullptr)
        {
            ++Timing->Acquisitions;
        }
        if(Thread != nullptr)
        {
            LG_LogData("[SLAMMapLock] thread=%s iteration=%llu stage=%s event=locked "
                    "wall_s=%.9f wait_ms=%.6f\n", Thread,
                    static_cast<unsigned long long>(Frame), Stage,
                    std::chrono::duration<fp64>(AcquiredAt.time_since_epoch()).count(), Wait * 1000.0);
        }
    }

    std::mutex& Mutex;
    const char* Thread;
    const char* Stage;
    u64 Frame;
    typeTrackingMapLockTiming* Timing;
    PantoClock::time_point AcquiredAt{};
};

#endif
