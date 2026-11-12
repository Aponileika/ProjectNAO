#include "LG_MapLockTiming.hpp"
#include <future>
#include <thread>
#include "../src/LG_Logging.cpp"

static void Require(bool Condition, const char* Message)
{
    if(!Condition)
    {
        std::cerr << "FAIL: " << Message << '\n';
        std::exit(1);
    }
}

static void TestMapContention()
{
    std::mutex Map;
    std::unique_lock Held(Map);
    std::promise<void> Starting;
    fp64 Wait = 0.0;
    std::thread Tracking([&]()
    {
        const fp64 TimeStamp = 123.25;
        typeTrackingMapLockTiming Timing{1, TimeStamp};
        Timing.Enabled = true;
        typeLoggedMapMutex Mutex(Map, "tracking", "map_snapshot", 1, &Timing);
        Starting.set_value();
        {
            std::scoped_lock Lock(Mutex);
            Wait = Timing.WaitSeconds;
            Require(Timing.Acquisitions == 1, "one acquisition recorded");
        }
    });
    Starting.get_future().wait();
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    Held.unlock();
    Tracking.join();
    Require(Wait >= 0.02, "time blocked on the map mutex is recorded");
    Require(Map.try_lock(), "underlying map mutex released");
    Map.unlock();
}

static void TestGraphContention()
{
    std::mutex Map;
    std::mutex Graph;
    std::unique_lock Held(Graph);
    std::promise<void> Starting;
    fp64 Wait = 0.0;
    fp64 Elapsed = 0.0;
    std::thread Tracking([&]()
    {
        const fp64 TimeStamp = 123.5;
        typeTrackingMapLockTiming Timing{2, TimeStamp};
        Timing.Enabled = true;
        typeLoggedMapMutex Mutex(Map, "tracking", "map_and_graph", 2, &Timing);
        const auto Start = PantoClock::now();
        Starting.set_value();
        {
            std::scoped_lock Lock(Mutex, Graph);
            Wait = Timing.WaitSeconds;
            Elapsed = std::chrono::duration<fp64>(PantoClock::now() - Start).count();
        }
    });
    Starting.get_future().wait();
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    Held.unlock();
    Tracking.join();
    Require(Elapsed >= 0.02 && Wait < Elapsed * 0.5, "graph contention is excluded from map wait");
}

int main()
{
    // Test the real logger without touching the user's log directory.
    glogger.Datafp = std::tmpfile();
    Require(glogger.Datafp != nullptr, "temporary DATA log");
    gloggerisinit = true;
    LG_Log(LogSeverity::DATA, "muted runtime message\n");
    TestMapContention();
    TestGraphContention();
    {
        const fp64 TimeStamp = 123.75;
        typeTrackingMapLockTiming Skipped{3, TimeStamp};
        Skipped.Enabled = true;
    }
    {
        std::mutex Map;
        typeLoggedMapMutex Mutex(Map, "local_mapping", "topology", 7);
        std::scoped_lock Lock(Mutex);
    }
    std::rewind(glogger.Datafp);
    std::string Logs;
    char Buffer[1024];
    while(std::fgets(Buffer, sizeof(Buffer), glogger.Datafp))
    {
        Logs += Buffer;
    }
    Require(Logs.find("muted runtime message") == std::string::npos, "existing runtime suppression preserved");
    Require(Logs.find("[SLAMTrackingMapWait] loop=1 frame_s=123.250000000 acquisitions=1") != std::string::npos,
            "per-loop DATA summary emitted while runtime logging is muted");
    Require(Logs.find("loop=3 frame_s=123.750000000 acquisitions=0 wait_ms=0.000000") != std::string::npos,
            "loops without a map acquisition still emit a zero summary");
    Require(Logs.find("thread=local_mapping iteration=7 stage=topology event=locked") != std::string::npos &&
            Logs.find("thread=local_mapping iteration=7 stage=topology event=unlocked") != std::string::npos,
            "mapping acquisition and release stages emitted");
    LG_CloseLogger();
    std::cout << "Global-map lock diagnostics tests passed\n";
}
