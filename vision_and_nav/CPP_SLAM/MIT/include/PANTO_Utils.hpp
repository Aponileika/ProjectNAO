#ifndef __VT_VECUTILS_HPP_
#define __VT_VECUTILS_HPP_
#include <vector>
#include <assert.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "CArenaAlloc.h"
#include "Config.hpp"
#include "PANTOVEC_PantoVector.hpp"

template<typename T>
void PANTO_EraseUnordered(std::vector<T> vec, u64 idx)
{
    std::size_t last_idx = vec.size() - static_cast<std::size_t>(1);

    assert(idx <= last_idx);
    
    if(idx != last_idx)
    {
        vec[idx] = std::move(vec.back());
    }

    vec.pop_back();
}

template<typename T>
T PANTO_Cv2Eigen(cv::Mat cvmat)
{
    T eigenmat;
    cv2eigen(cvmat, eigenmat);
    return eigenmat;
}

template<typename T>
T PANTO_Cv2Eigen(cv::Matx33d cvmat)
{
    T eigenmat;
    cv2eigen(cvmat, eigenmat);
    return eigenmat;
}

//Templated single producer single consumer queue
template<typename T>
class typeSPSCQueue 
{
    public:

        std::queue<T> DenseDataQueue;
        std::mutex Mutex;
        std::condition_variable QueueCV;
        std::atomic_bool Stop{false};

        typeSPSCQueue() = default;

        void enque(const T& DenseData)
        {
            {
                // this makes sense, if for some reason the push fails, tracking continues as normal
                std::lock_guard<std::mutex> Lock(Mutex);
                DenseDataQueue.push(std::move(DenseData));
            }
            QueueCV.notify_one();
        }

        bool deque(T& DenseData)
        {
            // unique_lock since wait needs to be able to unlock and lock again.
            std::unique_lock<std::mutex> Lock(Mutex);

            QueueCV.wait(
                    Lock,
                    [this]()
                    {
                        return !DenseDataQueue.empty() || Stop.load();
                    });

            if(DenseDataQueue.empty())
            {
                return false;
            }

            DenseData = std::move(DenseDataQueue.front());
            DenseDataQueue.pop();

            return true;
        }

        void stop(void)
        {
            Stop.store(true);
            QueueCV.notify_all();
        }
};

u32 PANTO_HammingDistance(const typeDescriptor& a, const typeDescriptor& b);
u32 PANTO_HammingDistance(typeDescriptor& a, typeDescriptor& b);

#endif //__VT_VECUTILS_HPP_
