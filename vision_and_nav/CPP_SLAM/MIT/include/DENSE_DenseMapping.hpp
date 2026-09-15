#ifndef DENSE_DENSEMAPPING_HPP
#define DENSE_DENSEMAPPING_HPP
#include "FR_Frames.hpp"
#include "CM_Camera.hpp"
#include <Eigen/Dense>
#include <condition_variable>
#include <mutex>
#include "PANTOVEC_PantoVector.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/opencv.hpp"

typedef struct
{
    u64 KeyFrameID;

    Eigen::Matrix<fp64, 3, Eigen::Dynamic> CameraPoints; // Points in ccs
}typeDenseKeyFrameMap;

typedef struct 
{
    u64 KeyFrameID;
    cv::Mat LeftImage;
    cv::Mat RightImage;
}typeDenseData;

class typeDenseFrameQueue
{
    public:

        std::queue<typeDenseData> DenseDataQueue;
        std::mutex Mutex;
        std::condition_variable QueueCV;
        std::atomic_bool Stop{false};

        void enque(const typeDenseData& DenseData)
        {
            {
                // this makes sense, if for some reason the push fails, tracking continues as normal
                std::lock_guard<std::mutex> Lock(Mutex);
                DenseDataQueue.push(std::move(DenseData));
            }
            QueueCV.notify_one();
        }

        bool deque(typeDenseData& DenseData)
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
};

typedef struct
{
    // Later the removal in localmapping must be synced with this.
    typePantoVector<typeDenseKeyFrameMap> KeyFrameMaps;

    class typeDenseFrameQueue* DenseQueue;
}typeDenseMapData;

#endif // DENSE_DENSEMAPPING_HPP
