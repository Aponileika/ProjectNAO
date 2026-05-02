#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include "../../third-party/ORBSLAM/include/ORBextractor.h"
#include "../../third-party/ORBSLAM/src/ORBextractor.cc"

static std::vector<cv::DMatch> ratioKnnMatches(
    const cv::Mat& des1,
    const cv::Mat& des2,
    cv::BFMatcher& matcher,
    float matchratio)
{
    std::vector<cv::DMatch> good;

    if (des1.empty() || des2.empty())
        return good;

    if (des1.rows < 2 || des2.rows < 2)
        return good;

    std::vector<std::vector<cv::DMatch>> matches;
    matcher.knnMatch(des1, des2, matches, 2);

    for (const auto& m : matches)
    {
        if (m.size() == 2 && m[0].distance < matchratio * m[1].distance)
        {
            good.push_back(m[0]);
        }
    }

    return good;
}

static cv::Mat drawKeypointsWithLabel(
    const cv::Mat& frame,
    const std::vector<cv::KeyPoint>& keypoints,
    int numMatches,
    const std::string& label)
{
    cv::Mat out;

    cv::drawKeypoints(
        frame,
        keypoints,
        out,
        cv::Scalar(0, 255, 0),
        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
    );

    cv::rectangle(
        out,
        cv::Point(0, 0),
        cv::Point(out.cols, 48),
        cv::Scalar(0, 0, 0),
        cv::FILLED
    );

    cv::putText(
        out,
        label,
        cv::Point(10, 20),
        cv::FONT_HERSHEY_SIMPLEX,
        0.55,
        cv::Scalar(255, 255, 255),
        2,
        cv::LINE_AA
    );

    cv::putText(
        out,
        "keypoints: " + std::to_string(keypoints.size()) +
            " | matches: " + std::to_string(numMatches),
        cv::Point(10, 42),
        cv::FONT_HERSHEY_SIMPLEX,
        0.55,
        cv::Scalar(255, 255, 255),
        2,
        cv::LINE_AA
    );

    return out;
}

int main()
{
    constexpr int nfeatures = 1000;
    constexpr float scaleFactor = 1.2f;
    constexpr int nlevels = 8;
    constexpr int edgeThreshold = 31;
    constexpr int patchSize = 31;
    constexpr int fastThreshold = 20;

    constexpr int cellSize = 30;
    constexpr int fastThresholdHigh = 20;
    constexpr int fastThresholdLow = 7;

    constexpr float matchratio = 0.75f;

    cv::VideoCapture cap(0);

    if (!cap.isOpened())
    {
        std::cerr << "Could not open webcam." << std::endl;
        return 1;
    }

    cv::Ptr<cv::ORB> orbPyramid = cv::ORB::create(
        nfeatures,
        scaleFactor,
        nlevels,
        edgeThreshold,
        0,
        2,
        cv::ORB::HARRIS_SCORE,
        patchSize,
        fastThreshold
    );

    cv::Ptr<cv::ORB> orbNoPyramid = cv::ORB::create(
        nfeatures,
        scaleFactor,
        1,
        edgeThreshold,
        0,
        2,
        cv::ORB::HARRIS_SCORE,
        patchSize,
        fastThreshold
    );

    ORB_SLAM::ORBextractor orbpyr{};

    cv::BFMatcher matcher(cv::NORM_HAMMING, false);

    cv::Mat prevCustomDesc;
    cv::Mat prevOrbPyramidDesc;
    cv::Mat prevOrbNoPyramidDesc;

    std::vector<cv::KeyPoint> prevCustomKps;
    std::vector<cv::KeyPoint> prevOrbPyramidKps;
    std::vector<cv::KeyPoint> prevOrbNoPyramidKps;

    while (true)
    {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty())
            break;

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // ------------------------------------------------------------
        // 1. Custom grid FAST over 8-level pyramid, then ORB descriptors
        // ------------------------------------------------------------
        std::vector<cv::KeyPoint> customKps;
        cv::Mat customDesc;
        orbpyr(gray, {}, customKps, customDesc);

        // ------------------------------------------------------------
        // 2. Normal OpenCV ORB with pyramid
        // ------------------------------------------------------------
        std::vector<cv::KeyPoint> orbPyramidKps;
        cv::Mat orbPyramidDesc;

        orbPyramid->detectAndCompute(
            gray,
            cv::noArray(),
            orbPyramidKps,
            orbPyramidDesc
        );

        // ------------------------------------------------------------
        // 3. OpenCV ORB with no pyramid
        // ------------------------------------------------------------
        std::vector<cv::KeyPoint> orbNoPyramidKps;
        cv::Mat orbNoPyramidDesc;

        orbNoPyramid->detectAndCompute(
            gray,
            cv::noArray(),
            orbNoPyramidKps,
            orbNoPyramidDesc
        );

        // ------------------------------------------------------------
        // Match current frame against previous frame
        // ------------------------------------------------------------
        std::vector<cv::DMatch> customGood = ratioKnnMatches(
            prevCustomDesc,
            customDesc,
            matcher,
            matchratio
        );

        std::vector<cv::DMatch> orbPyramidGood = ratioKnnMatches(
            prevOrbPyramidDesc,
            orbPyramidDesc,
            matcher,
            matchratio
        );

        std::vector<cv::DMatch> orbNoPyramidGood = ratioKnnMatches(
            prevOrbNoPyramidDesc,
            orbNoPyramidDesc,
            matcher,
            matchratio
        );

        // ------------------------------------------------------------
        // Visualization: keypoints + match counts
        // ------------------------------------------------------------
        cv::Mat visCustom = drawKeypointsWithLabel(
            frame,
            customKps,
            static_cast<int>(customGood.size()),
            "Custom Grid FAST + 8-Level Pyramid"
        );

        cv::Mat visORB = drawKeypointsWithLabel(
            frame,
            orbPyramidKps,
            static_cast<int>(orbPyramidGood.size()),
            "OpenCV ORB, 8 Levels"
        );

        cv::Mat visORBNoPyr = drawKeypointsWithLabel(
            frame,
            orbNoPyramidKps,
            static_cast<int>(orbNoPyramidGood.size()),
            "OpenCV ORB, 1 Level"
        );

        cv::Mat topRow;
        cv::hconcat(visCustom, visORB, topRow);

        cv::Mat bottomRow;
        cv::resize(visORBNoPyr, bottomRow, cv::Size(topRow.cols, topRow.rows / 2));

        cv::Mat display;
        cv::vconcat(topRow, bottomRow, display);

        cv::imshow("ORB Keypoint + Ratio Match Comparison", display);

        std::cout << "\r"
                  << "Custom kps: " << customKps.size()
                  << " | Custom matches: " << customGood.size()
                  << " || ORB pyramid kps: " << orbPyramidKps.size()
                  << " | ORB pyramid matches: " << orbPyramidGood.size()
                  << " || ORB no-pyr kps: " << orbNoPyramidKps.size()
                  << " | ORB no-pyr matches: " << orbNoPyramidGood.size()
                  << "        "
                  << std::flush;

        // Store current frame data as previous-frame data.
        prevCustomKps = customKps;
        prevCustomDesc = customDesc.clone();

        prevOrbPyramidKps = orbPyramidKps;
        prevOrbPyramidDesc = orbPyramidDesc.clone();

        prevOrbNoPyramidKps = orbNoPyramidKps;
        prevOrbNoPyramidDesc = orbNoPyramidDesc.clone();

        int key = cv::waitKey(1);

        if (key == 27 || key == 'q')
            break;
    }

    std::cout << std::endl;
    return 0;
}
