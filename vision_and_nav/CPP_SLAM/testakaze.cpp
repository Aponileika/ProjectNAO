#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <stdio.h>

int main()
{
    auto akaze = cv::AKAZE::create();
    std::string curr_frame1 = "datasets/tsbb33-datasets/turtle/frame1.png";
    cv::Mat frame1 = cv::imread(curr_frame1, cv::IMREAD_COLOR);
    cv::Mat img1;
    cv::cvtColor(frame1, img1, cv::COLOR_BGR2GRAY);
    cv::imshow("read frame gray", img1);
    std::vector<cv::KeyPoint> kp1;
    cv::Mat des1;

    printf("[AKAZE_GetMatches] detect and compute img1\n");
    akaze->detectAndCompute(img1, cv::noArray(), kp1, des1);
    return 0;
}
