#include "../include/EP_CorrespondingPoints.hpp"
#include "CM_Camera.hpp"
#include "FEAT_Features.hpp"
#include "LG_Logging.hpp"
#include "PT_Points.hpp"

static struct CorrespondenceExtractor ext;

void EP_InitCPointExtractor(void* extractor, CorrPReturn (*GetCorrp)(void* extractor, cv::Mat img1, cv::Mat img2))
{
    //extractor must be initiated
    ext.extractor = extractor;
    ext.GetCorrp = GetCorrp;
}

PointPair2D EP_CorrespExtract(cv::Mat img1, cv::Mat img2)
{
    LG_Log(LogSeverity::DBG, "[EP_CorrespExtract] Getting corrp\n");
    PointPair2D out = ext.GetCorrp(ext.extractor, img1, img2).first;
    return out;
}

#define SIGNX(x) \
    ((x < 0.0f) ? -1.0f : 1.0f)

cv::Mat __EP_CrossProdMat(cv::Mat x)
{
    assert(x.rows == 3);
    fp64 x1 = x.at<double>(0, 0);
    fp64 x2 = x.at<double>(1, 0);
    fp64 x3 = x.at<double>(2, 0);
    cv::Mat crossprodmat = (cv::Mat_<double>(3,3) <<
            0.0f, -x3, x2, 
            x3, 0.0f, -x1,
            -x2, x1, 0.0f
    );
    return crossprodmat;
}

cv::Mat EP_EFromRigid(cv::Mat R, cv::Mat t)
{
    //IREG p 156, (10.53)
    //This gives us E21
    return R.t() * __EP_CrossProdMat(t);
    //return __EP_CrossProdMat(t) * R;
}

std::pair<cv::Mat, cv::Mat> EP_GetR21t21(cv::Mat R1, cv::Mat t1, cv::Mat R2, cv::Mat t2)
{
    cv::Mat R21 = R2 * R1.t();
    cv::Mat t21 = t2 - R21 * t1;
    std::pair<cv::Mat, cv::Mat> R21t21(R21, t21);
    return R21t21;
}

PointPair2D EP_FindCorrpEpipolar(const PointPair2D& corrp, const cv::Mat& E)
{
    PointPair2D corr_p;
    corr_p.first.reserve(corrp.first.size());
    corr_p.second.reserve(corrp.second.size());
    cv::Matx33d K = CM_GetIntrinsics()->K;
    cv::Matx33d K_inv = K.inv();
    cv::Mat E64;
    E.convertTo(E64, CV_64F);
    cv::Matx33d Ex(E64);
    cv::Matx33d F21 = (K_inv.t() * Ex * K_inv).t();
    cv::Matx33d F21_T = F21.t();

    for(size_t i = 0; i < corrp.first.size(); i++)
    {
        //To homog
        cv::Mat firstp = PROJ_ToHomogFromCart(corrp.first[i]);
        cv::Mat secondp = PROJ_ToHomogFromCart(corrp.second[i]);
        cv::Mat epline1 = F21 * firstp;
        fp64 a = epline1.at<double>(0,0);
        fp64 b = epline1.at<double>(1,0);
        fp64 c = epline1.at<double>(2,0);

        fp64 p1 = secondp.at<double>(0,0);
        fp64 p2 = secondp.at<double>(1,0);
        fp64 p3 = secondp.at<double>(2,0);

        fp64 normfactor = (1.0f / sqrt(a*a + b*b));

        fp64 norma = a * normfactor;
        fp64 normb = b * normfactor;
        fp64 normc = c * normfactor;

        fp64 dist1 = abs(norma*p1 + normb*p2 + normc*p3);

        cv::Mat epline2 = F21_T * secondp;

        a = epline2.at<double>(0,0);
        b = epline2.at<double>(1,0);
        c = epline2.at<double>(2,0);

        p1 = firstp.at<double>(0,0);
        p2 = firstp.at<double>(1,0);
        p3 = firstp.at<double>(2,0);

        normfactor = (1.0f / sqrt(a*a + b*b));

        norma = a * normfactor;
        normb = b * normfactor;
        normc = c * normfactor;
        fp64 dist2 = abs(norma*p1 + normb*p2 + normc*p3);
        fp64 dist = (dist1 + dist2) / 2.0f;
        LG_Log(LogSeverity::DBG, "[EP_FindCorrpEpipolar] dist1 = %lf, dist2 = %lf, dist = %lf\n", dist1, dist2, dist);
        if(dist < PANTO_EPIPOLARTRESHOLD)
        {
            corr_p.first.push_back(corrp.first[i]);
            corr_p.second.push_back(corrp.second[i]);
        }
    }
    return corr_p;
}

PointPair2D EP_FilterPointPairByMask(const PointPair2D& corrp, const cv::Mat& mask)
{
    PointPair2D filtered;

    filtered.first.reserve(corrp.first.size());
    filtered.second.reserve(corrp.second.size());

    for (int i = 0; i < mask.rows; ++i) 
    {
        if (mask.at<uchar>(i)) 
        {
            filtered.first.push_back(corrp.first[i]);
            filtered.second.push_back(corrp.second[i]);
        }
    }

    return filtered;
}

void EP_DrawCorrespondences(const cv::Mat& img1, const cv::Mat& img2, const std::vector<cv::Point2d>& pts1,
        const std::vector<cv::Point2d>& pts2)
{
    const std::string windowName = "Correspondences";
    CV_Assert(pts1.size() == pts2.size());

    cv::Mat left, right;
    if (img1.channels() == 1) cv::cvtColor(img1, left, cv::COLOR_GRAY2BGR);
    else left = img1.clone();

    if (img2.channels() == 1) cv::cvtColor(img2, right, cv::COLOR_GRAY2BGR);
    else right = img2.clone();

    int rows = std::max(left.rows, right.rows);
    int cols = left.cols + right.cols;

    cv::Mat canvas(rows, cols, CV_8UC3, cv::Scalar(0,0,0));
    left.copyTo(canvas(cv::Rect(0, 0, left.cols, left.rows)));
    right.copyTo(canvas(cv::Rect(left.cols, 0, right.cols, right.rows)));

    for (size_t i = 0; i < pts1.size(); ++i)
    {
        cv::Point2f p1 = pts1[i];
        cv::Point2f p2 = pts2[i];
        p2.x += static_cast<float>(left.cols); // shift right image points

        cv::Scalar color(
            (i * 53) % 255,
            (i * 97) % 255,
            (i * 193) % 255
        );

        cv::circle(canvas, p1, 4, color, 2);
        cv::circle(canvas, p2, 4, color, 2);
        cv::line(canvas, p1, p2, color, 1);
    }

    cv::imshow(windowName, canvas);
    cv::waitKey(0);
}
