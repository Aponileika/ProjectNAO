#include "../include/EP_CorrespondingPoints.hpp"
#include "CM_Camera.hpp"
#include "LG_Logging.hpp"
#include "PT_Points.hpp"

static struct CorrespondenceExtractor ext;

void EP_InitCPointExtractor(void* extractor, PointPair2D (*GetCorrp)(void* extractor, cv::Mat img1, cv::Mat img2))
{
    //extractor must be initiated
    ext.extractor = extractor;
    ext.GetCorrp = GetCorrp;
}

PointPair2D EP_CorrespExtract(cv::Mat img1, cv::Mat img2)
{
    LG_Log("[EP_CorrespExtract] Getting corrp\n");
    PointPair2D out = ext.GetCorrp(ext.extractor, img1, img2);
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
    // cv::Mat R21 = R1.t() * R2;
    // cv::Mat t21 = R1.t() * (t2 - t1);
    // std::pair<cv::Mat, cv::Mat> R21t21(R21, t21);
    // return R21t21;
}

// static void EP_LogMat(const char* name, const cv::Mat& M)
// {
//     LG_Log("%s: rows=%d, cols=%d, type=%d, channels=%d\n",
//            name, M.rows, M.cols, M.type(), M.channels());
//
//     if (M.empty()) {
//         LG_Log("%s is empty\n", name);
//         return;
//     }
//
//     cv::Mat M64;
//     M.convertTo(M64, CV_64F);
//
//     for (int r = 0; r < M64.rows; ++r) {
//         LG_Log("%s[%d] = ", name, r);
//         for (int c = 0; c < M64.cols; ++c) {
//             double v = M64.at<double>(r, c);
//             LG_Log("%.17g ", v);
//         }
//         LG_Log("\n");
//     }
// }
//
// static void EP_LogMatx33d(const char* name, const cv::Matx33d& M)
// {
//     LG_Log("%s Matx33d:\n", name);
//     for (int r = 0; r < 3; ++r) {
//         LG_Log("%s[%d] = %.17g %.17g %.17g\n",
//                name, r, M(r, 0), M(r, 1), M(r, 2));
//     }
// }
//
// static void EP_LogVec3(const char* name, const cv::Vec3d& v)
// {
//     LG_Log("%s = %.17g %.17g %.17g\n", name, v[0], v[1], v[2]);
// }
// PointPair2D EP_FindCorrpEpipolar(const PointPair2D& corrp, const cv::Mat& E)
// {
//     PointPair2D corr_p;
//     corr_p.first.reserve(corrp.first.size());
//     corr_p.second.reserve(corrp.second.size());
//
//     LG_Log("[EP_FindCorrpEpipolar] input correspondences = %zu\n", corrp.first.size());
//
//     EP_LogMat("[EP_FindCorrpEpipolar] E input before conversion", E);
//
//     cv::Matx33d K = CM_GetIntrinsics()->K;
//     cv::Matx33d K_inv = K.inv();
//
//     EP_LogMatx33d("[EP_FindCorrpEpipolar] K", K);
//     EP_LogMatx33d("[EP_FindCorrpEpipolar] K_inv", K_inv);
//
//     cv::Mat E64;
//     E.convertTo(E64, CV_64F);
//
//     EP_LogMat("[EP_FindCorrpEpipolar] E64 after convertTo", E64);
//
//     if (E64.rows != 3 || E64.cols != 3) {
//         LG_Log("[EP_FindCorrpEpipolar] ERROR: E is not 3x3, got %dx%d\n",
//                E64.rows, E64.cols);
//         return corr_p;
//     }
//
//     cv::Matx33d Ex(E64);
//
//     EP_LogMatx33d("[EP_FindCorrpEpipolar] Ex as Matx33d", Ex);
//
//     cv::Matx33d F21 = K_inv.t() * Ex * K_inv;
//     cv::Matx33d F21_T = F21;
//
//     EP_LogMatx33d("[EP_FindCorrpEpipolar] F21 = K_inv.t() * E * K_inv", F21);
//     EP_LogMatx33d("[EP_FindCorrpEpipolar] F21_T", F21_T);
//
//     for (size_t i = 0; i < corrp.first.size(); i++)
//     {
//         const cv::Vec3d x1(corrp.first[i].x,  corrp.first[i].y,  1.0);
//         const cv::Vec3d x2(corrp.second[i].x, corrp.second[i].y, 1.0);
//
//         const cv::Vec3d l2 = F21 * x1;
//         const cv::Vec3d l1 = F21_T * x2;
//
//         const double denom2 = std::sqrt(l2[0]*l2[0] + l2[1]*l2[1]);
//         const double denom1 = std::sqrt(l1[0]*l1[0] + l1[1]*l1[1]);
//
//         const double r = x2.dot(l2);
//
//         if (i < 10) {
//             LG_Log("[EP_FindCorrpEpipolar] i=%zu\n", i);
//             EP_LogVec3("  x1", x1);
//             EP_LogVec3("  x2", x2);
//             EP_LogVec3("  l2 = F21*x1", l2);
//             EP_LogVec3("  l1 = F21_T*x2", l1);
//             LG_Log("  algebraic r=x2^T F x1 = %.17g\n", r);
//             LG_Log("  denom2 = %.17g, denom1 = %.17g\n", denom2, denom1);
//         }
//
//         if (!std::isfinite(denom1) || !std::isfinite(denom2) ||
//             denom1 < 1e-12 || denom2 < 1e-12) {
//             LG_Log("[EP_FindCorrpEpipolar] skipping i=%zu due to invalid denom: denom1=%.17g denom2=%.17g\n",
//                    i, denom1, denom2);
//             continue;
//         }
//
//         const double dist1 = std::abs(r) / denom2;
//         const double dist2 = std::abs(r) / denom1;
//         const double dist = 0.5 * (dist1 + dist2);
//
//         if (!std::isfinite(dist)) {
//             LG_Log("[EP_FindCorrpEpipolar] nan/inf distance at i=%zu: dist1=%.17g dist2=%.17g dist=%.17g\n",
//                    i, dist1, dist2, dist);
//             continue;
//         }
//
//         if (i < 10) {
//             LG_Log("[EP_FindCorrpEpipolar] dist1=%.17g, dist2=%.17g, dist=%.17g\n",
//                    dist1, dist2, dist);
//         }
//
//         if (dist < EpiPolarTreshhold)
//         {
//             corr_p.first.push_back(corrp.first[i]);
//             corr_p.second.push_back(corrp.second[i]);
//         }
//     }
//
//     LG_Log("[EP_FindCorrpEpipolar] kept %zu / %zu correspondences\n",
//            corr_p.first.size(), corrp.first.size());
//
//     return corr_p;
// }

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
        cv::Mat firstp = PT_ToHomogFromCart(corrp.first[i]);
        cv::Mat secondp = PT_ToHomogFromCart(corrp.second[i]);
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
        LG_Log("[EP_FindCorrpEpipolar] dist1 = %lf, dist2 = %lf, dist = %lf\n", dist1, dist2, dist);
        if(dist < EpiPolarTreshhold)
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
