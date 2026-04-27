#include "../include/OB_Observations.hpp"

static void __OB_AddObs(struct ObservationSet* obs, struct ViewSet* views, struct PointSet* points,
        cv::Point2d observation, u64 view_index, u64 point_index);
static void __OB_AddObsPnP(struct ViewSet* views, struct ObservationSet* obs, struct PointSet* points, 
        std::vector<cv::Point2d> pnpPoints, std::vector<u64> points3Didx);

struct ObservationSet* OB_InitObs()
{
    struct ObservationSet* obs = new ObservationSet;
    return obs;
}

void OB_AddObs(struct ObservationSet* obs, struct ViewSet* views, struct PointSet* points, PointPair2D corrp)
{
    std::vector<cv::Point2d> img1points = corrp.first;
    std::vector<cv::Point2d> img2points = corrp.second;
    size_t num_points = img1points.size();
    size_t num_points2 = img2points.size();
    assert(num_points == num_points2);
    for(size_t i = 0; i < num_points; i++)
    {
        __OB_AddObs(obs, views, points, img1points[i], 0, i);
        __OB_AddObs(obs, views, points, img2points[i], 1, i);
    }
}

void OB_Print(struct ObservationSet* obs)
{
    LG_Log("ObservationSet\n");
    LG_Log("observations.size() = %lld\n", obs->observations.size());
    LG_Log("view_indexes.size() = %lld\n", obs->view_indexes.size());
    LG_Log("point_indexes.size() = %lld\n", obs->point_indexes.size());

    size_t n = std::min<size_t>(obs->observations.size(), 10);
    for (size_t i = 0; i < n; ++i)
    {
        LG_Log("obs[%lld] = (%f, %f), view=%llu, point=%llu\n", i, obs->observations[i].x, obs->observations[i].y,
                   obs->view_indexes[i], obs->point_indexes[i]);
    }
}

void OB_SolvePnP(PointPair2D corrp, ViewSet* TView, ObservationSet* TObs, PointSet* TPoints)
{
    /*
     * Find 2D->3D corrp from last view, where the corrp come from the new frame
     * and the last view. Using these 3D points solve PnP on the new frame.
     * After solving pnp it adds the new view, and links the observations
     * and 3d points used in optimization to this view
     * */
    std::vector<cv::Point2d> pnpPoints;
    //This is a bit (very naive).
    pnpPoints.reserve(corrp.second.size());

    std::vector<cv::Point3d> pnpPoints3D;
    //This is a bit (very naive).
    pnpPoints3D.reserve(corrp.second.size());

    std::vector<u64> pnpPoints3Didx;
    //This is a bit (very naive).
    pnpPoints3D.reserve(corrp.second.size());

    u64 vidx = TView->last_sz;
    std::vector<u64> obsidx = TView->observations_indexes[TView->last_sz];

    LG_Log("Finding 2D->3D correspondences\n");
    for(size_t i = 0; i < corrp.first.size(); i++)
    {
        cv::Point2d imagepoint = corrp.first[i];
        std::pair<fp64, fp64> point(imagepoint.x, imagepoint.y);
        std::pair<std::pair<fp64, fp64>, u64> key(point, vidx);
        auto it = TObs->imagepoint2idx.find(key);
        if(it != TObs->imagepoint2idx.end())
        {
            pnpPoints.push_back(corrp.second[i]);
            u64 pidx = TObs->point_indexes[it->second];
            pnpPoints3Didx.push_back(pidx);
            Eigen::Vector3d v = TPoints->points[pidx];
            pnpPoints3D.emplace_back(v(0), v(1), v(2));
        }
    }
    LG_Log("Found %lld 2D<->3D correspondences\n", pnpPoints3D.size());
    struct CameraIntrinsics* ci = CM_GetIntrinsics();
    cv::Mat rvec, t;
    LG_Log("Solving PnP\n");
    cv::solvePnPRansac(pnpPoints3D, pnpPoints, ci->K, ci->distcoeffs, rvec, t,
            false, PnPRansacIts, Reprojerr, conf);
    struct Camera cam;
    LG_Log("Creating cam\n");
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cam = CM_CreateCam(R, t);
    LG_Log("Adding View\n");
    VW_AddView(TView, cam);
    __OB_AddObsPnP(TView, TObs, TPoints, pnpPoints, pnpPoints3Didx);
}

static void __OB_AddObsPnP(struct ViewSet* views, struct ObservationSet* obs, struct PointSet* points, 
        std::vector<cv::Point2d> pnpPoints, std::vector<u64> points3Didx)
{
    assert(pnpPoints.size() == points3Didx.size());
    u64 view_index = views->last_sz;
    for(size_t i = 0; i < pnpPoints.size(); i++)
    {
        cv::Point2d p = pnpPoints[i];
        obs->observations.push_back(p);
        std::pair<fp64, fp64> point(p.x, p.y);
        std::pair<std::pair<fp64, fp64>, u64> key(point, view_index);
        obs->imagepoint2idx[key] = obs->observations.size() - 1;
        obs->view_indexes.push_back(view_index);
        obs->point_indexes.push_back(points3Didx[i]);

        views->observations_indexes[view_index].push_back(obs->observations.size() - 1);
        points->observations_indexes[points3Didx[i]].push_back(obs->observations.size() - 1);
    }
}

#define PUSHVIEW(vidx, views, idx)\
    (vidx.push_back(views->last_sz - 1 + idx))

#define PUSHPOINT(pidx, points, idx)\
    (pidx.push_back(points->last_sz + idx))

void __OB_AddObs(struct ObservationSet* obs, struct ViewSet* views, struct PointSet* points,
        cv::Point2d observation, u64 view_index, u64 point_index)
{
    obs->observations.push_back(observation);
    std::pair<fp64, fp64> point(observation.x, observation.y);
    std::pair<std::pair<fp64, fp64>, u64> key(point, view_index);
    obs->imagepoint2idx[key] = obs->observations.size() - 1;
    PUSHVIEW(obs->view_indexes, views, view_index);
    PUSHPOINT(obs->point_indexes, points, point_index);
    VW_AddObs(views, view_index, (u64)obs->observations.size() - 1);
    PT_AddObs(points, point_index, (u64)obs->observations.size() - 1);
}
