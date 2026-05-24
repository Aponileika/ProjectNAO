#include "../include/OB_Observations.hpp"
#include "EP_CorrespondingPoints.hpp"
#include "PROJ_ProjectiveUtils.hpp"

static void __OB_AddObs(struct ObservationSet* obs, struct ViewSet* views, struct PointSet* points,
        cv::Point2d observation, u64 view_index, u64 point_index);
static void __OB_AddObsPnP(struct ViewSet* views, struct ObservationSet* obs, struct PointSet* points, 
        std::vector<cv::Point2d> pnpPoints, std::vector<u64> points3Didx);

struct ObservationSet* OB_InitObs()
{
    struct ObservationSet* obs = new ObservationSet{};
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

struct offset
{
    i64 dx;
    i64 dy;
};

constexpr auto __OB_MakeSearchWindow()
{
    const i32 W = SEARCHWINDOW2D3D*2 + 1;
    const i32 N = W*W;

    std::array<struct offset, N> search_window{};
    for(i64 i = 0; i < W; i++)
    {
        for(i64 j = 0; j < W; j++)
        {
            search_window[i*W + j] = offset{i - SEARCHWINDOW2D3D, j - SEARCHWINDOW2D3D};
        }
    }
    struct offset temp = search_window[0];
    search_window[0] = search_window[SEARCHWINDOW2D3D*W + SEARCHWINDOW2D3D];
    search_window[SEARCHWINDOW2D3D*W + SEARCHWINDOW2D3D] = temp;
    return search_window;
}

inline constexpr auto search = __OB_MakeSearchWindow();

auto __OB_Find2D3D(cv::Point2d imagepoint, struct ObservationSet* TObs, u64 vidx)
{
    i64 x = static_cast<i64>(round(imagepoint.x));
    i64 y = static_cast<i64>(round(imagepoint.y));
    for(const auto& offset : search)
    {
        i64 dx = offset.dx;
        i64 dy = offset.dy;
        std::pair<i64, i64> point(x + dx, y + dy);
        std::pair<std::pair<i64, i64>, u64> key(point, vidx);
        auto it = TObs->imagepoint2idx.find(key);
        if(it != TObs->imagepoint2idx.end())return it;
    }
    return TObs->imagepoint2idx.end();
}

struct PnPret OB_SolvePnP(PointPair2D corrp, ViewSet* TView, ObservationSet* TObs, PointSet* TPoints)
{
    /*
     * Find 2D->3D corrp from last view, where the corrp come from the new frame
     * and the last view. Using these 3D points solve PnP on the new frame.
     * After solving pnp it adds the new view, and links the observations
     * and 3d points used in optimization to this view
     * Returns points not used in pnp
     * */
    struct PnPret ret;

    std::vector<cv::Point2d> pnpPoints;
    //This is a bit (very naive).
    pnpPoints.reserve(corrp.second.size());

    std::vector<cv::Point3d> pnpPoints3D;
    //This is a bit (very naive).
    pnpPoints3D.reserve(corrp.second.size());

    std::vector<u64> pnpPoints3Didx;
    //This is a bit (very naive).
    pnpPoints3Didx.reserve(corrp.second.size());

    u64 vidx = TView->last_sz;
    LG_Log("[OB_SolvePnP] TView->last_sz = %llu\n", TView->last_sz);
    LG_Log("[OB_SolvePnP] TView->views.size() = %zu\n", TView->views.size());
    std::vector<u64> obsidx = TView->observations_indexes[TView->last_sz];

    std::vector<bool> used_3D(TObs->point_indexes.size());
    LG_Log("[OB_SolvePnP] Finding 2D->3D correspondences, with %lld correspondences\n", corrp.first.size());
    for(size_t i = 0; i < corrp.first.size(); i++)
    {
        cv::Point2d imagepoint = corrp.first[i];
        auto it = __OB_Find2D3D(imagepoint, TObs, vidx);
        if(it != TObs->imagepoint2idx.end() && used_3D[it->second] != true)
        {
            pnpPoints.push_back(corrp.second[i]);
            u64 pidx = TObs->point_indexes[it->second];
            used_3D[it->second] = true;
            pnpPoints3Didx.push_back(pidx);
            Eigen::Vector3d v = PROJ_Homog2Cart(TPoints->points[pidx]);
            pnpPoints3D.emplace_back(v(0), v(1), v(2));
        }
        else
        {
            ret.nonpnpPoints.first.push_back(corrp.first[i]);
            ret.nonpnpPoints.second.push_back(corrp.second[i]);
        }
    }
    LG_Log("Found %lld 2D<->3D correspondences\n", pnpPoints3D.size());
    LG_Log("[OB_SolvePnP] found %lld new correspondences\n", ret.nonpnpPoints.first.size());
    struct CameraIntrinsics* ci = CM_GetIntrinsics();
    cv::Mat rvec, t;
    LG_Log("Solving PnP\n");
    if(pnpPoints3D.size() < (size_t)PnPPointCntThreshold)
    {
        LG_Log("not enough observed points were found for pnp, found %lld, need %d\n",
            pnpPoints3D.size(), PnPPointCntThreshold);
        ret.ret = PNP_NOT_ENOUGH_2D3D;
        return ret;
    }
    ret.ret = PNP_SUCCESS;
    cv::solvePnPRansac(pnpPoints3D, pnpPoints, ci->K, ci->distcoeffs, rvec, t,
            false, PnPRansacIts, Reprojerr, conf);
    struct Camera cam;
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    LG_Log("[OB_SolvePnP] creating cam with index %llu\n", vidx + 1);
    cam = CM_CreateCam(R, t, vidx + 1);
    LG_Log("[OB_SolvePnP] Adding View\n");
    VW_AddView(TView, cam);
    __OB_AddObsPnP(TView, TObs, TPoints, pnpPoints, pnpPoints3Didx);
    return ret;
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
        std::pair<i64, i64> point(static_cast<i64>(round(p.x)),static_cast<i64>(round(p.y)));
        std::pair<std::pair<i64, i64>, u64> key(point, view_index);
        obs->imagepoint2idx[key] = obs->observations.size() - 1;
        obs->view_indexes.push_back(view_index);
        obs->point_indexes.push_back(points3Didx[i]);

        views->observations_indexes[view_index].push_back(obs->observations.size() - 1);
        points->observations_indexes[points3Didx[i]].push_back(obs->observations.size() - 1);
    }
}

#define PUSHVIEW(vidx, views, idx)\
    (vidx.push_back(views->last_sz + (idx - 1)))

#define PUSHPOINT(pidx, points, idx)\
            (pidx.push_back(points->last_sz + idx))

void __OB_AddObs(struct ObservationSet* obs, struct ViewSet* views, struct PointSet* points,
        cv::Point2d observation, u64 view_index, u64 point_index)
{
    obs->observations.push_back(observation);
    std::pair<i64, i64> point(static_cast<i64>(round(observation.x)), static_cast<i64>(round(observation.y)));
    std::pair<std::pair<i64, i64>, u64> key(point, views->last_sz - 1 +  view_index);
    obs->imagepoint2idx[key] = obs->observations.size() - 1;
    PUSHVIEW(obs->view_indexes, views, view_index);
    PUSHPOINT(obs->point_indexes, points, point_index);
    VW_AddObs(views, view_index, (u64)obs->observations.size() - 1);
    PT_AddObs(points, point_index, (u64)obs->observations.size() - 1);
}
