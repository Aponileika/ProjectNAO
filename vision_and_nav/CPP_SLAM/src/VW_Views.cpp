#include "../include/VW_Views.hpp"

struct ViewSet* VW_InitViewSet()
{
    struct ViewSet* viewset = (struct ViewSet*)malloc(sizeof(struct ViewSet));
    viewset->views = {};
    viewset->last_sz = 0;
    viewset->observations_indexes = {};
    return viewset;
}

void VW_AddView(struct ViewSet* views, Camera camera)
{
    views->last_sz = views->views.size();
    views->views.push_back(camera);
    views->observations_indexes.push_back({});
}

void VW_AddObs(struct ViewSet* views, u64 viewidx, u64 obsidx)
{
    views->observations_indexes[views->last_sz - 1 + viewidx].push_back(obsidx);
}

void VW_Print(struct ViewSet* views)
{
    LG_Log("ViewSet\n");
    LG_Log("views.size(): %zu\n", views->views.size());
    LG_Log("observations_indexes.size(): %zu\n", views->observations_indexes.size());
    LG_Log("last_sz: %zu\n", views->last_sz);

    size_t n = std::min<size_t>(views->views.size(), 10);
    for (size_t i = 0; i < n; ++i)
    {
        const Camera& cam = views->views[i];

        LG_Log("view[%zu]\n", i);
        LG_Log("  obs count: %zu\n", views->observations_indexes[i].size());
        LG_Log("  intrinsics: %s\n", cam.intrinsics ? "set" : "null");
        LG_Log("  p: %s\n", cam.p ? "set" : "null");

        if (cam.p)
        {
            const Eigen::Quaterniond& q = cam.p->q;
            const Eigen::Vector3d& t = cam.p->t;

            LG_Log("  q (w,x,y,z) = (%.15f, %.15f, %.15f, %.15f)\n",
                   q.w(), q.x(), q.y(), q.z());

            LG_Log("  t = (%.15f, %.15f, %.15f)\n",
                   t.x(), t.y(), t.z());
        }

        if (!cam.R.empty())
        {
            LG_Log("  R =\n");
            for (int r = 0; r < cam.R.rows; ++r)
            {
                LG_Log("    ");
                for (int c = 0; c < cam.R.cols; ++c)
                {
                    LG_Log("%.15f ", cam.R.at<double>(r, c));
                }
                LG_Log("\n");
            }
        }
        else
        {
            LG_Log("  R: empty\n");
        }

        if (!cam.t.empty())
        {
            LG_Log("  t_cv = (");
            if (cam.t.rows == 3 && cam.t.cols == 1)
            {
                LG_Log("%.15f, %.15f, %.15f",
                       cam.t.at<double>(0, 0),
                       cam.t.at<double>(1, 0),
                       cam.t.at<double>(2, 0));
            }
            else if (cam.t.rows == 1 && cam.t.cols == 3)
            {
                LG_Log("%.15f, %.15f, %.15f",
                       cam.t.at<double>(0, 0),
                       cam.t.at<double>(0, 1),
                       cam.t.at<double>(0, 2));
            }
            else
            {
                LG_Log("shape=(%d x %d)", cam.t.rows, cam.t.cols);
            }
            LG_Log(")\n");
        }
        else
        {
            LG_Log("  t_cv: empty\n");
        }
    }
}
