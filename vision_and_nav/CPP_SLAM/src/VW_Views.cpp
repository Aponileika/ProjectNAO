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
    LG_Log("views.size(): %lld\n", views->views.size());
    LG_Log("observations_indexes.size(): %lld\n", views->observations_indexes.size());
    LG_Log("last_sz: %lld\n", views->last_sz);

    size_t n = std::min<size_t>(views->observations_indexes.size(), 10);
    for (size_t i = 0; i < n; ++i)
    {
        LG_Log("view[%lld] has %lld observations\n", i, views->observations_indexes[i].size());
    }
}
