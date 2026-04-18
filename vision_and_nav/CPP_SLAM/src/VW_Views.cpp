#include "VW_Views.hpp"

struct ViewSet* VW_InitView()
{
    struct ViewSet* viewset = (struct ViewSet*)malloc(sizeof(struct ViewSet));
    viewset->views = {};
    viewset->observations_indexes = {};
    return viewset;
}

void VW_AddView(struct ViewSet* views, Camera camera)
{
    views->views.push_back(camera);
    views->observations_indexes.push_back({});
}

void VW_AddObs(struct ViewSet* views, u64 viewidx, u64 obsidx)
{
    views->observations_indexes[viewidx].push_back(obsidx);
}
