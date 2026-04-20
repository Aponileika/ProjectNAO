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
    std::cout << "ViewSet\n";
    std::cout << "  views.size(): " << views->views.size() << "\n";
    std::cout << "  observations_indexes.size(): " << views->observations_indexes.size() << "\n";
    std::cout << "  last_sz: " << views->last_sz << "\n";

    size_t n = std::min<size_t>(views->observations_indexes.size(), 10);
    for (size_t i = 0; i < n; ++i)
    {
        std::cout << "  view[" << i << "] has "
                  << views->observations_indexes[i].size()
                  << " observations\n";
    }
}
