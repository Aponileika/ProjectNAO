#ifndef __VIZ_VISUALIZATION_HPP_
#define __VIZ_VISUALIZATION_HPP_
#include <string>
#include "PT_PantoImagePoint.hpp"
#include "PT_PantoMapPoints.hpp"
#include "PT_Types.hpp"
#include "MAP_Mapping.hpp"
#include "CM_Camera.hpp"
#include "PROJ_ProjectiveUtils.hpp"
#include "Config.hpp"

void VIZ_WriteColmap(struct ObservationSet os, struct PointSet ps, struct ViewSet vs,
                    std::string path);

#endif //__VIZ_VISUALIZATION_HPP_
