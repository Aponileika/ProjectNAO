#ifndef __VIZ_VISUALIZATION_HPP_
#define __VIZ_VISUALIZATION_HPP_
#include <string>
#include "OB_Observations.hpp"
#include "PT_Points.hpp"
#include "VW_Views.hpp"
#include "CM_Camera.hpp"
#include "PROJ_ProjectiveUtils.hpp"
#include "Config.hpp"

void VIZ_WriteColmap(struct ObservationSet os, struct PointSet ps, struct ViewSet vs,
                    std::string path);

#endif //__VIZ_VISUALIZATION_HPP_
