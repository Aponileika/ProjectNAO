#ifndef __VIZ_VISUALIZATION_HPP_
#define __VIZ_VISUALIZATION_HPP_
#include <string>
#include <sys/types.h>
#include <unistd.h>
#include <cstdlib>
#include <filesystem>
#include <signal.h>
#include <csignal>
#include <sys/wait.h>
#include "PT_PantoImagePoint.hpp"
#include "PT_PantoMapPoints.hpp"
#include "PT_Types.hpp"
#include "MAP_Mapping.hpp"
#include "CM_Camera.hpp"
#include "PROJ_ProjectiveUtils.hpp"
#include "Config.hpp"
#include "PANTOVEC_PantoVector.hpp"

void VIZ_InitVisualization(void);
void VIZ_StopViewer();
void VIZ_SignalHandler(int Signal);
void VIZ_WriteColmap(const typeGlobalMap& GlobalMap, const std::vector<Eigen::Vector3d>& TrackingTrajectory);

#endif //__VIZ_VISUALIZATION_HPP_
