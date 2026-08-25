#ifndef __VIZ_VISUALIZATION_HPP_
#define __VIZ_VISUALIZATION_HPP_
#include <string>
#include <sys/types.h>
#include <unistd.h>
#include <cstdlib>
#include <filesystem>
#include <signal.h>
#include <sys/wait.h>
#include "PT_PantoImagePoint.hpp"
#include "PT_PantoMapPoints.hpp"
#include "PT_Types.hpp"
#include "MAP_Mapping.hpp"
#include "CM_Camera.hpp"
#include "PROJ_ProjectiveUtils.hpp"
#include "Config.hpp"

void VIZ_InitVisualization(void);
void VIZ_DestroyVisualization(void);
void VIZ_WriteColmap(const typeGlobalMap& GlobalMap);

#endif //__VIZ_VISUALIZATION_HPP_
