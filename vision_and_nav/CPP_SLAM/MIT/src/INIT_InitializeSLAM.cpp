#include "../include/INIT_InitializeSLAM.hpp"
#include "INITPriv_InitializeSLAM.hpp"

static typePantoInitStruct InitStruct = 
{
    .InitFrames{},
    .FeatureTracks{},
    .EnoughStationaryPointsForInit = false
};

void INIT_CreateInitStruct(void)
{
    typePantoFrame FirstFrame = FR_GetFrame();
    DescRet Descriptors = EP_GetDescriptors(FirstFrame.Frame);
}

void INIT_MatchHistoricalFrames(void);
void INIT_STRANSAC(void);
std::pair<typeInitFrame, typeInitFrame> INIT_Initialize(void);
void INIT_DestroyInitStruct(void);

