#ifndef TH_THREADING_HPP_
#define TH_THREADING_HPP_

// Configure process-wide library pools before calibration or worker startup.
void TH_InitializeLibraries();
// Apply thread-local OpenMP and Accelerate policies to this caller.
void TH_ConfigureCurrentThread();
// Set QoS inside the tracking/mapping thread, without changing process priority.
void TH_SetUserInitiatedQoS(const char* ThreadName);

#endif
