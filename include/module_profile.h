/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#ifndef MODULE_PROFILE_H
#define MODULE_PROFILE_H

#ifdef ENABLE_FUNCTION_PROFILING
extern volatile int * pprof_buff;
void dumpFunctionParams(int ID);
void initProfile(void);
void closeProfile(void);
void flushProfile(void);

#define PROFILE_FUNCTION_START(ID) dumpFunctionParams(ID);
#define PROFILE_FUNCTION_END(ID) PROFILE_FUNCTION_START(ID)

#define FLUSH_FUNCTION_PROFILE() flushProfile()
#define INIT_FUNCTION_PROFILE() initProfile()
#define CLOSE_FUNCTION_PROFILE() closeProfile()

#else /* !ENABLE_FUNCTION_PROFILING */
#define PROFILE_FUNCTION_START(ID)
#define PROFILE_FUNCTION_END(ID)

#define FLUSH_FUNCTION_PROFILE()
#define INIT_FUNCTION_PROFILE()
#define CLOSE_FUNCTION_PROFILE()
#endif /* !ENABLE_FUNCTION_PROFILING */

#ifdef PARSE_FUNCTION
#define PROFILE_ID_START()   \
  void profile_id_init(void) \
  {
#define PROFILE_ID_END() }
#define PROFILE_ID(id_name, id_num) profiles[id_num].func_name = #id_name;
#else
#define PROFILE_ID_START()
#define PROFILE_ID_END()
#define PROFILE_ID(id_name, id_num) static const int id_name = id_num;
#endif /* PARSE_FUNCTION */

/* ID list */

PROFILE_ID_START()

PROFILE_ID(adtf31xx_readNextFrame, 0)
PROFILE_ID(publish_PointCloud, 1)
PROFILE_ID(adtf31xx_computePointCloud, 2)
PROFILE_ID(adtf31xx_abFrameCompression, 3)
PROFILE_ID(adtf31xx_depthFrameCompression, 4)
PROFILE_ID(Publish_CompressImg, 5)
PROFILE_ID(readInput_Thread, 6)
PROFILE_ID(processOutput_Thread, 7)

PROFILE_ID_END()

#endif /* MODULE_PROFILE_H */
