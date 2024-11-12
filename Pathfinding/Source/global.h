/* Copyright Steve Rabin, 2012. 
 * All rights reserved worldwide.
 *
 * This software is provided "as is" without express or implied
 * warranties. You may freely copy and compile this source into
 * applications you distribute provided that the copyright text
 * below is included in the resulting source code, for example:
 * "Portions Copyright Steve Rabin, 2012"
 */

#pragma once

/* uncomment the following for Project 3 */
//#define PROJECT_THREE

/* uncomment the following for any extra credit */
//#define EXTRACREDIT_ROYFLOYDWARSHALL
//#define EXTRACREDIT_GOALBOUNDING
//#define EXTRACREDIT_JPSPLUS
#define EXTRACREDIT_FOGOFWAR

enum ExtraCredit
{
	EXTRA_None,
#if defined (EXTRACREDIT_ROYFLOYDWARSHALL)
	EXTRA_RoyFloydWarshall,
#endif
#if defined (EXTRACREDIT_GOALBOUNDING)
	EXTRA_GoalBounding,
#endif
#if defined (EXTRACREDIT_JPSPLUS)
	EXTRA_JPSPlus,
#endif
#if defined (EXTRACREDIT_FOGOFWAR)
	EXTRA_FogOfWar,
#endif

	EXTRA_Count,
};

enum SampleTestStatus
{
	Status_None = 0,
	Status_Pass,
	Status_Fail,
	Status_Running,
};

#define ASSERTMSG(eval, message) assert(eval && message)
#define COMPILE_TIME_ASSERT(expression, message) { typedef int ASSERT__##message[1][(expression)]; }

#define g_blackboard Singleton<BlackBoard>::GetInstance()
#define g_clock Singleton<Clock>::GetInstance()
#define g_database Singleton<Database>::GetInstance()
#define g_msgroute Singleton<MsgRoute>::GetInstance()
#define g_debuglog Singleton<DebugLog>::GetInstance()
#define g_debugdrawing Singleton<DebugDrawing>::GetInstance()
#define g_terrain Singleton<Terrain>::GetInstance()
#define g_trigger Singleton<TriggerSystem>::GetInstance()
#define g_random Singleton<Random>::GetInstance()
#define g_tests Singleton<PathfindingTests>::GetInstance()

#define INVALID_OBJECT_ID 0
#define SYSTEM_OBJECT_ID 1

typedef unsigned int objectID;

static const float DEFAULT_DECAY = 0.05f;
static const float DEFAULT_GROWING = 0.150f;
static const float DEFAULT_UPDATEFREQUENCY = 0.066f;
static const float DEFAULT_FOVANGLE = 180.0f;
static const float DEFAULT_FOVCLOSEDISTANCE = 2.0f;

static bool Near(float x, float y) { return std::abs(x - y) < 1e-5f; }

static const unsigned NUMTESTS = 1000;