/* Copyright Steve Rabin, 2012. 
 * All rights reserved worldwide.
 *
 * This software is provided "as is" without express or implied
 * warranties. You may freely copy and compile this source into
 * applications you distribute provided that the copyright text
 * below is included in the resulting source code, for example:
 * "Portions Copyright Steve Rabin, 2012"
 */

//These message names are processed inside msg.h

REGISTER_MESSAGE_NAME(MSG_NULL)							//Reserved message name
REGISTER_MESSAGE_NAME(MSG_GENERIC_TIMER)				//Reserved message name
REGISTER_MESSAGE_NAME(MSG_SPAWN_GENERIC_TIMER)			//Reserved message name
REGISTER_MESSAGE_NAME(MSG_CHANGE_STATE_DELAYED)			//Reserved message name
REGISTER_MESSAGE_NAME(MSG_CHANGE_SUBSTATE_DELAYED)		//Reserved message name

//Add new messages here
REGISTER_MESSAGE_NAME(MSG_SetGoal)
REGISTER_MESSAGE_NAME(MSG_SetHeuristicWeight)
REGISTER_MESSAGE_NAME(MSG_SetHeuristicCalc)
REGISTER_MESSAGE_NAME(MSG_SetSmoothing)
REGISTER_MESSAGE_NAME(MSG_SetRubberbanding)
REGISTER_MESSAGE_NAME(MSG_SetStraightline)
REGISTER_MESSAGE_NAME(MSG_SetAgentSpeed)
REGISTER_MESSAGE_NAME(MSG_SetSingleStep)
REGISTER_MESSAGE_NAME(MSG_SetAStarDebugDraw)
REGISTER_MESSAGE_NAME(MSG_SetAStarUsesAnalysis)
REGISTER_MESSAGE_NAME(MSG_SetMoving)
REGISTER_MESSAGE_NAME(MSG_RunTimingsShort)
REGISTER_MESSAGE_NAME(MSG_RunTimingsLong)
REGISTER_MESSAGE_NAME(MSG_RunSampleTestDijkstra)
REGISTER_MESSAGE_NAME(MSG_RunSampleTestAStarEuclidean)
REGISTER_MESSAGE_NAME(MSG_RunSampleTestAStarOctile)
REGISTER_MESSAGE_NAME(MSG_RunSampleTestShortDijkstra)
REGISTER_MESSAGE_NAME(MSG_RunSampleTestShortAStarEuclidean)
REGISTER_MESSAGE_NAME(MSG_RunSampleTestShortAStarOctile)
REGISTER_MESSAGE_NAME(MSG_RunSampleTestDiagonal)
REGISTER_MESSAGE_NAME(MSG_RunSampleTestStraightline)
REGISTER_MESSAGE_NAME(MSG_RunSampleTestRubberbanding)
REGISTER_MESSAGE_NAME(MSG_RunSampleTestSmoothing)
REGISTER_MESSAGE_NAME(MSG_RunSampleTestRubberbandSmooth)
REGISTER_MESSAGE_NAME(MSG_GetSampleTestResult1)
REGISTER_MESSAGE_NAME(MSG_GetSampleTestResult2)
REGISTER_MESSAGE_NAME(MSG_RunDefaultTest1)
REGISTER_MESSAGE_NAME(MSG_RunDefaultTest2)
REGISTER_MESSAGE_NAME(MSG_RunAllTests)
REGISTER_MESSAGE_NAME(MSG_CreateSampleTests)
REGISTER_MESSAGE_NAME(MSG_MapChange)
REGISTER_MESSAGE_NAME(MSG_ExtraCredit)

REGISTER_MESSAGE_NAME(MSG_SetGoalEnemy)
REGISTER_MESSAGE_NAME(MSG_ArrivedEnemy)
REGISTER_MESSAGE_NAME(MSG_SetGoalEnemyRunToPlayer)
REGISTER_MESSAGE_NAME(MSG_SetGoalEnemySeekPlayer)
REGISTER_MESSAGE_NAME(MSG_SetGoalEnemyPatrol)
REGISTER_MESSAGE_NAME(MSG_FoWRePath)

//Used for Zombie and Human demo state machines
REGISTER_MESSAGE_NAME(MSG_CheckTouch)
REGISTER_MESSAGE_NAME(MSG_Tagged)
REGISTER_MESSAGE_NAME(MSG_SetTargetPosition)
REGISTER_MESSAGE_NAME(MSG_Arrived)
REGISTER_MESSAGE_NAME(MSG_Reset)


//Unit test messages
REGISTER_MESSAGE_NAME(MSG_UnitTestMessage)
REGISTER_MESSAGE_NAME(MSG_UnitTestMessage2)
REGISTER_MESSAGE_NAME(MSG_UnitTestMessage3)
REGISTER_MESSAGE_NAME(MSG_UnitTestMessage4)
REGISTER_MESSAGE_NAME(MSG_UnitTestMessage5)
REGISTER_MESSAGE_NAME(MSG_UnitTestMessage6)
REGISTER_MESSAGE_NAME(MSG_UnitTestMessage7)
REGISTER_MESSAGE_NAME(MSG_UnitTestMessage8)
REGISTER_MESSAGE_NAME(MSG_UnitTestMessage9)
REGISTER_MESSAGE_NAME(MSG_UnitTestBroken)
REGISTER_MESSAGE_NAME(MSG_UnitTestPing)
REGISTER_MESSAGE_NAME(MSG_UnitTestAck)
REGISTER_MESSAGE_NAME(MSG_UnitTestDone)
REGISTER_MESSAGE_NAME(MSG_UnitTestTimer)
