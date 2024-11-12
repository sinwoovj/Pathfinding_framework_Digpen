/* Copyright Steve Rabin, 2012. 
 * All rights reserved worldwide.
 *
 * This software is provided "as is" without express or implied
 * warranties. You may freely copy and compile this source into
 * applications you distribute provided that the copyright text
 * below is included in the resulting source code, for example:
 * "Portions Copyright Steve Rabin, 2012"
 */


#include <Stdafx.h>

extern D3DXVECTOR3 g_click3D;


//Add new states here
enum StateName {
	STATE_Initialize,	//Note: the first enum is the starting state
	STATE_CalcPath,
	STATE_MoveToGoal,
	STATE_Idle,
	STATE_TimingTestShort,
	STATE_TimingTestLong,
	STATE_SampleTestDijkstra,
	STATE_SampleTestAStarEuclidean,
	STATE_SampleTestAStarOctile,
	STATE_SampleTestShortDijkstra,
	STATE_SampleTestShortAStarEuclidean,
	STATE_SampleTestShortAStarOctile,
	STATE_SampleTestDiagonal,
	STATE_SampleTestStraightline,
	STATE_SampleTestRubberbanding,
	STATE_SampleTestSmoothing,
	STATE_SampleTestRubberbandSmooth,
	STATE_GetSampleTest1,
	STATE_GetSampleTest2,
	STATE_DefaultTest1,
	STATE_DefaultTest2,
	STATE_AllTests,
	STATE_CreateSampleTests
};

//Add new substates here
enum SubstateName {
	//empty
};

void Agent::InitFogOfWar(bool enabled)
{
	if (enabled)
		g_terrain.InitFogOfWar();
	else
		g_terrain.ClearFogOfWar();
}

bool Agent::States( State_Machine_Event event, MSG_Object * msg, int state, int substate )
{
BeginStateMachine

	//Global message responses

	OnMsg( MSG_MapChange )
		//////////////////////////////////////
		// Add code here to handle map changes
		//////////////////////////////////////


#if defined (EXTRACREDIT_FOGOFWAR)
		InitFogOfWar(m_owner->GetMovement().GetFogOfWar());
#endif

	OnMsg( MSG_Reset )
		ResetStateMachine();

	OnMsg( MSG_SetHeuristicWeight )
		m_owner->GetMovement().SetHeuristicWeight( msg->GetFloatData() );

	OnMsg( MSG_SetHeuristicCalc )
		m_owner->GetMovement().SetHeuristicCalc( msg->GetIntData() );

	OnMsg( MSG_SetSmoothing )
		m_owner->GetMovement().SetSmoothPath( msg->GetBoolData() );

	OnMsg( MSG_SetRubberbanding )
		m_owner->GetMovement().SetRubberbandPath( msg->GetBoolData() );

	OnMsg( MSG_SetStraightline )
		m_owner->GetMovement().SetStraightlinePath( msg->GetBoolData() );

	OnMsg( MSG_SetAgentSpeed )
		m_animStyle = msg->GetIntData();

	OnMsg( MSG_SetSingleStep )
		m_owner->GetMovement().SetSingleStep( msg->GetBoolData() );

	OnMsg( MSG_ExtraCredit )
		m_owner->GetMovement().SetExtraCredit( msg->GetIntData() );

	OnMsg(MSG_SetAStarDebugDraw)
		m_owner->GetMovement().SetDebugDraw(msg->GetBoolData());

	OnMsg( MSG_SetAStarUsesAnalysis )
		m_owner->GetMovement().AStarUsesAnalysis( msg->GetBoolData() );

	OnMsg( MSG_SetMoving )
		m_moving = msg->GetBoolData();

	OnMsg( MSG_RunTimingsShort )
		ChangeState( STATE_TimingTestShort );

	OnMsg( MSG_RunTimingsLong )
		ChangeState( STATE_TimingTestLong );

	OnMsg(MSG_RunSampleTestDijkstra)
		ChangeState(STATE_SampleTestDijkstra);

	OnMsg(MSG_RunSampleTestAStarEuclidean)
		ChangeState(STATE_SampleTestAStarEuclidean);

	OnMsg(MSG_RunSampleTestAStarOctile)
		ChangeState(STATE_SampleTestAStarOctile);

	OnMsg(MSG_RunSampleTestShortDijkstra)
		ChangeState(STATE_SampleTestShortDijkstra);

	OnMsg(MSG_RunSampleTestShortAStarEuclidean)
		ChangeState(STATE_SampleTestShortAStarEuclidean);

	OnMsg(MSG_RunSampleTestShortAStarOctile)
		ChangeState(STATE_SampleTestShortAStarOctile);

	OnMsg(MSG_RunSampleTestDiagonal)
		ChangeState(STATE_SampleTestDiagonal);

	OnMsg(MSG_RunSampleTestStraightline)
		ChangeState(STATE_SampleTestStraightline);

	OnMsg(MSG_RunSampleTestRubberbanding)
		ChangeState(STATE_SampleTestRubberbanding);

	OnMsg(MSG_RunSampleTestSmoothing)
		ChangeState(STATE_SampleTestSmoothing);

	OnMsg(MSG_RunSampleTestRubberbandSmooth)
		ChangeState(STATE_SampleTestRubberbandSmooth);

	OnMsg(MSG_GetSampleTestResult1)
		ChangeState(STATE_GetSampleTest1);

	OnMsg(MSG_GetSampleTestResult2)
		ChangeState(STATE_GetSampleTest2);

	OnMsg(MSG_RunDefaultTest1)
		ChangeState(STATE_DefaultTest1);

	OnMsg(MSG_RunDefaultTest2)
		ChangeState(STATE_DefaultTest2);

	OnMsg(MSG_RunAllTests)
		ChangeState(STATE_AllTests);

	OnMsg(MSG_CreateSampleTests)
		ChangeState(STATE_CreateSampleTests);

	OnMsg( MSG_SetGoal )
		int row = -1;
		int col = -1;
		if(g_terrain.GetRowColumn( &g_click3D, &row, &col ))
		{
			if( !g_terrain.IsWall( row, col ) )
			{ 
				g_terrain.ResetColors();
				g_terrain.SetColor( row, col, DEBUG_COLOR_BLUE );

				m_rowGoal = row;
				m_colGoal = col;
				bool foundPath = m_owner->GetMovement().ComputePathWithTiming( m_rowGoal, m_colGoal, true ); 
				if( foundPath ) 
				{
					ChangeState( STATE_MoveToGoal );
				}
				else
				{
					ChangeState( STATE_CalcPath );
				}
			}
		}

	///////////////////////////////////////////////////////////////
	DeclareState( STATE_Initialize )

		OnEnter
			m_animStyle = 1;
			m_owner->GetMovement().SetIdleSpeed();
			

	///////////////////////////////////////////////////////////////
	DeclareState(STATE_CalcPath)

		OnEnter
			m_owner->GetMovement().SetIdleSpeed();

		OnUpdate
			Movement& move = m_owner->GetMovement();
			bool foundPath = m_owner->GetMovement().ComputePathWithTiming( m_rowGoal, m_colGoal, false );
			if(foundPath)
			{
				ChangeState( STATE_MoveToGoal );
			}


	///////////////////////////////////////////////////////////////
	DeclareState( STATE_MoveToGoal )

		OnEnter
			if(!m_moving) { ChangeState( STATE_Idle ); }
			if(m_animStyle == 0) { m_owner->GetMovement().SetWalkSpeed(); }
			else if(m_animStyle == 1) { m_owner->GetMovement().SetJogSpeed(); }
			else { m_owner->GetMovement().SetWalkSpeed(); }

		OnMsg( MSG_Arrived )
			ChangeState( STATE_Idle );

		OnMsg( MSG_SetAgentSpeed )
			m_animStyle = msg->GetIntData();
			if(m_animStyle == 0) { m_owner->GetMovement().SetWalkSpeed(); }
			else if(m_animStyle == 1) { m_owner->GetMovement().SetJogSpeed(); }
			else { m_owner->GetMovement().SetWalkSpeed(); }


	///////////////////////////////////////////////////////////////
	DeclareState( STATE_Idle )

		OnEnter
			m_owner->GetMovement().SetIdleSpeed();


	///////////////////////////////////////////////////////////////
	DeclareState( STATE_TimingTestShort )

		OnEnter
			RunTimingTest(4, 18, 16, 1, 0, "TimingResultsShort.txt");
			ChangeState( STATE_Idle );


	///////////////////////////////////////////////////////////////
	DeclareState( STATE_TimingTestLong )

		OnEnter
			RunTimingTest(1, 38, 36, 1, 0, "TimingResultsLong.txt");
			ChangeState( STATE_Idle );

	///////////////////////////////////////////////////////////////
	DeclareState(STATE_SampleTestDijkstra)

		OnEnter
			g_tests.RunSampleTest("Samples\\sample.txt", "outcome.txt", *this, 50, 0, 0.0f);
			ChangeState(STATE_Idle);

	///////////////////////////////////////////////////////////////
	DeclareState(STATE_SampleTestAStarEuclidean)

		OnEnter
			g_tests.RunSampleTest("Samples\\sample.txt", "outcome.txt", *this, 50, 0, 1.0f);
			ChangeState(STATE_Idle);

	///////////////////////////////////////////////////////////////
	DeclareState(STATE_SampleTestAStarOctile)

		OnEnter
			g_tests.RunSampleTest("Samples\\sample.txt", "outcome.txt", *this, 50, 1, 1.01f);
			ChangeState(STATE_Idle);

	///////////////////////////////////////////////////////////////
	DeclareState(STATE_SampleTestShortDijkstra)

		OnEnter
			g_tests.RunSampleTest("Samples\\sampleshort.txt", "outcome.txt", *this, 50, 0, 0.0f);
			ChangeState(STATE_Idle);

	///////////////////////////////////////////////////////////////
	DeclareState(STATE_SampleTestShortAStarEuclidean)

		OnEnter
			g_tests.RunSampleTest("Samples\\sampleshort.txt", "outcome.txt", *this, 50, 0, 1.0f);
			ChangeState(STATE_Idle);

	///////////////////////////////////////////////////////////////
	DeclareState(STATE_SampleTestShortAStarOctile)

		OnEnter
			g_tests.RunSampleTest("Samples\\sampleshort.txt", "outcome.txt", *this, 50, 1, 1.01f);
			ChangeState(STATE_Idle);

	///////////////////////////////////////////////////////////////
	DeclareState(STATE_SampleTestDiagonal)

		OnEnter
			g_tests.GetSampleTestResult("Samples\\samplediagonal.txt", *this, true, false, false, false, FLS_DIAGONAL);
			ChangeState(STATE_Idle);

	///////////////////////////////////////////////////////////////
	DeclareState(STATE_SampleTestStraightline)

		OnEnter
			g_tests.GetSampleTestResult("Samples\\samplestraightline.txt", *this, true, true, false, false, FLS_STRAIGHTLINE);
			ChangeState(STATE_Idle);

	///////////////////////////////////////////////////////////////
	DeclareState(STATE_SampleTestRubberbanding)

		OnEnter
			g_tests.GetSampleTestResult("Samples\\samplerubberbandsmooth.txt", *this, true, false, true, false, FLS_RUBBERSMOOTH);
			ChangeState(STATE_Idle);

	///////////////////////////////////////////////////////////////
	DeclareState(STATE_SampleTestSmoothing)

		OnEnter
			g_tests.GetSampleTestResult("Samples\\samplerubberbandsmooth.txt", *this, true, false, false, true, FLS_RUBBERSMOOTH);
			ChangeState(STATE_Idle);

	///////////////////////////////////////////////////////////////
	DeclareState(STATE_SampleTestRubberbandSmooth)

		OnEnter
			g_tests.GetSampleTestResult("Samples\\samplerubberbandsmooth.txt", *this, true, false, true, true, FLS_RUBBERSMOOTH);
			ChangeState(STATE_Idle);

	///////////////////////////////////////////////////////////////
	DeclareState(STATE_GetSampleTest1)

		OnEnter
			g_tests.GetSampleTestResult("outcome.txt", *this, true, false, false, false, FLS_OUTCOME);
			ChangeState(STATE_Idle);

	///////////////////////////////////////////////////////////////
	DeclareState(STATE_GetSampleTest2)

		OnEnter
			g_tests.GetSampleTestResult("outcome.txt", *this, false, false, false, false, FLS_OUTCOME);
			ChangeState(STATE_Idle);

	///////////////////////////////////////////////////////////////
	DeclareState(STATE_DefaultTest1)

		OnEnter
			g_tests.RunTest(1, 38, 36, 1, 0, "TimingResultsLong.txt", *this, 0);
			ChangeState(STATE_Idle);

	///////////////////////////////////////////////////////////////
	DeclareState(STATE_DefaultTest2)

		OnEnter
			g_tests.RunTest(1, 38, 36, 1, 0, "TimingResultsLong.txt", *this, 1);
			ChangeState(STATE_Idle);

	///////////////////////////////////////////////////////////////
	DeclareState(STATE_AllTests)

		OnEnter
			g_tests.RunAllTests("test", *this);
			ChangeState(STATE_Idle);

	///////////////////////////////////////////////////////////////
	DeclareState(STATE_CreateSampleTests)

		OnEnter
			g_tests.CreateSampleTest("test", "sample", 0, 20, 10, 0);
			g_tests.CreateSampleTest("test", "sample", 1, 200, 100, 10);
			g_tests.CreateSampleTest("test", "sample", 2, 200, 100, 0);
			g_tests.CreateSampleTest("test", "sample", 3, 50, 25, 0);
			g_tests.CreateSampleTest("test", "sample", 4, 50, 25, 0);
			g_tests.CreateSampleTest("test", "sample", 5, 50, 25, 5);
			ChangeState(STATE_Idle);

EndStateMachine
}

void Agent::RunTimingTest(int mapIndex, int xStart, int yStart, int xGoal, int yGoal, std::string filename)
{
	Movement& movement = m_owner->GetMovement();
	int heuristic = movement.GetHeuristicCalc();
	float weight = movement.GetHeuristicWeight();
	bool step = movement.GetSingleStep();
	bool smoothing = movement.GetSmoothPath();
	bool rubberband = movement.GetRubberbandPath();
	bool straightline = movement.GetStraightlinePath();
	bool analysis = movement.GetAnalysis();

	movement.SetHeuristicCalc(1); // Should be Octile
	movement.SetHeuristicWeight(1.01f);
	movement.SetSingleStep(false);
	movement.SetSmoothPath(false);
	movement.SetRubberbandPath(false);
	movement.SetStraightlinePath(false);
	movement.AStarUsesAnalysis(false);

	while (g_terrain.GetMapIndex() != mapIndex)
	{
		g_terrain.NextMap();
	}
			
	m_owner->GetBody().SetPos(g_terrain.GetCoordinates(xStart, yStart));            
						
	double results[NUMTESTS];
	double resultsTime[NUMTESTS];

	g_clock.UpdateQPCFrequency();

	// Run tests
	for (int i = 0; i < NUMTESTS; ++i)
	{
		movement.ComputePathWithTiming(xGoal, yGoal, true);
		results[i] = g_clock.GetStopwatchPathfinding();
		resultsTime[i] = g_clock.GetStopwatchPathfindingTime();
	}

	// Find minimum
	double min = results[0];
	double minTime = resultsTime[0];
	for (int i = 1; i < NUMTESTS; ++i)
	{
		if (min > results[i])
		{
			min = results[i];
			minTime = resultsTime[i];
		}
	}

	// Output results
	std::ofstream out(filename.c_str());

	out << std::endl << "Minimum time:  " << min << "\t(" << minTime << " ms)" << std::endl << std::endl;
	out << "Test #\t\t" << "Time" << std::endl;

	for (int i = 0; i < NUMTESTS; ++i)
		out << i + 1 << ":\t\t" << results[i] << "\t(" << resultsTime[i] << " ms)" << std::endl;

	out.close();

	movement.SetHeuristicCalc(heuristic);
	movement.SetHeuristicWeight(weight);
	movement.SetSingleStep(step);
	movement.SetSmoothPath(smoothing);
	movement.SetRubberbandPath(rubberband);
	movement.AStarUsesAnalysis(analysis);
	movement.SetStraightlinePath(straightline);
}



