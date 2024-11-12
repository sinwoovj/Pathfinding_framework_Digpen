/******************************************************************************/
/*!
\file		Blackboard.cpp
\project	CS380/CS580 AI Framework
\author		Chi-Hao Kuo
\summary	Global blackboard.

Copyright (C) 2016 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <Stdafx.h>

// constructor
MovementSetting::MovementSetting()
	: m_row(0), m_col(0), m_mapindex(0), m_heuristicWeight(1.00f), m_heuristicCalc(0),
	m_singleStep(true), m_smooth(false), m_rubberband(false), m_straightline(false),
	m_extracredit(0), m_aStarUsesAnalysis(false)
{

}

void MovementSetting::Set(Movement& movement)
{
	m_heuristicCalc = movement.GetHeuristicCalc();
	m_heuristicWeight = movement.GetHeuristicWeight();
	m_singleStep = movement.GetSingleStep();
	m_smooth = movement.GetSmoothPath();
	m_rubberband = movement.GetRubberbandPath();
	m_straightline = movement.GetStraightlinePath();
	m_aStarUsesAnalysis = movement.GetAnalysis();
	m_extracredit = movement.GetExtraCredit();
}

void MovementSetting::Reset(Movement& movement)
{
	movement.SetHeuristicCalc(m_heuristicCalc);
	movement.SetHeuristicWeight(m_heuristicWeight);
	movement.SetSingleStep(m_singleStep);
	movement.SetSmoothPath(m_smooth);
	movement.SetRubberbandPath(m_rubberband);
	movement.AStarUsesAnalysis(m_aStarUsesAnalysis);
	movement.SetStraightlinePath(m_straightline);
	movement.SetExtraCredit(m_extracredit);
}

// set pos data
void MovementSetting::SetPos(int row, int col)
{
	m_row = row;
	m_col = col;

	m_mapindex = g_terrain.GetMapIndex();
}

// get pos data
void MovementSetting::GetPos(int &row, int &col)
{
	row = m_row;
	col = m_col;

	g_terrain.UpdateMap(m_mapindex);
}

BlackBoard::BlackBoard()
	: m_rPlayer(-1), m_cPlayer(-1), m_rStart(-1), m_cStart(-1), m_rGoal(-1), m_cGoal(-1),
	m_terrainAnalysisFlag(false), m_terrainAnalysisType(0), 
	m_decayFactor(DEFAULT_DECAY), m_growFactor(DEFAULT_GROWING), m_updateFrequency(DEFAULT_UPDATEFREQUENCY),
	m_fovAngle(DEFAULT_FOVANGLE), m_sampleTestStatus(Status_None),
	m_fovCloseDistance(DEFAULT_FOVCLOSEDISTANCE), m_seekPlayer(false)
{

}

void BlackBoard::UpdatePlayerPos(int r, int c)
{
	m_rPlayer = r;
	m_cPlayer = c;
}

void BlackBoard::UpdateStartPos(void)
{
	m_rStart = m_rPlayer;
	m_cStart = m_cPlayer;
}

void BlackBoard::UpdateGoalPos(int r, int c)
{
	m_rGoal = r;
	m_cGoal = c;
}

void BlackBoard::SetTerrainAnalysisType(int type)
{
#if defined(_DEBUG)
	ASSERTMSG(type >= 0 && type < TerrainAnalysis_Count, "Invalid Type");
#endif

	m_terrainAnalysisType = type;
}

void BlackBoard::IncTerrainAnalysisType(void)
{
	if (++m_terrainAnalysisType >= TerrainAnalysis_Count)
		m_terrainAnalysisType = 0;

	if (g_blackboard.GetTerrainAnalysisFlag())
	{
		g_terrain.SetTerrainAnalysis();
	}
}

void BlackBoard::SetDecayFactor(float factor)
{
#if defined(_DEBUG)
	ASSERTMSG(factor >= 0.0f && factor <= 1.0f, "Invalid Decay Factor");
#endif

	m_decayFactor = factor;
}

void BlackBoard::SetGrowFactor(float factor)
{
#if defined(_DEBUG)
	ASSERTMSG(factor >= 0.0f && factor <= 1.0f, "Invalid Grow Factor");
#endif

	m_growFactor = factor;
}

void BlackBoard::SetUpdateFrequency(float frequency)
{
#if defined(_DEBUG)
	ASSERTMSG(frequency >= 0.0f, "Invalid Update Frequency");
#endif

	m_updateFrequency = frequency;
}