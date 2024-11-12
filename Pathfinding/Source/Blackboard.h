/******************************************************************************/
/*!
\file		Blackboard.h
\project	CS380/CS580 AI Framework
\author		Chi-Hao Kuo
\summary	Global blackboard.

Copyright (C) 2016 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once

class Movement;

class MovementSetting
{
public:
	// constructor
	MovementSetting();

	// set movementdata from movement
	void Set(Movement& movement);

	// reset data from MovementData back to Movement
	void Reset(Movement& movement);

	// set pos data
	void SetPos(int row, int col);

	// get pos data
	void GetPos(int &row, int &col);

	float GetHeuristicWeight(void)		{ return m_heuristicWeight; }
	int GetHeuristicCalc(void)			{ return m_heuristicCalc; }
	bool GetSingleStep(void)			{ return m_singleStep; }
	bool GetSmoothing(void)				{ return m_smooth; }
	bool GetRubberbanding(void)			{ return m_rubberband; }
	bool GetStraightLine(void)			{ return m_straightline; }
	int GetExtraCredit(void)			{ return m_extracredit; }
	bool GetAnalysis(void)				{ return m_aStarUsesAnalysis; }

private:
	int m_row;
	int m_col;
	int m_mapindex;

	float m_heuristicWeight;
	int m_heuristicCalc;
	bool m_singleStep;
	bool m_smooth;
	bool m_rubberband;
	bool m_straightline;
	int m_extracredit;
	bool m_aStarUsesAnalysis;
};

class BlackBoard
{
public:
	BlackBoard();

	void UpdatePlayerPos(int r, int c);
	void UpdateStartPos(void);
	void UpdateGoalPos(int r, int c);

	int GetRowPlayer(void)								{ return m_rPlayer; }
	int GetColPlayer(void)								{ return m_cPlayer; }
	int GetRowStart(void)								{ return m_rStart; }
	int GetColStart(void)								{ return m_cStart; }
	int GetRowGoal(void)								{ return m_rGoal; }
	int GetColGoal(void)								{ return m_cGoal; }

	void ToggleTerrainAnalysisFlag(void)				{ m_terrainAnalysisFlag = !m_terrainAnalysisFlag; }
	void SetTerrainAnalysisType(int type);
	void IncTerrainAnalysisType(void);
	void SetDecayFactor(float factor);
	void SetGrowFactor(float factor);
	void SetUpdateFrequency(float frequency);
	void SetFOVAngle(float angle)						{ m_fovAngle = angle; }
	void SetFOVCloseDistance(float distance)			{ m_fovCloseDistance = distance; }
	void SetSeekPlayerFlag(bool flag)					{ m_seekPlayer = flag; }
	void SetSampleTestStatus(SampleTestStatus status)	{ m_sampleTestStatus = status; }


	bool GetTerrainAnalysisFlag(void)					{ return m_terrainAnalysisFlag; }
	int GetTerrainAnalysisType(void)					{ return m_terrainAnalysisType; }
	float GetDecayFactor(void)							{ return m_decayFactor; }
	float GetGrowFactor(void)							{ return m_growFactor; }
	float GetUpdateFrequency(void)						{ return m_updateFrequency; }
	float GetFOVAngle(void)								{ return m_fovAngle; }
	float GetFOVCloseDistance(void)						{ return m_fovCloseDistance; }
	bool GetSeekPlayerFlag(void)						{ return m_seekPlayer; }
	SampleTestStatus GetSampleTestStatus(void)			{ return m_sampleTestStatus; }

	MovementSetting &GetMovementSetting(void)			{ return m_movementSetting; }

private:
	int	m_rPlayer;		// player row position
	int	m_cPlayer;		// player col position
	int	m_rStart;		// start row in A* (player)
	int	m_cStart;		// start col in A* (player)
	int	m_rGoal;		// goal row in A* (player)
	int	m_cGoal;		// goal col in A* (player)

	bool m_terrainAnalysisFlag;		// toggle terrain analysis on/off
	int m_terrainAnalysisType;		// terrain analysis type
	float m_decayFactor;			// decay factor
	float m_growFactor;				// grow factor
	float m_updateFrequency;		// update frequency for propagation
	float m_fovAngle;				// FOV angle (hide and seek)
	float m_fovCloseDistance;		// FOV close distance (hide and seek)
	bool m_seekPlayer;				// flag: is enemy seeking player (hide and seek)

	MovementSetting m_movementSetting;
	SampleTestStatus m_sampleTestStatus;
};