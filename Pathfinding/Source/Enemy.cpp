// Author: Chi-Hao Kuo
// Updated: 12/25/2015

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
	STATE_RunToPlayer,
	STATE_SeekPlayer,
	STATE_Patrol,
	STATE_Idle,
};

//Add new substates here
enum SubstateName {
	//empty
};

bool Enemy::States(State_Machine_Event event, MSG_Object * msg, int state, int substate)
{
BeginStateMachine

	//Global message responses

	OnMsg( MSG_Reset )
		ResetStateMachine();
		ResetEnemy();

	OnMsg( MSG_SetGoalEnemy )
		int row = -1;
		int col = -1;
		if(g_terrain.GetRowColumn( &g_click3D, &row, &col ))
		{
			if( !g_terrain.IsWall( row, col ) )
			{ 
				g_terrain.ResetColors();

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

	OnMsg(MSG_SetGoalEnemyRunToPlayer)
		m_changeGoal = false;
		bool foundPath = m_owner->GetMovement().ComputePathWithTiming(m_rowGoal, m_colGoal, true);
		if (foundPath)
		{
			ChangeState(STATE_RunToPlayer);
		}
		else
		{
			ChangeState(STATE_CalcPath);
		}

	OnMsg(MSG_SetGoalEnemySeekPlayer)
		m_changeGoal = false;
		bool foundPath = m_owner->GetMovement().ComputePathWithTiming(m_rowGoal, m_colGoal, true);
		if (foundPath)
		{
			ChangeState(STATE_SeekPlayer);
		}
		else
		{
			ChangeState(STATE_CalcPath);
		}

	OnMsg(MSG_SetGoalEnemyPatrol)
		m_changeGoal = false;
		bool foundPath = m_owner->GetMovement().ComputePathWithTiming(m_rowGoal, m_colGoal, true);
		if (foundPath)
		{
			ChangeState(STATE_Patrol);
		}
		else
		{
			ChangeState(STATE_CalcPath);
		}

	///////////////////////////////////////////////////////////////
	DeclareState( STATE_Initialize )

		OnEnter
			m_owner->GetMovement().SetIdleSpeed();

			TerrainAnalysisType type = static_cast<TerrainAnalysisType>(g_blackboard.GetTerrainAnalysisType());
			if (g_blackboard.GetTerrainAnalysisFlag() && type == TerrainAnalysis_HideAndSeek)
				ChangeState(STATE_Idle);


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
			m_owner->GetMovement().SetJogSpeed();

		OnUpdate
		{
			FieldOfView(g_blackboard.GetFOVAngle(), g_blackboard.GetFOVCloseDistance(), -0.25f);
		}

		OnMsg(MSG_ArrivedEnemy)
			ChangeState(STATE_Initialize);


	///////////////////////////////////////////////////////////////
	DeclareState(STATE_RunToPlayer)

		OnEnter
			if (m_state != ENEMYSTATE_RunToPlayer)
			{
				g_terrain.ResetInfluenceMap();
			}
			m_state = ENEMYSTATE_RunToPlayer;
			m_owner->GetMovement().SetJogSpeed();

		OnUpdate
			{
				FieldOfView(g_blackboard.GetFOVAngle(), g_blackboard.GetFOVCloseDistance(), -0.5f);

				if (UpdateTimer(REACT_TIME_SEEK))
				{
					if (FindPlayer())
					{
						if (m_changeGoal)
							SendMsgDelayedToState(0.1f, MSG_SetGoalEnemyRunToPlayer);
					}
					else
						ChangeState(STATE_SeekPlayer);
				}
			}

		OnMsg(MSG_ArrivedEnemy)
			m_changeGoal = true;
			ChangeState(STATE_Idle);

	///////////////////////////////////////////////////////////////
	DeclareState(STATE_SeekPlayer)

		OnEnter
			if (m_state != ENEMYSTATE_SeekPlayer)
			{
				g_terrain.ResetInfluenceMap();

				m_changeGoal = true;

				g_terrain.InitialOccupancyMap(m_rowGoal, m_colGoal, 1.0f);

				// run propagation a few times to make sure it expands before being removed by FOV
				for (int i = 0; i < 3; ++i)
					g_terrain.Propagation(g_blackboard.GetDecayFactor(), g_blackboard.GetGrowFactor(), false);
				g_blackboard.SetSeekPlayerFlag(true);
			}

			m_state = ENEMYSTATE_SeekPlayer;
			m_owner->GetMovement().SetJogSpeed();

		OnUpdate
			{
				FieldOfView(g_blackboard.GetFOVAngle(), g_blackboard.GetFOVCloseDistance(), -0.5f);

				if (UpdateTimer(REACT_TIME_SEEK))
				{
					if (FindPlayer() && m_changeGoal)
					{
						SendMsgDelayedToState(0.1f, MSG_SetGoalEnemyRunToPlayer);
					}
					else if (SeekPlayer() && m_changeGoal)
					{
						SendMsgDelayedToState(0.1f, MSG_SetGoalEnemySeekPlayer);
					}
				}
			}

		OnMsg(MSG_ArrivedEnemy)
			m_changeGoal = true;
			ChangeState(STATE_Idle);

	///////////////////////////////////////////////////////////////
	DeclareState(STATE_Patrol)

		OnEnter
			if (m_state != ENEMYSTATE_Patrol)
			{
				g_terrain.ResetInfluenceMap();
			}
			m_state = ENEMYSTATE_Patrol;
			m_owner->GetMovement().SetWalkSpeed();

		OnUpdate
			{
				FieldOfView(g_blackboard.GetFOVAngle(), g_blackboard.GetFOVCloseDistance(), -0.25f);

				if (UpdateTimer(REACT_TIME_IDLE))
				{
					if (FindPlayer() && m_changeGoal)
					{
						SendMsgDelayedToState(0.1f, MSG_SetGoalEnemyRunToPlayer);
					}
				}
			}

		OnMsg(MSG_ArrivedEnemy)
			m_changeGoal = true;
			ChangeState(STATE_Idle);

	///////////////////////////////////////////////////////////////
	DeclareState( STATE_Idle )

		OnEnter
			m_owner->GetMovement().SetIdleSpeed();

		OnUpdate
		{
			if (m_state == ENEMYSTATE_Patrol)
			{
				FieldOfView(g_blackboard.GetFOVAngle(), g_blackboard.GetFOVCloseDistance(), -0.25f);

				if (UpdateTimer(REACT_TIME_IDLE))
				{
					if (FindPlayer())
						SendMsgDelayedToState(0.1f, MSG_SetGoalEnemyRunToPlayer);
					else 
					{
						GetPatrolGoal(5);
						SendMsgDelayedToState(0.1f, MSG_SetGoalEnemyPatrol);
					}
				}
			}
			else
			{
				FieldOfView(g_blackboard.GetFOVAngle(), g_blackboard.GetFOVCloseDistance(), -0.5f);

				if (UpdateTimer(REACT_TIME_SEEK))
				{
					if (FindPlayer())
					{
						SendMsgDelayedToState(0.1f, MSG_SetGoalEnemyRunToPlayer);
					}
					else if (m_state == ENEMYSTATE_SeekPlayer && SeekPlayer())
					{
						SendMsgDelayedToState(0.1f, MSG_SetGoalEnemySeekPlayer);
					}
					else
					{
						//m_changeGoal = true;

						if (!HasOccupancy())
						{
							GetPatrolGoal(5);
							SendMsgDelayedToState(0.1f, MSG_SetGoalEnemyPatrol);
						}
					}
				}
			}
		}


	///////////////////////////////////////////////////////////////

EndStateMachine
}

bool Enemy::UpdateTimer(float time)
{
	if (m_timer < 0.0f)
	{
		m_timer = time;
		return true;
	}
	else
	{
		m_timer -= g_clock.GetElapsedTime();
		return false;
	}
}

void Enemy::ResetEnemy(void)
{
	m_timer = 0.0f;
	m_rowGoal = -1;
	m_colGoal = -1;
	m_changeGoal = true;
	m_state = ENEMYSTATE_Patrol;
}

void Enemy::GetPatrolGoal(int offset)
{
	D3DXVECTOR3 pos = m_owner->GetBody().GetPos();
	int row, col;
	g_terrain.GetRowColumn(&pos, &row, &col);

	bool re_roll = true;

	while (re_roll)
	{
		int diff_r = g_random.RangeInt(-offset, offset);
		int diff_c = g_random.RangeInt(-offset, offset);
		int new_row = row + diff_r;
		int new_col = col + diff_c;

		if (new_row < 0 || new_row >= g_terrain.GetWidth() ||
			new_col < 0 || new_col >= g_terrain.GetWidth() ||
			(row == new_row && col == new_col) || g_terrain.IsWall(new_row, new_col))
		{
			re_roll = true;
		}
		else
		{
			m_rowGoal = new_row;
			m_colGoal = new_col;

			return;
		}
	}


}

bool Enemy::HasOccupancy(void)
{
	for (int r = 0; r < g_terrain.GetWidth(); ++r)
	{
		for (int c = 0; c < g_terrain.GetWidth(); ++c)
		{
			if (!g_terrain.IsWall(r, c))
			{
				if (g_terrain.GetInfluenceMapValue(r, c) > 0.3f)
				{
					return true;
				}
			}
		}
	}

	return false;
}

void Enemy::ChangeGoal(int row, int col)
{
	m_rowGoal = row;
	m_colGoal = col;
	m_changeGoal = true;
}
