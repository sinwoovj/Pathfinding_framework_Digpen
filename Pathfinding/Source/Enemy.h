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

#include <statemch.h>

static float REACT_TIME_IDLE = 0.4f;
static float REACT_TIME_SEEK = 0.1f;

enum EnemyState
{
	ENEMYSTATE_RunToPlayer,
	ENEMYSTATE_SeekPlayer,
	ENEMYSTATE_Patrol,
	ENEMYSTATE_Count
};

class Enemy : public StateMachine
{
public:

	Enemy(GameObject & object)
		: StateMachine(object), m_timer(0.0f), m_changeGoal(true), m_state(ENEMYSTATE_Patrol),
		m_rowGoal(-1), m_colGoal(-1)
		{}
	~Enemy(void) {}


private:

	virtual bool States( State_Machine_Event event, MSG_Object * msg, int state, int substate );

	void FieldOfView(float angle, float closeDistance, float occupancyValue);
	bool FindPlayer(void);
	bool SeekPlayer(void);

	bool UpdateTimer(float time);

	void GetPatrolGoal(int offset);
	bool HasOccupancy(void);
	void ChangeGoal(int row, int col);

	void ResetEnemy(void);

	//Put member variables or functions here
	float m_timer;			// countdown timer
	bool m_changeGoal;		// flag: if the goal is changed?
	EnemyState m_state;		// enemy state: run to player, seek player, patrol
	int m_rowGoal;			// goal row
	int m_colGoal;			// goal column
};
