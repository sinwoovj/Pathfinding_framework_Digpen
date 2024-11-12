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

class Agent : public StateMachine
{
	friend class PathfindingTests;

public:

	Agent( GameObject & object )
		: StateMachine( object ), m_moving(true) {}
	~Agent( void ) {}


private:

	virtual bool States( State_Machine_Event event, MSG_Object * msg, int state, int substate );

	void RunTimingTest(int mapIndex, int xStart, int yStart, int xGoal, int yGoal, std::string filename);
	void InitFogOfWar(bool);

	//Put member variables or functions here
	int m_animStyle;	//0=walk, 1=jog
	int m_rowGoal;
	int m_colGoal;
	bool m_moving;
};
