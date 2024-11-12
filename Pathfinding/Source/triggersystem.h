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

class Trigger
{
public:
	Trigger( objectID owner, objectID attached, D3DXVECTOR2 pos, float radius, MSG_Name msgToOwner, MSG_Name msgToTarget, float periodicDelay, bool persistAfterFiring, unsigned int affectedType, float expirationTime );

	objectID m_owner;			//Owner who can destroy it and who gets notification on trigger
	objectID m_attached;			//If valid objectID, then it is attached to this game object
	D3DXVECTOR3 m_pos;			//If not attached, then it exist at this position
	float m_radiusSquared;			//Radius (squared) of trigger
	MSG_Name m_msgToOwner;			//Message to send owner when trigger fires
	MSG_Name m_msgToTarget;			//Message to send target when trigger fires
	bool m_persistAfterFiring;		//Whether trigger continues to exist after firing (can cause many duplicate messages)
	unsigned int m_affectedType;		//Type of object that this trigger applies to
	float m_periodicDelay;			//Frequency (in seconds) that the trigger should be run
	float m_nextTimeToRun;			//The time at which this trigger should be run
	float m_expirationTime;			//The time at which this trigger should automatically expire
	bool m_markedForDeletion;		//Whether this trigger should be deleted (when safe to do so)

	inline void Delete( void )	{ m_markedForDeletion = true; }
};


class TriggerSystem
{
public:
	TriggerSystem( void );
	~TriggerSystem( void );

	void Update( void );
	void Add( Trigger& trigger );

	void SendMsgRadius( float radius, float delay, MSG_Name name, objectID sender, unsigned int targetGameObjectType, MSG_Data& data = MSG_Data() );

private:
	typedef std::list<Trigger*> TriggerList;

	TriggerList m_triggerList;

};
