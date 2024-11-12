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

Trigger::Trigger( objectID owner, objectID attached, D3DXVECTOR2 pos, float radius, MSG_Name msgToOwner, MSG_Name msgToTarget, float periodicDelay, bool persistAfterFiring, unsigned int affectedType, float expirationTime )
: m_owner( owner ),
  m_attached( attached ),
  m_pos( pos ),
  m_radiusSquared( radius * radius ),
  m_msgToOwner( msgToOwner ),
  m_msgToTarget( msgToTarget ),
  m_periodicDelay( periodicDelay ),
  m_persistAfterFiring( persistAfterFiring ),
  m_affectedType( affectedType ),
  m_expirationTime( expirationTime )
{
	m_nextTimeToRun = m_periodicDelay + g_clock.GetCurTime();
	m_markedForDeletion = false;
}

TriggerSystem::TriggerSystem(void)
{
}

TriggerSystem::~TriggerSystem(void)
{
}

void TriggerSystem::Update( void )
{
	float currentTime = g_clock.GetCurTime();

	//Delete triggers that are marked for deletion or expired
	TriggerList::iterator i = m_triggerList.begin();
	while( i != m_triggerList.end() )
	{
		if( (*i)->m_markedForDeletion ||			//Marked for deletion
			currentTime > (*i)->m_expirationTime)	//Expired
		{	//Destroy trigger
			delete( *i );
			i = m_triggerList.erase( i );
		}
		else
		{
			++i;
		}
	}

	for( TriggerList::iterator trigger = m_triggerList.begin(); trigger != m_triggerList.end(); ++trigger )
	{
		if( (*trigger)->m_nextTimeToRun < currentTime )
		{
			GameObject* ownerGameObject = g_database.Find( (*trigger)->m_owner );
			if( ownerGameObject == 0 )
			{	//Trigger's owner no longer exists - delete trigger when safe
				(*trigger)->m_markedForDeletion = true;
				continue;
			}

			dbCompositionList list;
			g_database.ComposeList( list, (*trigger)->m_affectedType );
		
			if( (*trigger)->m_attached != INVALID_OBJECT_ID )
			{	//Set dynamic position
				GameObject* attachedGameObject = g_database.Find( (*trigger)->m_attached );
				if( attachedGameObject == 0 )
				{	//Trigger attached to game object that no longer exists - delete trigger when safe
					(*trigger)->m_markedForDeletion = true;
					continue;
				}
				(*trigger)->m_pos = attachedGameObject->GetBody().GetPos();
			}

			for( dbCompositionList::iterator i = list.begin(); i != list.end(); ++i )
			{
				GameObject* targetGameObject = (*i);
				D3DXVECTOR3 targetPos = targetGameObject->GetBody().GetPos();
				D3DXVECTOR3 triggerPos = (*trigger)->m_pos;

				float distSquared = (targetPos.x * triggerPos.x) + (targetPos.y * triggerPos.y) + (targetPos.z * triggerPos.z);
				if( distSquared <= (*trigger)->m_radiusSquared )
				{	//Collision
					if( MSG_INVALID != (*trigger)->m_msgToTarget )
					{	//Send message to Target
						g_database.SendMsgFromSystem( targetGameObject, (*trigger)->m_msgToTarget );
					}
					if( MSG_INVALID != (*trigger)->m_msgToOwner )
					{	//Send message to Owner
						g_database.SendMsgFromSystem( ownerGameObject, (*trigger)->m_msgToOwner );
					}

					if( !(*trigger)->m_persistAfterFiring )
					{
						(*trigger)->m_markedForDeletion = true;
					}
				}
			}

			//Set the next time this trigger should run
			(*trigger)->m_nextTimeToRun = (*trigger)->m_periodicDelay + currentTime;
		}
	}
}

void TriggerSystem::Add( Trigger& trigger )
{
	m_triggerList.push_front( &trigger );
}

void TriggerSystem::SendMsgRadius( float radius, float delay, MSG_Name name, objectID sender, unsigned int targetGameObjectType, MSG_Data& data )
{
	GameObject* senderGameObject = g_database.Find( sender );
	if( senderGameObject != INVALID_OBJECT_ID )
	{	//The sender is a valid game object
		D3DXVECTOR3 senderPos = senderGameObject->GetBody().GetPos();
		float radiusSquared = radius * radius;	//Precompute the radius squared

		dbCompositionList list;
		g_database.ComposeList( list, targetGameObjectType );

		for( dbCompositionList::iterator i = list.begin(); i != list.end(); ++i )
		{	//Loop through all game objects of a given type
			GameObject* targetGameObject = (*i);
			if(targetGameObject != senderGameObject)
			{	//Not the sender
				D3DXVECTOR3 targetPos = targetGameObject->GetBody().GetPos();
				float distSquared = (targetPos.x * senderPos.x) + (targetPos.y * senderPos.y) + (targetPos.z * senderPos.z);

				if( distSquared <= radiusSquared && targetGameObject != senderGameObject)
				{	//Close enough - send the message
					g_msgroute.SendMsg( delay, name, SYSTEM_OBJECT_ID, targetGameObject->GetID(), SCOPE_TO_STATE_MACHINE, 0, STATE_MACHINE_QUEUE_ALL, data, false, false );
				}
			}
		}
	}
}