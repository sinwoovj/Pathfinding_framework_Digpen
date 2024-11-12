#pragma once

#pragma warning(disable: 4005)
#pragma warning(disable: 4995) 

#include <DXUT/DXUT.h>
#include <DXUT/DXUTcamera.h>
#include <DXUT/DXUTenum.h>
#include <DXUT/DXUTgui.h>
#include <DXUT/DXUTmisc.h>
#include <DXUT/DXUTres.h>
#include <DXUT/DXUTsettingsdlg.h>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <xinput.h>

#include <io.h>
#include <fcntl.h>

#include <ctime>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <unordered_map>

#include <global.h>
#include <singleton.h>
#include <Random.h>
#include <MultiAnimation.h>

// Resource
#include <resource.h>

// Animation
#include <Tiny.h>

// Game Engine
#include <Blackboard.h>
#include <database.h>
#include <Map.h>
#include <terrain.h>
#include <Clock.h>
#include <triggersystem.h>
#include <world.h>				// project 2 only

// Game Object
#include <body.h>
#include <gameobject.h>
#include <movement.h>			// project 1 only

// State Machine Language
#include <debuglog.h>
#include <msg.h>
#include <msgroute.h>
#include <statemch.h>

// Unit test state machines
#include <agent.h>
#include <Enemy.h>
//#include <UnitTests/unittest1.h>
//#include <UnitTests/unittest2a.h>
//#include <UnitTests/unittest2b.h>
//#include <UnitTests/unittest2c.h>
//#include <UnitTests/unittest3a.h>
//#include <UnitTests/unittest3b.h>
//#include <UnitTests/unittest4.h>
//#include <UnitTests/unittest5.h>
//#include <UnitTests/unittest6.h>

#include <PathfindingTests.h>

#pragma warning(default: 4005)
#pragma warning(default: 4995)