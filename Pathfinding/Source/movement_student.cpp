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
#include <queue>
#include <unordered_set>
#include <vector>
#include <string>
#include <list>
#include <algorithm>


bool Movement::IsTileInvaild(int x, int y, int m_width)
{
	return ((x < 0) || (x > m_width - 1) || (y < 0) || (y > m_width - 1));
}

std::string Movement::GetPosKey(int x, int y)
{
	return std::to_string(x) + "," + std::to_string(y);
}

/*
 euclidean   = sprt(x^2+y^2)
 octile      = min(x,y) * sqrt(2) + max(x,y) - min(x,y)
 chebyshev   = max(x,y)
 manhattan   = x + y
 */
float Movement::GetHeuristicResult(int c_x, int c_y, int goal_x, int goal_y)
{
	float diff_x = float(std::abs(goal_x - c_x));
	float diff_y = float(std::abs(goal_y - c_y));
	float min = std::fminf(diff_x, diff_y);
	float max = std::fmaxf(diff_x, diff_y);
	switch (GetHeuristicCalc())
	{
	case 0: // euclidean
		return std::sqrt((diff_x * diff_x) + (diff_y * diff_y));
	case 1: // octile
		return min * sqrtf(2) + max - min;
	case 2: // chebyshev
		return max;
	case 3: // manhattan
		return diff_x + diff_y;
	}
	return -1.0f;
}

float Movement::GetCost(float g, float h, float w)
{
	return g + (h * w);
}

struct CompareNode {
	bool operator()(const Movement::Node* a, const Movement::Node* b) {
		return a->cost > b->cost; // Higher priority comes first
	}
};

bool Movement::ComputePath( int r, int c, bool newRequest )
{
	m_goal = g_terrain.GetCoordinates( r, c );
	m_movementMode = MOVEMENT_WAYPOINT_LIST;
	int m_width = g_terrain.GetWidth();
	std::priority_queue<Node*, std::vector<Node*>, CompareNode> open_list;
	std::unordered_set<Node*> close_list;
	std::unordered_set<std::string> visited; // 방문한 노드 추적
	// project 2: change this flag to true
	bool useAStar = true;

	if( useAStar )
	{
		int count = 0;
		int curR, curC;
		D3DXVECTOR3 cur = m_owner->GetBody().GetPos();
		g_terrain.GetRowColumn(&cur, &curR, &curC);

		m_waypointList.clear();
		if (curR == r && curC == c)
		{
			m_waypointList.push_back(cur);
			return true;
		}
		//m_waypointList.push_back(cur);
		///////////////////////////////////////////////////////////////////////////////////////////////////
		//INSERT YOUR A* CODE HERE
		//1. You should probably make your own A* class.
		//2. You will need to make this function remember the current progress if it preemptively exits.
		//3. Return "true" if the path is complete, otherwise "false".
		///////////////////////////////////////////////////////////////////////////////////////////////////
		//Push Start Node onto the Open List
		float given = 0;
		float heuristic = GetHeuristicResult(curR, curC, r, c);
		float cost = GetCost(given, heuristic, GetHeuristicWeight());
		open_list.push(new Node{ curR, curC, cost, given, Node::STATUS::open });
		visited.insert(GetPosKey(curR, curC));
		//While (Open List is not empty) {
		while (!open_list.empty())
		{
			/*if (count > 50000)
			{
				return true;
			}*/
			//If node is the Goal Node, then path found (RETURN “found”)
			if (open_list.top()->x == r && open_list.top()->y == c) { 
				break; 
			}
			
			Movement::Node* parentNode = open_list.top();
			open_list.pop();

			for (const auto& dir : directions) {
				bool diagonal = false;
				int newX = parentNode->x + dir.x;
				int newY = parentNode->y + dir.y;
				//해당 좌표가 유효한지 체크 (맵 안에 있는지, 벽에 막혔는지)
				if (IsTileInvaild(newX, newY, m_width))
					continue;
				if (g_terrain.IsWall(newX, newY))
					continue;

				// 대각선 방향에 대한 검사
				if (dir.x != 0 && dir.y != 0) {  // 대각선인 경우
					diagonal = true;

					if (IsTileInvaild(parentNode->x, newY, m_width) || IsTileInvaild(newX, parentNode->y, m_width))
					{
						continue;
					}
					else
					{
						if (g_terrain.IsWall(newX, parentNode->y) || g_terrain.IsWall(parentNode->x, newY))
						{
							continue;
						}
					}
				}
				//해당 좌표를 방문한적(체크한적)이 있는지
				if (visited.find(GetPosKey(newX, newY)) != visited.end())
					continue;
				//	Compute its cost, f(x) = g(x) + h(x)
				given = parentNode->given + (diagonal ? std::sqrtf(2) : 1);
				heuristic = GetHeuristicResult(newX, newY, r, c);
				cost = GetCost(given, heuristic, GetHeuristicWeight());
				open_list.push(new Node{ newX , newY, cost, given, Node::STATUS::open, parentNode });
				g_terrain.SetColor(newX, newY, DEBUG_COLOR_BLUE);
				//	If child node isn’t on Open or Closed list, put it on Visited List.
				visited.insert(GetPosKey(newX, newY));
			}
			//	If child node is on Open or Closed List, AND this new one is cheaper,
			//	then take the old expensive one off both lists and put this new cheaper
			//	one on the Open List.
			g_terrain.SetColor(parentNode->x, parentNode->y, DEBUG_COLOR_YELLOW);
			close_list.insert(parentNode);
			
			count++;
		}
		if (open_list.empty()) // not find path
		{
			m_waypointList.push_back(cur);
			return true;
		}
		Node* shortestPath = open_list.top();
		while (shortestPath->parent != nullptr)
		{
			D3DXVECTOR3 spot = g_terrain.GetCoordinates(shortestPath->x, shortestPath->y);
			m_waypointList.push_back(spot);
			shortestPath = shortestPath->parent;
		}
		m_waypointList.reverse();
		
		//delete Nodes
		while (!open_list.empty())
		{
			delete open_list.top();
			open_list.pop();
		}
		for (auto& it : close_list)
		{
			delete it;
		}
		close_list.clear();
		visited.clear();
		return true;
	}
	else
	{	
		//Randomly meander toward goal (might get stuck at wall)
		int curR, curC;
		D3DXVECTOR3 cur = m_owner->GetBody().GetPos();
		g_terrain.GetRowColumn( &cur, &curR, &curC );

		m_waypointList.clear();
		m_waypointList.push_back( cur );

		int countdown = 100;
		while( curR != r || curC != c )
		{
			if( countdown-- < 0 ) { break; }

			if( curC == c || (curR != r && rand()%2 == 0) )
			{	//Go in row direction
				int last = curR;
				if( curR < r ) { curR++; }
				else { curR--; }

				if( g_terrain.IsWall( curR, curC ) )
				{
					curR = last;
					continue;
				}
			}
			else
			{	//Go in column direction
				int last = curC;
				if( curC < c ) { curC++; }
				else { curC--; }

				if( g_terrain.IsWall( curR, curC ) )
				{
					curC = last;
					continue;
				}
			}

			D3DXVECTOR3 spot = g_terrain.GetCoordinates( curR, curC );
			m_waypointList.push_back( spot );
			g_terrain.SetColor( curR, curC, DEBUG_COLOR_YELLOW );
		}
		return true;
	}
}
