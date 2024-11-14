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
#include <functional>

bool Movement::CheckStraightLine(int curR, int curC, int resR, int resC, bool expDir, int expR, int expC)
{
	auto checkWalls = [](int startX, int endX, int startY, int endY) {
		if (startX > endX) std::swap(startX, endX);
		if (startY > endY) std::swap(startY, endY);
		for (int x = startX; x <= endX; x++)
		{
			for (int y = startY; y <= endY; y++)
			{
				if (g_terrain.IsWall(x, y))
				{
					return false;
				}
			}
		}
		return true;
	};

	if (expDir)
	{
		// Check vertical line at expR
		if (!checkWalls(expR, expR, curC, resC)) return false;

		// Check horizontal line at expC
		if (!checkWalls(curR, resR, expC, expC)) return false;

		// Check area from expR to resR and expC to resC
		if (!checkWalls(expR, resR, expC, resC)) return false;
	}
	else
	{
		// Check entire area from curR to resR and curC to resC
		if (!checkWalls(curR, resR, curC, resC)) return false;
	}

	return true;
}

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
void Movement::Rubberbanding(std::list<D3DXVECTOR3>& _coordList)
{
	if (_coordList.size() < 3)
		return;

	std::list<D3DXVECTOR3>::iterator it = _coordList.begin();
	std::list<D3DXVECTOR3>::iterator nodeIt[3];

	for (int i = 0; i < 3; ++i)
	{
		nodeIt[i] = it++;
	}

	while (true)
	{
		int node[3][2] = {};
		g_terrain.GetRowColumn(&(*nodeIt[0]), &node[0][0], &node[0][1]);
		g_terrain.GetRowColumn(&(*nodeIt[1]), &node[1][0], &node[1][1]);
		g_terrain.GetRowColumn(&(*nodeIt[2]), &node[2][0], &node[2][1]);

		if (CheckStraightLine(node[0][0], node[0][1], node[2][0], node[2][1], false, node[1][0], node[1][1]))
		{
			_coordList.erase(nodeIt[1]);
			nodeIt[1] = nodeIt[2];
			nodeIt[2]++;
		}
		else
		{
			for (int i = 0; i < 3; ++i)
			{
				nodeIt[i]++;
			}
		}

		if (nodeIt[2] == _coordList.end())
			break;
	}
}

D3DXVECTOR3 CalculateMidpoint(const D3DXVECTOR3& point1, const D3DXVECTOR3& point2)
{
	return D3DXVECTOR3(
		(point1.x + point2.x) / 2.0f,
		0,
		(point1.z + point2.z) / 2.0f
	);
}




void Movement::Smoothing(std::list<D3DXVECTOR3>& _coordList)
{
	std::list<D3DXVECTOR3> res = _coordList;
	std::list<D3DXVECTOR3>::iterator it = _coordList.begin();
	std::list<D3DXVECTOR3>::iterator it_res = res.begin();
	//�� �� ������ �Ÿ��� 1.5�̻� ���̰� ���� ���ο� ���� ��������� ����
	using MyFunc = std::function<bool(std::list<D3DXVECTOR3>::iterator&, std::list<D3DXVECTOR3>::iterator, std::list<D3DXVECTOR3>&)>;
	MyFunc SmoothingDistance;
	SmoothingDistance = [&SmoothingDistance](std::list<D3DXVECTOR3>::iterator& a, std::list<D3DXVECTOR3>::iterator b, std::list<D3DXVECTOR3>& list)
	{
		if (list.size() < 2 || a == list.end() || b == list.end())
			return false;
		float lenf = std::sqrt(((*a).x - (*b).x) * ((*a).x - (*b).x) + ((*a).z - (*b).z) * ((*a).z - (*b).z));
		float standard = (1.f / g_terrain.GetWidth() / 2.f) * 3.f;
		if (lenf > standard)
		{
			D3DXVECTOR3 mid = CalculateMidpoint(*a, *b);
			std::list<D3DXVECTOR3>::iterator it_mid = list.insert(std::next(a), mid);
			SmoothingDistance(a, it_mid, list);
			SmoothingDistance(it_mid, b, list);
		}
		return true;
	};

	//3���� �� �̸��̸� ����ó��
	SmoothingDistance(it_res, std::next(it_res), res);
	if (res.size() >= 3)
	{
		//��� ��������Ʈ ��Ʈ ���̿� 3���� ���ο� ����Ʈ�� �����ϰ� ��Ȱȭ�� ����� ���� ����
		using MyFunc2 = std::function<bool(
			std::list<D3DXVECTOR3>::iterator&, std::list<D3DXVECTOR3>::iterator&, 
			std::list<D3DXVECTOR3>::iterator&, std::list<D3DXVECTOR3>::iterator&, 
			std::list<D3DXVECTOR3>::iterator, std::list<D3DXVECTOR3>&)>;
		MyFunc2 CreateSmoothingList;
		CreateSmoothingList = [&CreateSmoothingList](
			std::list<D3DXVECTOR3>::iterator& a, std::list<D3DXVECTOR3>::iterator b, 
			std::list<D3DXVECTOR3>::iterator& c, std::list<D3DXVECTOR3>::iterator& d, 
			std::list<D3DXVECTOR3>::iterator& e, std::list<D3DXVECTOR3>& list)
		{
			//mid�� 3�� ����� catmull �ϱ� �׸��� ����Ʈ��
			//D3DXVec3CatmullRom(&(*it), &(*it), &(*it), &(*it), &(*it), 0.25);
			return true;
		};
	}
	_coordList.clear();
	_coordList.assign(res.begin(), res.end());
}

bool Movement::ComputePath( int r, int c, bool newRequest )
{
	m_goal = g_terrain.GetCoordinates(r, c);
	m_movementMode = MOVEMENT_WAYPOINT_LIST;
	int m_width = g_terrain.GetWidth();
	int curR, curC;
	D3DXVECTOR3 cur = m_owner->GetBody().GetPos();
	g_terrain.GetRowColumn(&cur, &curR, &curC);

	//A* Initialization Variable
	static std::priority_queue<Node*, std::vector<Node*>, CompareNode> open_list;
	static std::unordered_set<Node*> close_list;
	static std::unordered_set<std::string> visited; // �湮�� ��� ����
	float given, heuristic, cost;

	//Initialization
	if (newRequest)
	{
		if (GetStraightlinePath()) // Straight
		{
			if (!CheckStraightLine(curR, curC, r, c))
				goto OutStraight;
			//���� ��� straight ����
			m_waypointList.clear();
			m_waypointList.push_back(cur);
			if (curR != r || curC != c)
				m_waypointList.push_back(m_goal);
			return true;
		}
	OutStraight:
		m_waypointList.clear();
		if (curR == r && curC == c)
		{
			m_waypointList.push_back(cur);
			return true;
		}

		//Push Start Node onto the Open List
		given = 0;
		heuristic = GetHeuristicResult(curR, curC, r, c);

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

		cost = GetCost(given, heuristic, GetHeuristicWeight());
		open_list.push(new Node{ curR, curC, cost, given, Node::STATUS::open });
		visited.insert(GetPosKey(curR, curC));
	}
	

	bool useAStar = true;

	if( useAStar )
	{
		//While (Open List is not empty) {
		while (!open_list.empty())
		{
			//If node is the Goal Node, then path found (RETURN ��found��)
			if (open_list.top()->x == r && open_list.top()->y == c) { 
				break; 
			}
			
			Movement::Node* parentNode = open_list.top();
			open_list.pop();

			for (const auto& dir : directions) {
				bool diagonal = false;
				int newX = parentNode->x + dir.x;
				int newY = parentNode->y + dir.y;
				//�ش� ��ǥ�� ��ȿ���� üũ (�� �ȿ� �ִ���, ���� ��������)
				if (IsTileInvaild(newX, newY, m_width))
					continue;
				if (g_terrain.IsWall(newX, newY))
					continue;

				// �밢�� ���⿡ ���� �˻�
				if (dir.x != 0 && dir.y != 0) {  // �밢���� ���
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
				//�ش� ��ǥ�� �湮����(üũ����)�� �ִ���
				if (visited.find(GetPosKey(newX, newY)) != visited.end())
					continue;
				//	Compute its cost, f(x) = g(x) + h(x)
				given = parentNode->given + (diagonal ? std::sqrtf(2) : 1);
				heuristic = GetHeuristicResult(newX, newY, r, c);
				cost = GetCost(given, heuristic, GetHeuristicWeight());
				open_list.push(new Node{ newX , newY, cost, given, Node::STATUS::open, parentNode });
				g_terrain.SetColor(newX, newY, DEBUG_COLOR_BLUE);
				//	If child node isn��t on Open or Closed list, put it on Visited List.
				visited.insert(GetPosKey(newX, newY));
			}
			//	If child node is on Open or Closed List, AND this new one is cheaper,
			//	then take the old expensive one off both lists and put this new cheaper
			//	one on the Open List.
			if(!g_terrain.IsWall(parentNode->x, parentNode->y))
				g_terrain.SetColor(parentNode->x, parentNode->y, DEBUG_COLOR_YELLOW);
			close_list.insert(parentNode);
			if (GetSingleStep())
				return false;
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
		
		if (GetRubberbandPath())
		{
			Rubberbanding(m_waypointList);
		}

		if (GetSmoothPath())
		{
			Smoothing(m_waypointList);
		}

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
