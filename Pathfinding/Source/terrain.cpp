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

Terrain::Terrain( void )
: m_nextMap(0),
  m_rPlayer(-1),
  m_cPlayer(-1),
  m_reevaluateAnalysis(false),
  m_timerUpdatePropagation(DEFAULT_UPDATEFREQUENCY),
  m_width(40)
{
	m_maps.push_back(Map(m_width));
}

Terrain::~Terrain()
{
	for (MapList::iterator i = m_maps.begin(); i != m_maps.end(); ++i)
		i->Destroy();
}

void Terrain::Create( void )
{
	NextMap();

	// Create new maps from Maps directory
	WCHAR pathBuffer[MAX_PATH];
	GetCurrentDirectory(MAX_PATH-1, pathBuffer);
	std::wstring path(pathBuffer);
	path += L"\\Maps\\";

	WIN32_FIND_DATA file;
	HANDLE h = FindFirstFile((std::wstring(path) += L"*.txt").c_str(), &file);

	if (!h)
		return;

	do 
	{
		if ((file.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0)
			continue;
		m_maps.push_back(Map());
		m_maps.back().Serialize(std::wstring(path) += file.cFileName);
	}
	while (FindNextFile(h, &file));

	FindClose(h);
}

void Terrain::NextMap( void )
{
	assert(m_maps.size() && "No maps loaded");
	m_nextMap = ++m_nextMap % m_maps.size();

	Map& map = m_maps[m_nextMap];
	m_width = map.GetWidth();
	m_terrain = map.GetTerrain();
	m_terrainColor = map.GetTerrainColor();
	m_terrainInfluenceMap = map.GetInfluenceMap();
	m_timerUpdatePropagation = DEFAULT_UPDATEFREQUENCY;

	ResetColors();
	ResetInfluenceMap();
	Analyze();

	g_database.SendMsgFromSystem(MSG_MapChange);
}

Map *Terrain::GetCurrentMap(void) 
{ 
	if (static_cast<unsigned>(m_nextMap) > m_maps.size()) 
		return NULL; 
	
	return &m_maps[m_nextMap]; 
}

D3DXVECTOR3 Terrain::GetCoordinates( int r, int c )
{
	D3DXVECTOR3 pos;

	const float offset = 1.f / m_width / 2.f;

	pos.x = ((float)c/(float)m_width) + offset;
	pos.y = 0.0f;
	pos.z = ((float)r/(float)m_width) + offset;
	
	return( pos );
}

bool Terrain::GetRowColumn( D3DXVECTOR3* pos, int* r, int* c )
{
	if( pos->x >= 0.0f && pos->x <= 1.0f && pos->z >= 0.0f && pos->z <= 1.0f )
	{
		*r = (int)(pos->z * m_width);
		*c = (int)(pos->x * m_width);
		return true;
	}
	else
	{
		return false;
	}
}

void Terrain::UpdatePlayerPos( D3DXVECTOR3& pos, D3DXVECTOR3& dir )
{
	int r, c;
	GetRowColumn(&pos, &r, &c);

	g_blackboard.UpdatePlayerPos(r, c);

	if (m_rPlayer < 0 || m_cPlayer < 0)
	{
		m_rPlayer = r;
		m_cPlayer = c;
		m_dirPlayer = dir;
	}
	else if ((m_rPlayer != r || m_cPlayer != c) && (g_blackboard.GetTerrainAnalysisType() == TerrainAnalysis_VisibleToPlayer || g_blackboard.GetTerrainAnalysisType() == TerrainAnalysis_Search))
	{
		m_rPlayer = r;
		m_cPlayer = c;
		m_dirPlayer = dir;
		m_reevaluateAnalysis = true;
	}
}

void Terrain::SetColor( int r, int c, DebugDrawingColor color )
{
	if( m_terrain[r][c] >= 0 )
	{
		m_terrainColor[r][c] = color;
	}
	else
	{
		ASSERTMSG( 0, "Can't set color to a grid square that is a wall" );
	}
}

void Terrain::ResetColors( void )
{
	for( int r=0; r<m_width; r++ )
	{
		for( int c=0; c<m_width; c++ )
		{
			if( m_terrain[r][c] >= 0 )
			{
				m_terrainColor[r][c] = DEBUG_COLOR_WHITE;
			}
			else
			{
				m_terrainColor[r][c] = DEBUG_COLOR_BLACK;
			}
		}
	}
}

void Terrain::ResetInfluenceMap( void )
{
	m_reevaluateAnalysis = true;

	for( int r=0; r<m_width; r++ )
	{
		for( int c=0; c<m_width; c++ )
		{
			m_terrainInfluenceMap[r][c] = 0.0f;
		}
	}
}

void Terrain::DrawDebugVisualization( IDirect3DDevice9* pd3dDevice )
{
	D3DXVECTOR3 pos( 0.0f, 0.001f, 0.0f );

	for( int r=0; r<m_width; r++ )
	{
		for( int c=0; c<m_width; c++ )
		{
			pos.x = (float)c / m_width;
			pos.z = (float)r / m_width;

			switch (m_terrain[r][c])
			{
			case TILE_WALL:
				g_debugdrawing.DrawCube( pd3dDevice, &pos, 1.0f/m_width, DEBUG_COLOR_BLACK );
				break;
			case TILE_EMPTY:
				g_debugdrawing.DrawQuad( pd3dDevice, &pos, 1.0f/m_width, m_terrainColor[r][c] );
				break;
			case TILE_WALL_INVISIBLE:
				g_debugdrawing.DrawCube( pd3dDevice, &pos, 1.f/m_width, DEBUG_COLOR_GRAY );
				break;
			default:
				assert("Invalid tile type");
			}
		}
	}

	if (!g_blackboard.GetTerrainAnalysisFlag())
		return;

	if (g_blackboard.GetTerrainAnalysisType() == TerrainAnalysis_Search)
	{	//Discount search tiles
		m_reevaluateAnalysis = true;
		for( int r=0; r<m_width; r++ )
		{
			for( int c=0; c<m_width; c++ )
			{
				m_terrainInfluenceMap[r][c] *= 0.999f;
			}
		}
	}

	UpdateOccupancyMap();

	pos.y = 0.002f;
	Analyze();

	TerrainAnalysisType type = static_cast<TerrainAnalysisType>(g_blackboard.GetTerrainAnalysisType());

	for( int r=0; r<m_width; r++ )
	{
		for( int c=0; c<m_width; c++ )
		{
			pos.x = (float)c / m_width;
			pos.z = (float)r / m_width;

			if (type == TerrainAnalysis_HideAndSeek || 
				TerrainAnalysis_Propagation || 
				TerrainAnalysis_PropagationWithNormalizingInfluence)
			{
				if (Near(m_terrainInfluenceMap[r][c], 0.0f))
					continue;

				if (m_terrainInfluenceMap[r][c] < 0.0f)
				{
					D3DXCOLOR color = D3DXCOLOR(0.0f, 0.0f, 1.0f, -m_terrainInfluenceMap[r][c]);
					g_debugdrawing.DrawQuad(pd3dDevice, &pos, 1.0f / m_width, color);
				}
				else if (m_terrainInfluenceMap[r][c] > 0.0f)
				{
					D3DXCOLOR color = D3DXCOLOR(1.0f, 0.0f, 0.0f, m_terrainInfluenceMap[r][c]);
					g_debugdrawing.DrawQuad(pd3dDevice, &pos, 1.0f / m_width, color);
				}
			}
			else
			{
				D3DXCOLOR color = D3DXCOLOR(1.0f, 0.0f, 0.0f, m_terrainInfluenceMap[r][c]);
				g_debugdrawing.DrawQuad(pd3dDevice, &pos, 1.0f / m_width, color);
			}

		}
	}
}

void Terrain::Analyze( void )
{
	if(m_reevaluateAnalysis)
	{
		m_reevaluateAnalysis = false;
		g_clock.ClearStopwatchAnalysis();

		if (!g_blackboard.GetTerrainAnalysisFlag())
		{
			ResetInfluenceMap();

			return;
		}

		g_clock.StartStopwatchAnalysis();
		switch (g_blackboard.GetTerrainAnalysisType())
		{
			case TerrainAnalysis_OpennessClosestWall:
				AnalyzeOpennessClosestWall();
				break;

			case TerrainAnalysis_Visibility:
				AnalyzeVisibility();
				break;

			case TerrainAnalysis_VisibleToPlayer:
				AnalyzeVisibleToPlayer();
				break;

			case TerrainAnalysis_Search:
				AnalyzeSearch();
				break;

			default:
				ResetInfluenceMap();
				break;
		}

		g_clock.StopStopwatchAnalysis();
	}
}

void Terrain::SetTerrainAnalysis(void)
{
	m_reevaluateAnalysis = true;
	ClearTerrainAnalysis();
	Analyze();
}

void Terrain::ClearTerrainAnalysis(void)
{
	for (int r = 0; r<m_width; r++)
	{
		for (int c = 0; c<m_width; c++)
		{
			m_terrainInfluenceMap[r][c] = 0.0f;
		}
	}
}

// Determine the intersection point of two line segments
// Return FALSE if the lines don't intersect
#define ABS(a) (((a) < 0) ? -(a) : (a))
bool Terrain::LineIntersect( float x1, float y1, float x2, float y2,
							 float x3, float y3, float x4, float y4 )
{
	float y4y3 = y4 - y3;
	float y1y3 = y1 - y3;
	float y2y1 = y2 - y1;
	float x4x3 = x4 - x3;
	float x2x1 = x2 - x1;
	float x1x3 = x1 - x3;
	float denom  = y4y3 * x2x1 - x4x3 * y2y1;
	float numera = x4x3 * y1y3 - y4y3 * x1x3;
	float numerb = x2x1 * y1y3 - y2y1 * x1x3;

	const float eps = 0.0001f;
	if (ABS(numera) < eps && ABS(numerb) < eps && ABS(denom) < eps)
	{	//Lines coincident (on top of each other)
		return true;
	}

	if (ABS(denom) < eps)
	{	//Lines parallel
		return false;
	}

	float mua = numera / denom;
	float mub = numerb / denom;
	if (mua < 0 || mua > 1 || mub < 0 || mub > 1)
	{	//No intersection
		return false;
	}
	else
	{	//Intersection
		return true;
	}
}

float Terrain::Lerp(float num1, float num2, float t)
{
	return ((1.0f - t) * num1 + t * num2);
}

int Terrain::GetWidth() const
{
	return m_width;
}

void Terrain::UpdateMap(int mapindex)
{
	while (GetMapIndex() != mapindex)
	{
		NextMap();
	}
}

void Terrain::InitFogOfWar()
{
	for (int i = 0; i < m_width; ++i)
		for (int j = 0; j < m_width; ++j)
			if (m_terrain[i][j] == TILE_WALL)
				m_terrain[i][j] = TILE_WALL_INVISIBLE;
}

void Terrain::ClearFogOfWar()
{
	for (int i = 0; i < m_width; ++i)
		for (int j = 0; j < m_width; ++j)
			if (m_terrain[i][j] == TILE_WALL_INVISIBLE)
				m_terrain[i][j] = TILE_WALL;
}

void Terrain::InitialOccupancyMap(int row, int col, float value)
{
	m_terrainInfluenceMap[row][col] = value;
	m_reevaluateAnalysis = false;
}

void Terrain::UpdateOccupancyMap(void)
{
	if (g_blackboard.GetTerrainAnalysisFlag())
	{
		TerrainAnalysisType type = static_cast<TerrainAnalysisType>(g_blackboard.GetTerrainAnalysisType());
		if (type == TerrainAnalysis_Propagation ||
			type == TerrainAnalysis_PropagationWithNormalizingInfluence ||
			(type == TerrainAnalysis_HideAndSeek && g_blackboard.GetSeekPlayerFlag()))
		{
			if (m_timerUpdatePropagation < 0.0f)
			{
				if (type == TerrainAnalysis_Propagation ||
					type == TerrainAnalysis_PropagationWithNormalizingInfluence)
					Propagation(g_blackboard.GetDecayFactor(), g_blackboard.GetGrowFactor(), true);
				else
					Propagation(g_blackboard.GetDecayFactor(), g_blackboard.GetGrowFactor(), false);

				if (type == TerrainAnalysis_HideAndSeek && g_blackboard.GetSeekPlayerFlag())
					NormalizeOccupancyMap(false);
				else if (type == TerrainAnalysis_PropagationWithNormalizingInfluence)
					NormalizeOccupancyMap(true);

				m_timerUpdatePropagation = g_blackboard.GetUpdateFrequency();
			}
			else
			{
				m_timerUpdatePropagation -= g_clock.GetElapsedTime();
			}
		}

	}
}