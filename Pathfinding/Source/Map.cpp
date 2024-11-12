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

Map::Map()
	: m_width(0),
	m_terrain(0),
	m_terrainColor(0),
	m_terrainInfluenceMap(0)
{
}

Map::Map(int width)
	: m_width(width),
	m_terrain(0),
	m_terrainColor(0),
	m_terrainInfluenceMap(0)
{
	InitArray(m_terrain);
	InitArray(m_terrainColor);
	InitArray(m_terrainInfluenceMap);
}

void Map::Serialize(const std::wstring& filename)
{
	m_filename = filename;
	std::ifstream in(filename.c_str());

	assert(in.good() && "File not found");
	assert(!m_width && !m_terrain && !m_terrainColor &&
		!m_terrainInfluenceMap && "Map may already have data");

	// width is stored first
	in >> m_width;

	// initialize storage
	InitArray(m_terrain);
	InitArray(m_terrainColor);
	InitArray(m_terrainInfluenceMap);

	// read in map data
	for (int r = 0; r < m_width; ++r)
		for (int c = 0; c < m_width; ++c)
		{
			int val;
			in >> val;

			switch (val)
			{
			case TILE_EMPTY:
			case TILE_WALL:
				m_terrain[r][c] = (Tile)val;
				break;
			default:
				assert("Invalid tile type");
			}
		}
}

void Map::Destroy()
{
	DestroyArray(m_terrain);
	DestroyArray(m_terrainColor);
	DestroyArray(m_terrainInfluenceMap);
}

int Map::GetWidth() const
{
	return m_width;
}

Tile** Map::GetTerrain() const
{
	return m_terrain;
}

DebugDrawingColor** Map::GetTerrainColor() const
{
	return m_terrainColor;
}

float** Map::GetInfluenceMap() const
{
	return m_terrainInfluenceMap;
}

void Map::PlaceWall(int &row, int &col)
{
	if (row < 0 || row >= m_width)
		return;
	if (col < 0 || col >= m_width)
		return;

	Tile &curTile = m_terrain[row][col];
	if (curTile != Tile::TILE_EMPTY)
		return;

	curTile = Tile::TILE_WALL;
}

void Map::RemoveWall(int &row, int &col)
{
	if (row < 0 || row >= m_width)
		return;
	if (col < 0 || col >= m_width)
		return;

	Tile &curTile = m_terrain[row][col];
	m_terrainColor[row][col] = DEBUG_COLOR_WHITE;

	curTile = Tile::TILE_EMPTY;
}

void Map::SaveMap(void)
{
	std::ofstream out(m_filename.c_str());

	if (out.is_open() == false)
	{
		wchar_t buffer[50];
		swprintf(buffer, L"Map_%d.txt", g_terrain.NumberOfMaps());
		m_filename = buffer;
		out.open(m_filename.c_str());
	}

	// width is stored first
	out << m_width << std::endl << std::endl;

	// output the map data
	for (int r = 0; r < m_width; ++r)
	{
		for (int c = 0; c < m_width; ++c)
		{
			Tile &val = m_terrain[r][c];

			out << (int)val;
			out << ' ';
		}
		out << std::endl;
	}

	out.close();
}
