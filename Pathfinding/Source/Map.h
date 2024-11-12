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

#include "debugdrawing.h"

enum Tile
{
	TILE_WALL = -1,
	TILE_EMPTY,
	TILE_WALL_INVISIBLE,
};

class Map {
private:
	int m_width;
	std::wstring m_filename;

	Tile** m_terrain;
	DebugDrawingColor** m_terrainColor;
	float** m_terrainInfluenceMap;

	template <typename T>
	void InitArray(T**& t)
	{
		assert(m_width > 0 && "Invalid width for map");
		assert(!t && "Map data may already be initialized!");

		t = new T*[m_width];
		for (int i = 0; i < m_width; ++i)
		{
			t[i] = new T[m_width];
			memset(t[i], 0, sizeof(T)*m_width);
		}
	}

	template <typename T>
	void DestroyArray(T**& t)
	{
		for (int i = 0; i < m_width; ++i)
			delete[] t[i];
		delete[] t;

		t = 0;
	}
public:
	Map();
	Map(int);

	void Serialize(const std::wstring&);
	void Destroy();

	int GetWidth() const;
	Tile** GetTerrain() const;
	DebugDrawingColor** GetTerrainColor() const;
	float** GetInfluenceMap() const;

	void PlaceWall(int &row, int &col);
	void RemoveWall(int &row, int &col);
	void SaveMap(void);
	inline std::wstring GetFileName(void) { return m_filename; };
};