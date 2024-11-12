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

enum TerrainAnalysisType
{
	//TerrainAnalysis_None = 0,
	TerrainAnalysis_OpennessClosestWall,
	TerrainAnalysis_Visibility,
	TerrainAnalysis_VisibleToPlayer,
	TerrainAnalysis_Search,
	TerrainAnalysis_Propagation,
	TerrainAnalysis_PropagationWithNormalizingInfluence,
	TerrainAnalysis_HideAndSeek,
	TerrainAnalysis_Count
};

class Terrain
{
public:

	Terrain(void);
	~Terrain(void);

	void Create(void);
	void NextMap(void);
	int GetMapIndex(void)					{ return(m_nextMap); }
	Map *GetCurrentMap(void);
	inline size_t NumberOfMaps(void)		{ return m_maps.size(); }

	D3DXVECTOR3 GetCoordinates(int r, int c);
	bool GetRowColumn(D3DXVECTOR3* pos, int* r, int* c);
	inline bool IsWall(int r, int c)		{ return(m_terrain[r][c] == TILE_WALL); }

	bool IsClearPath(int r0, int c0, int r1, int c1);

	void SetColor(int r, int c, DebugDrawingColor color);
	void ResetColors(void);

	void UpdatePlayerPos(D3DXVECTOR3& pos, D3DXVECTOR3& dir);

	void Analyze(void);
	void SetTerrainAnalysis(void);
	inline void SetInfluenceMapValue(int r, int c, float value) { m_terrainInfluenceMap[r][c] = value; }
	inline float GetInfluenceMapValue(int r, int c) { return m_terrainInfluenceMap[r][c]; }
	void ResetInfluenceMap(void);

	void DrawDebugVisualization(IDirect3DDevice9* pd3dDevice);
	int GetWidth() const;

	void UpdateMap(int mapindex);

	void InitFogOfWar();
	void ClearFogOfWar();

	void InitialOccupancyMap(int row, int col, float value);
	void UpdateOccupancyMap(void);

	void Propagation(float decay, float growing, bool computeNegativeInfluence);
	void NormalizeOccupancyMap(bool computeNegativeInfluence);

protected:

	// Map List parameters
	int m_nextMap;
	typedef std::vector<Map> MapList;
	MapList m_maps;

	// Parameters for current map
	int m_width;
	Tile** m_terrain;
	DebugDrawingColor** m_terrainColor;
	float** m_terrainInfluenceMap;

	bool m_reevaluateAnalysis;
	int m_rPlayer, m_cPlayer;
	D3DXVECTOR3 m_dirPlayer;

	float m_timerUpdatePropagation;

	void AnalyzeOpennessClosestWall(void);
	void AnalyzeVisibility(void);
	void AnalyzeVisibleToPlayer(void);
	void AnalyzeSearch(void);
	void ClearTerrainAnalysis(void);

	float ClosestWall(int row, int col);
	bool LineIntersect(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4);
	float Lerp(float num1, float num2, float t);
};
