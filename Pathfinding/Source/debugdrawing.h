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

enum DebugDrawingColor
{
	DEBUG_COLOR_NULL,
	DEBUG_COLOR_WHITE,
	DEBUG_COLOR_BLACK,
	DEBUG_COLOR_GRAY,
	DEBUG_COLOR_RED,
	DEBUG_COLOR_GREEN,
	DEBUG_COLOR_BLUE,
	DEBUG_COLOR_YELLOW,
	DEBUG_COLOR_PURPLE,
	DEBUG_COLOR_CYAN,
	DEBUG_COLOR_NUM
};

class DebugDrawing
{
public:

	DebugDrawing( void );
	~DebugDrawing( void );

	void OnLostDevice( void );

	void DrawLine( IDirect3DDevice9* pd3dDevice, D3DXMATRIX* pViewProj, D3DXVECTOR3 p0, D3DXVECTOR3 p1, DebugDrawingColor color, bool bArrowHead );
	void DrawLine( IDirect3DDevice9* pd3dDevice, D3DXVECTOR3* vecPts, int nPtsCount, D3DXMATRIX* pViewProj, D3DXCOLOR* pColor, bool bArrowHead );
	void DrawCircle(IDirect3DDevice9* pd3dDevice, D3DXMATRIX* pViewProj, D3DXVECTOR3* pos, float fRadius, DebugDrawingColor color);

	void OnResetDeviceQuad(IDirect3DDevice9* pd3dDevice, const D3DSURFACE_DESC *pBackBuffer);
	void DrawQuad(IDirect3DDevice9* pd3dDevice, D3DXVECTOR3* pos, float width, D3DXCOLOR color);
	void DrawQuad(IDirect3DDevice9* pd3dDevice, D3DXVECTOR3* pos, float width, DebugDrawingColor color);
	void DrawCube(IDirect3DDevice9* pd3dDevice, D3DXVECTOR3* pos, float width, DebugDrawingColor color);

protected:

	//For line drawing
	ID3DXLine* m_pDebugLine;

	//For quad drawing
	bool m_quad;
	IDirect3DVertexBuffer9* m_pVertexBuffer;
	IDirect3DVertexBuffer9* m_pVertexBufferRight;
	IDirect3DVertexBuffer9* m_pVertexBufferLeft;
	IDirect3DVertexBuffer9* m_pVertexBufferFront;
	IDirect3DVertexBuffer9* m_pVertexBufferTop;
	D3DMATERIAL9 m_material;

	//Helper functions
	D3DXCOLOR GetColor( DebugDrawingColor color );

};
