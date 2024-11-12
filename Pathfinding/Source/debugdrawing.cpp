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

DebugDrawing::DebugDrawing( void )
: m_pDebugLine(NULL),
  m_pVertexBuffer(NULL),
  m_pVertexBufferRight(NULL),
  m_pVertexBufferLeft(NULL),
  m_pVertexBufferFront(NULL),
  m_pVertexBufferTop(NULL),
  m_quad(false)
{

}

DebugDrawing::~DebugDrawing( void )
{
	SAFE_RELEASE( m_pDebugLine );
	SAFE_RELEASE( m_pVertexBuffer );
	SAFE_RELEASE( m_pVertexBufferRight );
	SAFE_RELEASE( m_pVertexBufferLeft );
	SAFE_RELEASE( m_pVertexBufferFront );
	SAFE_RELEASE( m_pVertexBufferTop );
}

void DebugDrawing::OnLostDevice( void )
{
	SAFE_RELEASE( m_pDebugLine );
	SAFE_RELEASE( m_pVertexBuffer );
	SAFE_RELEASE( m_pVertexBufferRight );
	SAFE_RELEASE( m_pVertexBufferLeft );
	SAFE_RELEASE( m_pVertexBufferFront );
	SAFE_RELEASE( m_pVertexBufferTop );
	m_quad = false;
}

D3DXCOLOR DebugDrawing::GetColor( DebugDrawingColor color )
{
	D3DXCOLOR xColor( 0.0f, 0.0f, 0.0f, 1.0f );

	switch( color )
	{
		case DEBUG_COLOR_RED:		xColor.r = 1.0f; break;
		case DEBUG_COLOR_GREEN:		xColor.g = 1.0f; break;
		case DEBUG_COLOR_BLUE:		xColor.b = 1.0f; break;
		case DEBUG_COLOR_YELLOW:	xColor.r = 1.0f; xColor.g = 1.0f; break;
		case DEBUG_COLOR_PURPLE:	xColor.r = 1.0f; xColor.b = 1.0f; break;
		case DEBUG_COLOR_CYAN:		xColor.g = 1.0f; xColor.b = 1.0f; break;
		case DEBUG_COLOR_WHITE:		xColor.r = 1.0f; xColor.g = 1.0f; xColor.b = 1.0f; break;
		case DEBUG_COLOR_BLACK:		break;
        case DEBUG_COLOR_GRAY:      xColor.r = .75f; xColor.g = .75f; xColor.b = .75f; break;
		case DEBUG_COLOR_NULL:		ASSERTMSG( 0, "Color can't be DEBUG_COLOR_NULL" ); break;
	}

	return( xColor );
}

void DebugDrawing::DrawLine( IDirect3DDevice9* pd3dDevice, D3DXMATRIX* pViewProj, D3DXVECTOR3 p0, D3DXVECTOR3 p1, DebugDrawingColor color, bool bArrowHead)
{
	D3DXCOLOR xColor = GetColor( color );
	D3DXVECTOR3 vLine[2];
	vLine[0] = p0;
	vLine[1] = p1;

	DrawLine( pd3dDevice, (D3DXVECTOR3*) &vLine, 2, pViewProj, &xColor, bArrowHead );
}

void DebugDrawing::DrawLine( IDirect3DDevice9* pd3dDevice, D3DXVECTOR3* vecPts, int nPtsCount, D3DXMATRIX* pViewProj, D3DXCOLOR* pColor, bool bArrowHead )
{
	if( nPtsCount < 2 )
	{
		ASSERTMSG(0, "DebugDrawing::DrawLine - Need at least 2 points");
		return;
	}

	if( m_pDebugLine == NULL )
	{
		D3DXCreateLine( pd3dDevice, &m_pDebugLine );
	}

	if( m_pDebugLine != NULL )
	{
		m_pDebugLine->DrawTransform( vecPts, nPtsCount, pViewProj, *pColor );
	}

	if (bArrowHead)
	{
		D3DXVECTOR3 vArrowHead[3];

		D3DXVECTOR3 vBackDir = vecPts[nPtsCount-2] - vecPts[nPtsCount-1];
		D3DXVec3Normalize(&vBackDir, &vBackDir);

		D3DXVECTOR3 vUpDir = D3DXVECTOR3(0.0f, 1.0f, 0.0);

		D3DXVECTOR3 vCrossDir;
		D3DXVec3Cross(&vCrossDir, &vBackDir, &vUpDir);

		vArrowHead[1] = vecPts[nPtsCount-1];
		vArrowHead[0] = vecPts[nPtsCount-1] + (vCrossDir + (2.0f*vBackDir)) * 0.01f;  
		vArrowHead[2] = vecPts[nPtsCount-1] + (-vCrossDir + (2.0f*vBackDir)) * 0.01f;  

		m_pDebugLine->DrawTransform((D3DXVECTOR3*) &vArrowHead, 3, pViewProj, *pColor);
	}
}

void DebugDrawing::DrawCircle(IDirect3DDevice9* pd3dDevice, D3DXMATRIX* pViewProj, D3DXVECTOR3* pos, float fRadius, DebugDrawingColor color)
{
	D3DXCOLOR xColor = GetColor( color );
	const int nPoints = 24;
	const float pi = 3.14159f;
	D3DXVECTOR3 vLine[nPoints];

	for (int i = 0; i < nPoints; i++)
	{
		vLine[i].x = pos->x + cos( (pi / 11.0f) * i) * fRadius;
		vLine[i].y = pos->y;
		vLine[i].z = pos->z + sin( (pi / 11.0f) * i) * fRadius;
	}

	DrawLine(pd3dDevice, (D3DXVECTOR3*) &vLine, nPoints, pViewProj, &xColor, false);
}

struct FLOOR_TILE_VERTEX
{
	float x, y, z;
	float nx,ny, nz;
};

#define FLOOR_TILE_VERTEX_FORMAT ( D3DFVF_XYZ | D3DFVF_NORMAL)

void DebugDrawing::OnResetDeviceQuad(IDirect3DDevice9* pd3dDevice, const D3DSURFACE_DESC *pBackBuffer)
{
	UINT length = 4 * sizeof(FLOOR_TILE_VERTEX); // 4 vertices
	DWORD usage = 0;
	D3DPOOL pool = D3DPOOL_MANAGED;

	{	//Quad
		pd3dDevice->CreateVertexBuffer( length, usage, FLOOR_TILE_VERTEX_FORMAT, pool, &m_pVertexBuffer, NULL );

		// created our buffer, copy our stuff into it
		float vertexData[] = {
				0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 
				0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 
				1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 
				1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 
		};
		FLOOR_TILE_VERTEX *data;
		m_pVertexBuffer->Lock(0,0, (VOID**)&data, 0);
		memcpy(data, vertexData, sizeof(FLOOR_TILE_VERTEX) * 4); 
		m_pVertexBuffer->Unlock();
	}
	{	//Right
		pd3dDevice->CreateVertexBuffer( length, usage, FLOOR_TILE_VERTEX_FORMAT, pool, &m_pVertexBufferRight, NULL );

		// created our buffer, copy our stuff into it
		float vertexData[] = {
				1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 
				1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f,
				1.0f, 0.2f, 0.0f, 0.0f, 0.0f, 1.0f, 
				1.0f, 0.2f, 1.0f, 0.0f, 0.0f, 1.0f, 
		};
		FLOOR_TILE_VERTEX *data;
		m_pVertexBufferRight->Lock(0,0, (VOID**)&data, 0);
		memcpy(data, vertexData, sizeof(FLOOR_TILE_VERTEX) * 4); 
		m_pVertexBufferRight->Unlock();
	}
	{	//Left
		pd3dDevice->CreateVertexBuffer( length, usage, FLOOR_TILE_VERTEX_FORMAT, pool, &m_pVertexBufferLeft, NULL );

		// created our buffer, copy our stuff into it
		float vertexData[] = {
				0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 
				0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f,
				0.0f, 0.2f, 0.0f, 0.0f, 0.0f, 1.0f, 
				0.0f, 0.2f, 1.0f, 0.0f, 0.0f, 1.0f, 
		};
		FLOOR_TILE_VERTEX *data;
		m_pVertexBufferLeft->Lock(0,0, (VOID**)&data, 0);
		memcpy(data, vertexData, sizeof(FLOOR_TILE_VERTEX) * 4); 
		m_pVertexBufferLeft->Unlock();
	}
	{	//Front
		pd3dDevice->CreateVertexBuffer( length, usage, FLOOR_TILE_VERTEX_FORMAT, pool, &m_pVertexBufferFront, NULL );

		// created our buffer, copy our stuff into it
		float vertexData[] = {
				0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 
				1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
				0.0f, 0.2f, 0.0f, 0.0f, 0.0f, 1.0f, 
				1.0f, 0.2f, 0.0f, 0.0f, 0.0f, 1.0f, 
		};
		FLOOR_TILE_VERTEX *data;
		m_pVertexBufferFront->Lock(0,0, (VOID**)&data, 0);
		memcpy(data, vertexData, sizeof(FLOOR_TILE_VERTEX) * 4); 
		m_pVertexBufferFront->Unlock();
	}
	{	//Top
		pd3dDevice->CreateVertexBuffer( length, usage, FLOOR_TILE_VERTEX_FORMAT, pool, &m_pVertexBufferTop, NULL );

		// created our buffer, copy our stuff into it
		float vertexData[] = {
				0.0f, 0.2f, 0.0f, 0.0f, 1.0f, 0.0f, 
				0.0f, 0.2f, 1.0f, 0.0f, 1.0f, 0.0f, 
				1.0f, 0.2f, 0.0f, 0.0f, 1.0f, 0.0f, 
				1.0f, 0.2f, 1.0f, 0.0f, 1.0f, 0.0f, 
		};
		FLOOR_TILE_VERTEX *data;
		m_pVertexBufferTop->Lock(0,0, (VOID**)&data, 0);
		memcpy(data, vertexData, sizeof(FLOOR_TILE_VERTEX) * 4); 
		m_pVertexBufferTop->Unlock();
	}
	ZeroMemory( &m_material, sizeof(D3DMATERIAL9) );
}

void DebugDrawing::DrawQuad(IDirect3DDevice9* pd3dDevice, D3DXVECTOR3* pos, float width, DebugDrawingColor color)
{
	D3DXCOLOR xColor = GetColor( color );
	DrawQuad(pd3dDevice, pos, width, xColor);
}

void DebugDrawing::DrawQuad(IDirect3DDevice9* pd3dDevice, D3DXVECTOR3* pos, float width, D3DXCOLOR color)
{
	if( color.a == 0.0 ) { return; }

	if( !m_quad )
	{
		m_quad = true;
		OnResetDeviceQuad( pd3dDevice, NULL );
	}

	D3DXMATRIX translation;
	D3DXMatrixTranslation( &translation, pos->x, pos->y, pos->z );

	D3DXMATRIX scale;
	D3DXMatrixScaling( &scale, width, width, width );	

	D3DXMATRIX srt;
	srt = scale * translation;

	// translate to stand at location, draw our mesh
	pd3dDevice->SetTransform( D3DTS_WORLD, &srt );

	pd3dDevice->SetStreamSource(0, m_pVertexBuffer, 0, sizeof(FLOOR_TILE_VERTEX));
	pd3dDevice->SetFVF(FLOOR_TILE_VERTEX_FORMAT);
	m_material.Diffuse = color;
	m_material.Ambient = color;
	m_material.Specular = D3DXCOLOR( 0.f, 0.f, 0.f, 0.f );
    m_material.Emissive = D3DXCOLOR( .0f, 0.f, 0.f, 0.f );
    m_material.Power = 0.f;

	pd3dDevice->SetMaterial(&m_material);
	pd3dDevice->SetTexture(0, NULL);

	pd3dDevice->SetRenderState(D3DRS_ALPHABLENDENABLE, TRUE);
	pd3dDevice->SetRenderState(D3DRS_SRCBLEND, D3DBLEND_SRCALPHA);
	pd3dDevice->SetRenderState(D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA);

	pd3dDevice->DrawPrimitive( D3DPT_TRIANGLESTRIP, 0, 2); // draw our 2 triangles.
}

void DebugDrawing::DrawCube(IDirect3DDevice9* pd3dDevice, D3DXVECTOR3* pos, float width, DebugDrawingColor colorEnum)
{
    D3DXCOLOR color = GetColor(colorEnum);

	if( !m_quad )
	{
		m_quad = true;
		OnResetDeviceQuad( pd3dDevice, NULL );
	}

	D3DXMATRIX translation;
	D3DXMatrixTranslation( &translation, pos->x, pos->y, pos->z );

	D3DXMATRIX scale;
	D3DXMatrixScaling( &scale, width, width, width );	

	D3DXMATRIX srt;
	srt = scale * translation;

	// translate to stand at location, draw our mesh
	pd3dDevice->SetTransform( D3DTS_WORLD, &srt );

	pd3dDevice->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);
	pd3dDevice->SetFVF(FLOOR_TILE_VERTEX_FORMAT);

	//Draw Right
	m_material.Diffuse = color;
	m_material.Ambient = color;
	pd3dDevice->SetMaterial(&m_material);
	pd3dDevice->SetTexture(0, NULL);
	pd3dDevice->SetStreamSource(0, m_pVertexBufferRight, 0, sizeof(FLOOR_TILE_VERTEX));
	pd3dDevice->DrawPrimitive( D3DPT_TRIANGLESTRIP, 0, 2); // draw our 2 triangles.
	
	//Draw Left
	m_material.Diffuse = color;
	m_material.Ambient = color;
	pd3dDevice->SetMaterial(&m_material);
	pd3dDevice->SetTexture(0, NULL);
	pd3dDevice->SetStreamSource(0, m_pVertexBufferLeft, 0, sizeof(FLOOR_TILE_VERTEX));
	pd3dDevice->DrawPrimitive( D3DPT_TRIANGLESTRIP, 0, 2); // draw our 2 triangles.
	
	//Draw Front
	m_material.Diffuse = color;
	m_material.Ambient = color;
	pd3dDevice->SetMaterial(&m_material);
	pd3dDevice->SetTexture(0, NULL);
	pd3dDevice->SetStreamSource(0, m_pVertexBufferFront, 0, sizeof(FLOOR_TILE_VERTEX));
	pd3dDevice->DrawPrimitive( D3DPT_TRIANGLESTRIP, 0, 2); // draw our 2 triangles.
	
	//Draw Top
	m_material.Diffuse = color;
	m_material.Ambient = color;
	pd3dDevice->SetMaterial(&m_material);
	pd3dDevice->SetTexture(0, NULL);
	pd3dDevice->SetStreamSource(0, m_pVertexBufferTop, 0, sizeof(FLOOR_TILE_VERTEX));
	pd3dDevice->DrawPrimitive( D3DPT_TRIANGLESTRIP, 0, 2); // draw our 2 triangles.
}


