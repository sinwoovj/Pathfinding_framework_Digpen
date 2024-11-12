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


float Terrain::ClosestWall(int row, int col)
{
	// TODO: Helper function for the Terrain Analysis project.

	// For each tile, check the distance with every wall of the map.
	// Return the distance to the closest wall or side.


	// WRITE YOUR CODE HERE


	return 0.0f;	//REPLACE THIS
}

void Terrain::AnalyzeOpennessClosestWall(void)
{
	// TODO: Implement this for the Terrain Analysis project.

	// Mark every square on the terrain (m_terrainInfluenceMap[r][c]) with
	// the value 1/(d*d), where d is the distance to the closest wall in 
	// row/column grid space.
	// Edges of the map count as walls!

	// WRITE YOUR CODE HERE


}

void Terrain::AnalyzeVisibility(void)
{
	// TODO: Implement this for the Terrain Analysis project.

	// Mark every square on the terrain (m_terrainInfluenceMap[r][c]) with
	// the number of grid squares (that are visible to the square in question)
	// divided by 160 (a magic number that looks good). Cap the value at 1.0.

	// Two grid squares are visible to each other if a line between 
	// their centerpoints doesn't intersect the four boundary lines
	// of every walled grid square. Put this code in IsClearPath().


	// WRITE YOUR CODE HERE


}

void Terrain::AnalyzeVisibleToPlayer(void)
{
	// TODO: Implement this for the Terrain Analysis project.

	// For every square on the terrain (m_terrainInfluenceMap[r][c])
	// that is visible to the player square, mark it with 1.0.
	// For all non 1.0 squares that are visible to, and next to 1.0 squares,
	// mark them with 0.5. Otherwise, the square should be marked with 0.0.

	// Two grid squares are visible to each other if a line between 
	// their centerpoints doesn't intersect the four boundary lines
	// of every walled grid square. Put this code in IsClearPath().


	// WRITE YOUR CODE HERE


}

void Terrain::AnalyzeSearch(void)
{
	// TODO: Implement this for the Terrain Analysis project.

	// For every square on the terrain (m_terrainInfluenceMap[r][c])
	// that is visible by the player square, mark it with 1.0.
	// Otherwise, don't change the value (because it will get
	// decremented with time in the update call).
	// You must consider the direction the player is facing:
	// D3DXVECTOR2 playerDir = D3DXVECTOR2(m_dirPlayer.x, m_dirPlayer.z)
	// Give the player a field of view a tad greater than 180 degrees.

	// Two grid squares are visible to each other if a line between 
	// their centerpoints doesn't intersect the four boundary lines
	// of every walled grid square. Put this code in IsClearPath().


	// WRITE YOUR CODE HERE


}

bool Terrain::IsClearPath(int r0, int c0, int r1, int c1)
{
	// TODO: Implement this for the Terrain Analysis project.

	// Two grid squares (r0,c0) and (r1,c1) are visible to each other 
	// if a line between their centerpoints doesn't intersect the four 
	// boundary lines of every walled grid square. Make use of the
	// function LineIntersect(). You should also puff out the four
	// boundary lines by a very tiny bit so that a diagonal line passing
	// by the corner will intersect it.


	// WRITE YOUR CODE HERE


	return true;	//REPLACE THIS
}

void Terrain::Propagation(float decay, float growing, bool computeNegativeInfluence)
{
	// TODO: Implement this for the Occupancy Map project.

	// computeNegativeInfluence flag is true if we need to handle two agents
	// (have both positive and negative influence)
	// computeNegativeInfluence flag is false if we only deal with positive
	// influence (we should ignore all negative influence)

	// Pseudo code:
	//
	// For each tile on the map
	//
	//   Get the influence value of each neighbor after decay
	//   Then keep the decayed influence WITH THE HIGHEST ABSOLUTE.
	//
	//   Apply linear interpolation to the influence value of the tile, 
	//   and the maximum decayed influence value from all neighbors, with growing 
	//   factor as coefficient
	//
	//   Store the result to the temp layer
	//
	// Store influence value from temp layer

	// WRITE YOUR CODE HERE


}

void Terrain::NormalizeOccupancyMap(bool computeNegativeInfluence)
{
	// TODO: Implement this for the Occupancy Map project.

	// divide all tiles with maximum influence value, so the range of the
	// influence is kept in [0,1]
	// if we need to handle negative influence value, divide all positive
	// tiles with maximum influence value, and all negative tiles with
	// minimum influence value * -1, so the range of the influence is kept
	// at [-1,1]
	
	// computeNegativeInfluence flag is true if we need to handle two agents
	// (have both positive and negative influence)
	// computeNegativeInfluence flag is false if we only deal with positive
	// influence, ignore negative influence 

	// WRITE YOUR CODE HERE


}
