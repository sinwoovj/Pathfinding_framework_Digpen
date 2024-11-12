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

enum FileLoadSample
{
	FLS_NONE,
	FLS_OUTCOME,
	FLS_DIAGONAL,
	FLS_STRAIGHTLINE,
	FLS_RUBBERSMOOTH,
};

enum SampleTest
{
	Test_DijkstraSmall = 0,
	Test_AStarEuclideanSmall,
	Test_AStarOctileSmall,
	Test_DijkstraBig,
	Test_AStarEuclideanBig,
	Test_AStarOctileBig,
	Test_Diagonal,
	Test_Straightline,
	Test_Rubberbanding,
	Test_Smoothing,
	Test_RubberbandingSmoothing,
	Test_FullTest,
	
	Test_Count,
};

struct PathfindingOutcome
{
	int m_rStart;
	int m_cStart;
	int m_rGoal;
	int m_cGoal;
	int m_size;
	int m_dist;
	int m_success;
	int m_mapindex;

	PathfindingOutcome() {}

	PathfindingOutcome(int rstart, int cstart, int rgoal, int cgoal, 
		int size, int dist, int success, int mapindex)
		: m_mapindex(mapindex), m_rStart(rstart), m_cStart(cstart), m_rGoal(rgoal), m_cGoal(cgoal),
		m_size(size), m_dist(dist), m_success(success)
	{
	}

	bool operator==(const PathfindingOutcome &rhs) const
	{
		if (m_rStart == rhs.m_rStart && m_cStart == rhs.m_cStart &&
			m_rGoal == rhs.m_rGoal && m_cGoal == rhs.m_cGoal &&
			m_dist == rhs.m_dist && m_success == rhs.m_success &&
			m_mapindex == rhs.m_mapindex)
			return true;

		return false;
	}

	bool operator!=(const PathfindingOutcome &rhs) const
	{
		if (*this == rhs)
			return false;

		return true;
	}
};

typedef std::vector<PathfindingOutcome> PathFindingOutcomeArray;

class PathfindingTests
{
	friend class MovementSetting;

public: 

	PathfindingTests()
		: m_outcome_index(-1), m_sample(FLS_NONE)
		{}

	// default timing test
	void RunTest(int mapIndex, int xStart, int yStart, int xGoal, int yGoal,
		std::string filename, Agent &agent, int type);

	// run pathfinding for all possible pairs for all maps
	void RunAllTests(const char *filename_noext, Agent &agent);

	// create sample test files by picking random data
	void CreateSampleTest(const char *loadfile_noext, const char *savefile_noext,
		int mapindex, int num_totaltests, int num_bigtests = 0, int num_failedtests = 0);

	// pathfinding test against sample test data
	bool RunSampleTest(const char *filename, const char *out_filename,
		Agent &agent, unsigned num_faileddata, int heuristic, float weight);

	// get simple test result
	void GetSampleTestResult(const char *filename, Agent &agent, bool is_increasing,
		bool is_straightline, bool is_rubberband, bool is_smooth, FileLoadSample sample);

private:
	int m_outcome_index;
	PathFindingOutcomeArray m_outcomes;
	FileLoadSample m_sample;

	static void PrepareTest(Agent &agent, MovementSetting &movement_data,
		int heuristic, float weight, bool setpos);

	static void FinishTest(Agent &agent, MovementSetting &movement_data, bool applypos);

	static void RunPathfindingTest(int rstart, int cstart, int rgoal, int cgoal,
		Agent &agent, PathfindingOutcome &outcome);
};
