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

#include <algorithm>

struct Point2D
{
	int m_row;
	int m_col;

	Point2D(int row, int col) : m_row(row), m_col(col) {}

	bool operator==(const Point2D &rhs) const
	{
		return (m_row == rhs.m_row && m_col == rhs.m_col);
	}

	bool operator!=(const Point2D &rhs) const
	{
		return (m_row != rhs.m_row || m_col != rhs.m_col);
	}
};


// load test files into PathfindingOutcome array
static void LoadTestData(const char *filename_noext, int mapindex,
	PathFindingOutcomeArray &outcomes);

// load test files into PathfindingOutcome array
static void LoadTestData(const char *filename,
	PathFindingOutcomeArray &outcomes);

// load test files into PathfindingOutcome array
static void LoadTestData(const char *filename_noext, int mapindex,
	PathFindingOutcomeArray &outcomes_success,
	PathFindingOutcomeArray &outcomes_failure);

static std::string AddMapIndexToFilename(const std::string &filename_noext, int mapindex);

static int CalculateSqDist(WaypointList &waypointlist,
	const Point2D &start_pt, const Point2D &goal_pt, bool &is_success);

static int GetSqDist(const Point2D &pt1, const Point2D &pt2, bool &is_success);

static void GetIntFromTestDataString(std::string &str, int &value);

static void AddOutcomeToFileStream(PathfindingOutcome &outcome, std::ofstream &out);

static void SavePathfindingOutcomes(const char *filename, PathFindingOutcomeArray &outcomes);

void PathfindingTests::PrepareTest(Agent &agent, MovementSetting &movement_data,
	int heuristic, float weight, bool setpos)
{
	Movement& movement = agent.m_owner->GetMovement();
	movement_data.Set(movement);

	if (setpos)
	{
		int row, col;
		D3DXVECTOR3 mypos = agent.m_owner->GetBody().GetPos();
		g_terrain.GetRowColumn(&mypos, &row, &col);

		movement_data.SetPos(row, col);
	}

	movement.SetHeuristicCalc(heuristic); // Should be Octile
	movement.SetHeuristicWeight(weight);
	movement.SetSingleStep(false);
	movement.SetSmoothPath(false);
	movement.SetRubberbandPath(false);
	movement.SetStraightlinePath(false);
	movement.AStarUsesAnalysis(false);
}

void PathfindingTests::FinishTest(Agent &agent, MovementSetting &movement_data, bool applypos)
{
	Movement& movement = agent.m_owner->GetMovement();

	movement_data.Reset(movement);

	if (applypos)
	{
		int row, col;
		movement_data.GetPos(row, col);

		agent.m_owner->GetBody().SetPos(g_terrain.GetCoordinates(row, col));
	}
}

std::string AddMapIndexToFilename(const std::string &filename_noext, int mapindex)
{
	std::string filename(filename_noext);
	filename += "_";
	filename += std::to_string(mapindex);
	filename += ".txt";

	return filename;
}

int CalculateSqDist(WaypointList &waypointlist,
	const Point2D &start_pt, const Point2D &goal_pt, bool &is_success)
{
	if (waypointlist.empty())
	{
		is_success = false;

		return 0;
	}

	// first get the pointList (in RowCol) from waypointlist 
	std::list<Point2D> pointList;
	int row, col;
	for (WaypointList::iterator it = waypointlist.begin(); it != waypointlist.end(); ++it)
	{
		g_terrain.GetRowColumn(&(*it), &row, &col);
		pointList.emplace_back(Point2D(row, col));
	}

	if (pointList.back() != goal_pt)
	{
		is_success = false;

		return 0;
	}

	if (pointList.front() != start_pt)
		pointList.push_front(start_pt);

	int sqdist = 0;

	std::list<Point2D>::iterator it = pointList.begin();
	while (*it != pointList.back())
	{
		sqdist += GetSqDist(*it, *std::next(it, 1), is_success);

		if (!is_success)
			return 0;

		++it;
	}

	return sqdist;
}

int GetSqDist(const Point2D &pt1, const Point2D &pt2, bool &is_success)
{
	int row_diff = std::abs(pt1.m_row - pt2.m_row);
	int col_diff = std::abs(pt1.m_col - pt2.m_col);

	if (row_diff > 1 || col_diff > 1 || pt1 == pt2)
	{
		is_success = false;
		return 0;
	}

	return (row_diff * row_diff + col_diff * col_diff);

}

void GetIntFromTestDataString(std::string &str, int &value)
{
	size_t index = str.find_first_of(',');
	if (index != std::string::npos)
	{
		std::string value_str = str.substr(0, index);
		str = str.substr(index + 1, str.size());

		value = std::atoi(value_str.c_str());
	}
	else
		value = std::atoi(str.c_str());
}

void AddOutcomeToFileStream(PathfindingOutcome &outcome, std::ofstream &out)
{
	char tmpstring[100];

	sprintf_s(tmpstring, "%d,%d,%d,%d,%d,%d,%d,%d\n",
		outcome.m_rStart, outcome.m_cStart, outcome.m_rGoal, outcome.m_cGoal,
		outcome.m_size, outcome.m_dist, outcome.m_success, outcome.m_mapindex);

	out << tmpstring;
}

void PathfindingTests::RunPathfindingTest(int rstart, int cstart, int rgoal, int cgoal,
	Agent &agent, PathfindingOutcome &outcome)
{
	Movement& movement = agent.m_owner->GetMovement();
	agent.m_owner->GetBody().SetPos(g_terrain.GetCoordinates(rstart, cstart));
	movement.ComputePathWithTiming(rgoal, cgoal, true);

	Point2D start_pt(rstart, cstart);
	Point2D goal_pt(rgoal, cgoal);
	bool is_success = true;

	outcome.m_rStart = rstart;
	outcome.m_cStart = cstart;
	outcome.m_rGoal = rgoal;
	outcome.m_cGoal = cgoal;
	outcome.m_size = movement.m_waypointList.size();
	outcome.m_dist = CalculateSqDist(movement.m_waypointList, start_pt, goal_pt, is_success);;
	outcome.m_success = is_success;
}

// load test files into PathfindingOutcome array
void LoadTestData(const char *filename_noext, int mapindex,
	PathFindingOutcomeArray &outcomes)
{
	std::string filename = AddMapIndexToFilename(filename_noext, mapindex);

	LoadTestData(filename.c_str(), outcomes);
}

// load test files into PathfindingOutcome array
void LoadTestData(const char *filename,
	PathFindingOutcomeArray &outcomes)
{
	std::ifstream in(filename);
	std::string instr;

	while (std::getline(in, instr))
	{
		PathfindingOutcome new_outcome;

		GetIntFromTestDataString(instr, new_outcome.m_rStart);
		GetIntFromTestDataString(instr, new_outcome.m_cStart);
		GetIntFromTestDataString(instr, new_outcome.m_rGoal);
		GetIntFromTestDataString(instr, new_outcome.m_cGoal);
		GetIntFromTestDataString(instr, new_outcome.m_size);
		GetIntFromTestDataString(instr, new_outcome.m_dist);
		GetIntFromTestDataString(instr, new_outcome.m_success);
		GetIntFromTestDataString(instr, new_outcome.m_mapindex);

		outcomes.emplace_back(new_outcome);
	}
}

// load test files into PathfindingOutcome array
void LoadTestData(const char *filename_noext, int mapindex,
	PathFindingOutcomeArray &outcomes_success,
	PathFindingOutcomeArray &outcomes_failure)
{
	std::string filename = AddMapIndexToFilename(filename_noext, mapindex);
	std::ifstream in(filename);
	std::string instr;

	while (std::getline(in, instr))
	{
		PathfindingOutcome new_outcome;

		GetIntFromTestDataString(instr, new_outcome.m_rStart);
		GetIntFromTestDataString(instr, new_outcome.m_cStart);
		GetIntFromTestDataString(instr, new_outcome.m_rGoal);
		GetIntFromTestDataString(instr, new_outcome.m_cGoal);
		GetIntFromTestDataString(instr, new_outcome.m_size);
		GetIntFromTestDataString(instr, new_outcome.m_dist);
		GetIntFromTestDataString(instr, new_outcome.m_success);
		GetIntFromTestDataString(instr, new_outcome.m_mapindex);

		if (new_outcome.m_success)
			outcomes_success.emplace_back(new_outcome);
		else
			outcomes_failure.emplace_back(new_outcome);
	}
}

void SavePathfindingOutcomes(const char *filename, PathFindingOutcomeArray &outcomes)
{
	// Output results
	std::ofstream out(filename);

	for (PathFindingOutcomeArray::iterator it = outcomes.begin();
		it != outcomes.end(); ++it)
	{
		AddOutcomeToFileStream(*it, out);
	}

	out.close();
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void PathfindingTests::RunTest(int mapIndex, int xStart, int yStart, int xGoal, int yGoal,
	std::string filename, Agent &agent, int type)
{
	MovementSetting movement_setting;
	Movement& movement = agent.m_owner->GetMovement();

	PrepareTest(agent, movement_setting, 1, 1.01f, false);
	g_terrain.UpdateMap(mapIndex);

	g_blackboard.UpdatePlayerPos(xStart, yStart);
	agent.m_owner->GetBody().SetPos(g_terrain.GetCoordinates(xStart, yStart));

	double results[NUMTESTS];
	double resultsTime[NUMTESTS];

	g_clock.UpdateQPCFrequency();

	// Run tests
	for (int i = 0; i < NUMTESTS; ++i)
	{

		switch (type)
		{
		case 0:
		{
			g_blackboard.UpdateGoalPos(xGoal, yGoal);
			g_clock.ClearStopwatchPathfinding();
			g_clock.StartStopwatchPathfinding();
			bool foundPath = movement.ComputePath(xGoal, yGoal, true);
			g_clock.StopStopwatchPathfinding();
		}
		break;

		case 1:
		{
			if (i % 2)
			{
				g_blackboard.UpdatePlayerPos(38, 16);
				agent.m_owner->GetBody().SetPos(g_terrain.GetCoordinates(38, 16));
				g_clock.ClearStopwatchPathfinding();
				g_clock.StartStopwatchPathfinding();
				bool foundPath = movement.ComputePath(0, 5, true);
				g_clock.StopStopwatchPathfinding();
			}
			else
			{
				g_blackboard.UpdatePlayerPos(17, 39);
				agent.m_owner->GetBody().SetPos(g_terrain.GetCoordinates(17, 39));
				g_clock.ClearStopwatchPathfinding();
				g_clock.StartStopwatchPathfinding();
				bool foundPath = movement.ComputePath(0, 1, true);
				g_clock.StopStopwatchPathfinding();
			}
		}
		break;
		}

		results[i] = g_clock.GetStopwatchPathfinding();
		resultsTime[i] = g_clock.GetStopwatchPathfindingTime();
	}

	// Find minimum
	double min = results[0];
	double minTime = resultsTime[0];
	for (int i = 1; i < NUMTESTS; ++i)
	{
		if (min > results[i])
		{
			min = results[i];
			minTime = resultsTime[i];
		}
	}

	// Output results
	std::ofstream out(filename.c_str());

	out << std::endl << "Minimum time:  " << min << "\t(" << minTime << " ms)" << std::endl << std::endl;
	out << "Test #\t\t" << "Time" << std::endl;

	for (int i = 0; i < NUMTESTS; ++i)
		out << i + 1 << ":\t\t" << results[i] << "\t(" << resultsTime[i] << " ms)" << std::endl;

	out.close();

	FinishTest(agent, movement_setting, false);
}

void PathfindingTests::RunAllTests(const char *filename_noext, Agent &agent)
{
	MovementSetting movement_setting;
	Movement& movement = agent.m_owner->GetMovement();
	int heuristic = movement.GetHeuristicCalc();
	float weight = movement.GetHeuristicWeight();

	PrepareTest(agent, movement_setting, heuristic, weight, false);

	//int current_map = 1;
	//g_terrain.UpdateMap(current_map);

	//while (current_map != 0)
	{
		int current_map = g_terrain.GetMapIndex();

		// update file name
		std::string filename = AddMapIndexToFilename(filename_noext, current_map);

		PathFindingOutcomeArray outcome;

		// do stuff
		int width = g_terrain.GetWidth();

		for (int r_start = 0; r_start < width; ++r_start)
		{
			for (int c_start = 0; c_start < width; ++c_start)
			{
				for (int r_goal = 0; r_goal < width; ++r_goal)
				{
					for (int c_goal = 0; c_goal < width; ++c_goal)
					{
						if ((r_start == r_goal && c_start == c_goal) ||
							(g_terrain.IsWall(r_start, c_start)) ||
							(g_terrain.IsWall(r_goal, c_goal))
							)
						{
							continue;
						}

						// pathfinding tests
						PathfindingOutcome outcome_data;

						outcome_data.m_mapindex = current_map;
						RunPathfindingTest(r_start, c_start, r_goal, c_goal, agent, outcome_data);
						outcome.push_back(outcome_data);
					}
				}
			}
		}

		std::sort(outcome.begin(), outcome.end(),
			[](const PathfindingOutcome& lhs, const PathfindingOutcome& rhs)
		{
			return (lhs.m_dist > rhs.m_dist);
		});

		// Output results
		SavePathfindingOutcomes(filename.c_str(), outcome);

		g_terrain.NextMap();
	}

	FinishTest(agent, movement_setting, false);
}

// create sample test file from total test file
// number of big tests: number of tests with longest distance
// number of failed tests: number of tests that can't find goal
// number of total tests: total number of tests, include big tests and failed tests
void PathfindingTests::CreateSampleTest(const char *loadfile_noext, const char *savefile_noext,
	int mapindex, int num_totaltests, int num_bigtests, int num_failedtests)
{
	PathFindingOutcomeArray outcomes_success;
	PathFindingOutcomeArray outcomes_failure;
	PathFindingOutcomeArray outcomes_selected;

	LoadTestData(loadfile_noext, mapindex, outcomes_success, outcomes_failure);

	// first select the first nth tests from as big tests
	if (static_cast<int>(outcomes_success.size()) < num_bigtests)
	{
		outcomes_selected = outcomes_success;
		num_bigtests = static_cast<int>(outcomes_selected.size());
	}
	else
	{
		for (int i = 0; i < num_bigtests; ++i)
			outcomes_selected.emplace_back(outcomes_success[i]);
	}

	// then select failed tests
	if (static_cast<int>(outcomes_failure.size()) < num_failedtests)
	{
		for (PathFindingOutcomeArray::iterator it = outcomes_failure.begin();
			it != outcomes_failure.end(); ++it)
		{
			outcomes_selected.emplace_back(*it);
		}
	}
	else
	{
		std::set<int> selected_indices;
		int upper_boundary = outcomes_failure.size() - 1;

		while (static_cast<int>(selected_indices.size()) < num_failedtests)
			selected_indices.insert(g_random.RangeInt(0, upper_boundary));

		for (std::set<int>::iterator it = selected_indices.begin();
			it != selected_indices.end(); ++it)
		{
			outcomes_selected.emplace_back(outcomes_failure[*it]);
		}
	}

	// then select the rest
	{
		int num_tests = num_totaltests - outcomes_selected.size();
		if (num_tests > 0)
		{
			if (static_cast<int>(outcomes_success.size()) - num_bigtests < num_tests)
			{
				for (unsigned i = num_bigtests; i < outcomes_success.size(); ++i)
					outcomes_selected.emplace_back(outcomes_success[i]);
			}
			else
			{
				std::set<int> selected_indices;
				int upper_boundary = outcomes_success.size() - 1;

				while (static_cast<int>(selected_indices.size()) < num_tests)
					selected_indices.insert(g_random.RangeInt(num_bigtests, upper_boundary));

				for (std::set<int>::iterator it = selected_indices.begin();
					it != selected_indices.end(); ++it)
				{
					outcomes_selected.emplace_back(outcomes_success[*it]);
				}
			}
		}
	}

	// then save to new file
	std::string filename = AddMapIndexToFilename(savefile_noext, mapindex);
	std::ofstream out(filename);

	for (PathFindingOutcomeArray::iterator it = outcomes_selected.begin();
		it != outcomes_selected.end(); ++it)
	{
		AddOutcomeToFileStream(*it, out);
	}

	out.close();
}

// pathfinding test against sample test data
bool PathfindingTests::RunSampleTest(const char *filename, const char *out_filename,
	Agent &agent, unsigned num_faileddata, int heuristic, float weight)
{
	PathFindingOutcomeArray failed_outcomes;
	MovementSetting movement_setting;
	Movement& movement = agent.m_owner->GetMovement();

	PrepareTest(agent, movement_setting, heuristic, weight, false);

	PathFindingOutcomeArray outcomes;
	LoadTestData(filename, outcomes);

	for (PathFindingOutcomeArray::iterator it = outcomes.begin(); it != outcomes.end(); ++it)
	{
		// pathfinding tests
		PathfindingOutcome outcome_data;

		outcome_data.m_mapindex = it->m_mapindex;
		g_terrain.UpdateMap(it->m_mapindex);
		RunPathfindingTest(it->m_rStart, it->m_cStart, it->m_rGoal, it->m_cGoal, agent, outcome_data);

		if (outcome_data != *it)
		{
			failed_outcomes.emplace_back(outcome_data);

			if (failed_outcomes.size() > num_faileddata)
				break;
		}
	}

	FinishTest(agent, movement_setting, false);

	if (failed_outcomes.size() > 0)
	{
		SavePathfindingOutcomes(out_filename, failed_outcomes);
		g_blackboard.SetSampleTestStatus(Status_Fail);

		return false;
	}

	g_blackboard.SetSampleTestStatus(Status_Pass);
	std::ofstream out(out_filename);
	out << "All Pass!\n";
	out.close();

	return true;
}

// get simple test result
void PathfindingTests::GetSampleTestResult(const char *filename, Agent &agent, bool is_increasing,
	bool is_straightline, bool is_rubberband, bool is_smooth, FileLoadSample sample)
{
	if ((m_outcome_index < 0) || (m_sample != sample))
	{
		m_outcomes.clear();

		std::ifstream in(filename);
		std::string instr;
		std::getline(in, instr);
		if (instr != "All Pass!")
		{
			LoadTestData(filename, m_outcomes);
			m_outcome_index = 0;

			m_sample = sample;
		}
		else
			return;
	}

	Movement& movement = agent.m_owner->GetMovement();
	MovementSetting movement_setting;
	PathfindingOutcome &outcome = m_outcomes[m_outcome_index];

	g_terrain.UpdateMap(outcome.m_mapindex);
	PrepareTest(agent, movement_setting, movement.GetHeuristicCalc(), movement.GetHeuristicWeight(), false);

	if (is_straightline)
		movement.SetStraightlinePath(true);
	if (is_rubberband)
		movement.SetRubberbandPath(true);
	if (is_smooth)
		movement.SetSmoothPath(true);

	g_terrain.ResetColors();
	g_blackboard.UpdatePlayerPos(outcome.m_rStart, outcome.m_cStart);
	agent.m_owner->GetBody().SetPos(g_terrain.GetCoordinates(outcome.m_rStart, outcome.m_cStart));
	movement.ComputePathWithTiming(outcome.m_rGoal, outcome.m_cGoal, true);

	FinishTest(agent, movement_setting, false);

	if (is_increasing)
	{
		++m_outcome_index;
		if (m_outcome_index >= static_cast<int>(m_outcomes.size()))
			m_outcome_index = 0;
	}
	else
	{
		--m_outcome_index;
		if (m_outcome_index < 0)
			m_outcome_index = m_outcomes.size() - 1;
	}

}
