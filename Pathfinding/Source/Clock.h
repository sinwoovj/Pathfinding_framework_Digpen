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

class Clock
{
public:

	Clock(void);
	~Clock(void) {}

	void MarkTimeThisTick( void );
	inline float GetElapsedTime( void )			{ return( m_timeLastTick ); }
	inline float GetCurTime( void )				{ return( m_currentTime ); }
	inline double GetAbsoluteTime( void )		{ return( m_timer.GetAbsoluteTime() ); }
	inline double GetHighestResolutionTime( void )	{ LARGE_INTEGER qwTime; QueryPerformanceCounter( &qwTime ); return((double)qwTime.QuadPart); }

	inline void ClearQPCFrequency(void)				{ m_qpcfrequency = 0.0; }
	void UpdateQPCFrequency(void);
	inline double GetQPCFrequency(void)				{ return m_qpcfrequency; }

	inline void ClearStopwatchPathfinding( void )	{ m_stopwatchValuePathfinding = 0.0; }
	inline void StartStopwatchPathfinding( void )	{ m_stopwatchStartTimePathfinding = GetHighestResolutionTime(); }
	inline void StopStopwatchPathfinding( void )	{ m_stopwatchValuePathfinding += GetHighestResolutionTime() - m_stopwatchStartTimePathfinding; }
	inline double GetStopwatchPathfinding( void )	{ return m_stopwatchValuePathfinding; }

	inline double GetStopwatchPathfindingTime(void)	{ return m_stopwatchValuePathfinding / m_qpcfrequency; }

	inline void ClearStopwatchAnalysis( void )	{ m_stopwatchValueAnalysis = 0.0; }
	inline void StartStopwatchAnalysis( void )	{ m_stopwatchStartTimeAnalysis = GetHighestResolutionTime(); }
	inline void StopStopwatchAnalysis( void )	{ m_stopwatchValueAnalysis += GetHighestResolutionTime() - m_stopwatchStartTimeAnalysis; }
	inline double GetStopwatchAnalysis( void )	{ return m_stopwatchValueAnalysis; }

private:

	unsigned int m_startTime;
	float m_currentTime;
	float m_timeLastTick;
	CDXUTTimer m_timer;

	double m_stopwatchStartTimePathfinding, m_stopwatchStartTimeAnalysis;
	double m_stopwatchValuePathfinding, m_stopwatchValueAnalysis;

	double m_qpcfrequency;					// frequency used for QPC

};
