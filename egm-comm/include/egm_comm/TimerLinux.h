#ifndef _YIFAN_TIMER_LINUX_H_
#define _YIFAN_TIMER_LINUX_H_

#include <cstdlib>
#include <chrono>

class Timer
{
public:
	Timer();
	~Timer();

	void tic();
	double toc(); // return ms

private:
	std::chrono::high_resolution_clock::time_point _t1;
	std::chrono::high_resolution_clock::time_point _t2;
};
	

#endif // _YIFAN_TIMER_LINUX_H_