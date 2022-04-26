#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <vector>
#include <chrono>

/************************************
 *
 *  UTILS
 *
 ************************************/

struct TimerAvrg
{
	std::vector<double> times;
	size_t curr = 0,n;
	std::chrono::high_resolution_clock::time_point begin, end;

	TimerAvrg(int _n = 30)
	{
	    n = _n;
	    times.reserve(n);
	}

	inline void start()
	{
	    begin= std::chrono::high_resolution_clock::now();
	}

	inline void stop()
	{
	    end = std::chrono::high_resolution_clock::now();
	    double duration = double(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()) * 1e-6;
	    if (times.size() < n)
		    times.push_back(duration);
	    else
	    {
		    times[curr] = duration;
                    curr++;
		    if (curr >= times.size())
		        curr = 0;
	    }
	}

	double getAvrg()
	{
	    double sum=0;
	    for(auto t:times)
		sum+=t;
	    return sum/double(times.size());
	}
};


class CmdLineParser
{
    int argc;
    char** argv;

public:
    CmdLineParser(int _argc, char** _argv): argc(_argc), argv(_argv) {}

    //is the param?
    bool operator[](std::string param);

    //return the value of a param using a default value if it is not present
    std::string operator()(std::string param, std::string defvalue = "-1");
};

#endif
