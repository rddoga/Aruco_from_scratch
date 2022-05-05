#include "utils.h"

// is the param ?
bool CmdLineParser::operator[](std::string param)
{
    int idx = -1;
    for (int i = 0; i < argc && idx == -1; i++)if (std::string(argv[i]) == param) idx = i;return (idx != -1);
}

//return the value of a param using a default value if it is not present
std::string CmdLineParser::operator()(std::string param, std::string defvalue)
{
    int idx = -1;
    for (int i = 0; i < argc && idx == -1; i++)if (std::string(argv[i]) == param) idx = i;if (idx == -1) return defvalue;else return (argv[idx + 1]);
}

