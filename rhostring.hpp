/******************************************************************************\
* rhostring.hpp: a lib for tokenize happily                                    *
*                                                                              *
* Rossana Queiroz                                                              *
\******************************************************************************/

#ifndef _RHOSTRING_HPP_
#define _RHOSTRING_HPP_

//#include <cstdio>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>

#define SIZE_ 50000

using namespace std;

vector <string> tokenize(string buffer, string delim);
void print_tokens(vector <string> tokens);
vector <string> tokenize(string buffer);

std::vector<int> getZone(char *filename);

#endif // _RHOSTRING_HPP_
