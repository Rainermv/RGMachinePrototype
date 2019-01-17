/******************************************************************************\
* rhostring.cpp                                                                *
*                                                                              *
* Rossana B. Queiroz                                                           *
\******************************************************************************/

#include "rhostring.hpp"

vector <string> tokenize(string buffer, string delim) {
   vector <string> tokens;
   string strbuffer(buffer);
   string token;
   int pos;

   do {
      pos = strbuffer.find(delim);
      if (pos > 0) {
         token = strbuffer.substr(0,pos);
         strbuffer = strbuffer.substr(pos+delim.size());
         tokens.push_back(token);
      }
      else tokens.push_back(strbuffer);
   } while(pos > 0);

   return tokens;
}

vector <string> tokenize(string buffer){
   vector <string> tokens;
   string token;
   istringstream sbuffer(buffer);

   do {
      token.clear();
      sbuffer >> token;
      if (!token.empty())
         tokens.push_back(token);
   } while (!token.empty());
   return tokens;
}

void print_tokens(vector <string> tokens) {
   for (unsigned int i=0; i<tokens.size(); i++) {
      //cout << tokens[i] << "\t";
   }
   //cout << endl;
}
