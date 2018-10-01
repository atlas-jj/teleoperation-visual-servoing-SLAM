/*************************************************************************
	> File Name: triangular_dist.cpp
	> Jun Jin.:
	> Mail:jjin5@ualberta.ca
    > ---- triangular distribution functions.----
	> Created Time: Mon 20 Feb 2017 09:23:14 PM MST
 ************************************************************************/

#include "../include/string_convertor.h"

string string_convertor::d2s(double d)
{
  std::ostringstream d2str;
  d2str<<d;
  return d2str.str();
}


std::vector<float> string_convertor::convert2Float(std::vector<double> v)
{
  std::vector<float> t;
  for(int i=0;i<v.size();i++)
    t.push_back((float)v[i]);
  return t;
}

void string_convertor::printOutStdVector(std::vector<double> v)
{
 //cout<<"print out v:"<<endl;
 for(int i=0;i<v.size();i++)
 {
   cout<<v.at(i)<<"  ";
 }
 cout<<endl;
}

/**1 2 3 4 5 splitor: space
* input string 1 2 3 4 5 6
output array [1,2,3,4,5,6]
*/
std::vector< double > string_convertor::fromString2Array(string inStr)
{
   std::vector< double > vd;
   string buf; // Have a buffer string
   stringstream ss(inStr); // Insert the string into a stream
   while (ss >> buf)
       vd.push_back(atof(buf.c_str()));
   return vd;
}

/**
input string 1 2 3 4 5 6
output string array ["1","2","3","4","5","6"]
*/
std::vector< string > string_convertor::fromString2ArrayStr(string inStr)
{
   std::vector< string > vd;
   string buf; // Have a buffer string
   stringstream ss(inStr); // Insert the string into a stream
   while (ss >> buf)
       vd.push_back((buf.c_str()));
   return vd;
}

string string_convertor::constructPubStr(vector< vector<Point> > vps)
{
   string rtStr="";
   size_t strokesNum=vps.size();
   for(int i=0;i<strokesNum;i++)
   {
      size_t pointsNum=vps[i].size();
      if(pointsNum>0)
      {
        string thisLineStr="";
        for(int j=0;j<pointsNum-1;j++)
          thisLineStr += d2s(vps[i][j].x)+" "+d2s(vps[i][j].y)+" ";
        thisLineStr+=d2s(vps[i][pointsNum-1].x)+" "+d2s(vps[i][pointsNum-1].y);
        if(i==strokesNum-1)
          rtStr+=thisLineStr;
        else
          rtStr+=thisLineStr+";";
      }
   }
   return rtStr;
}

string string_convertor::constructPubStr(vector< vector<Point2d> > vps)
{
   string rtStr="";
   size_t strokesNum=vps.size();
   for(int i=0;i<strokesNum;i++)
   {
      size_t pointsNum=vps[i].size();
      if(pointsNum>0)
      {
        string thisLineStr="";
        for(int j=0;j<pointsNum-1;j++)
          thisLineStr += d2s(vps[i][j].x)+" "+d2s(vps[i][j].y)+" ";
        thisLineStr+=d2s(vps[i][pointsNum-1].x)+" "+d2s(vps[i][pointsNum-1].y);
        if(i==strokesNum-1)
          rtStr+=thisLineStr;
        else
          rtStr+=thisLineStr+";";
      }
   }
   return rtStr;
}

std::vector<string> string_convertor::split(const std::string &s, char delim) {
    std::vector<string> elems;
    std::stringstream ss(s);
    std::string number;
    while(std::getline(ss, number, delim)) {
        elems.push_back(number);
    }
    return elems;
}
