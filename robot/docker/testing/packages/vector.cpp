#include <vector>
#include <iostream>
#include <bits/stdc++.h>
using namespace std;

vector<int> vec_rev(int s, vector<int> v)
{
    v.push_back(s);
    std::reverse(v.begin(), v.end());
    // vector<float> v1(v.end(), v.begin());
    return v;
}

int main(int argc, char * argv[])
{
//   vector<float> v;
  vector<int> v {20,2,5,7,8,10, 30};
//   std::reverse(v.begin(), v.end());
  float s = 10;
  auto v1 = vec_rev(s, v);
  cout<<"vector index and element access"<<endl;
  for(int i=0; i<v1.size();i++)
  {
    cout<<i<<' '<<v1.at(i)<<endl;
  }
  cout<<"-------------------------------"<<endl;
  v1.clear();
  return 0;
}