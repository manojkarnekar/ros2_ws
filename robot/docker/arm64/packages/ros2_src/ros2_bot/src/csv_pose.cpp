#include <iostream>
#include<fstream>
#include <unistd.h>
#include <cstdlib>
#include <bits/stdc++.h>

using namespace std;

void read()
{
    ifstream fin;
    string line;
    // fin.open("/home/nilutpolk/zed-yolo/zed_cpp_sample/coordinates.csv");
    while(1)
    {
        try 
        {
            fin.open("/home/nilutpolk/zed-yolo/zed_cpp_sample/coordinates.csv");
            fin>>line;
            string str = line;

            stringstream ss(str);
            vector<string> v;
            
            while (ss.good()) {
                string substr;
                getline(ss, substr, ',');
                v.push_back(substr);
            }

            cout<<str<<" "<<"X = " << std::stof(v[0]) << " Z = " << std::stof(v[1]) << endl;
            cout<<"----------------------------------------------"<<std::endl;
            // sleep(0.1);
            fin.close();

            v.clear();
            
        }
        catch (...) {
            continue;
        }
        
    }
    }

int main()
{
    read();
    return 0;
}