#include <iostream>
#include<fstream>
#include <unistd.h>
#include <cstdlib>
#include <bits/stdc++.h>

using namespace std;

#include <time.h> 
    
void delay(float delay) 
{ 
    int now=time(NULL); 
    int later=now+delay; 
    while(now<=later)now=time(NULL); 
}

void read()
{
    ifstream fin;
    string line;
    // Open an existing file
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

            cout<<"X = " << v[0] << " Z = " << v[1] << endl;
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