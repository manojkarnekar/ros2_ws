#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
 
using namespace std;
// char* csv_path;

void read_csv(string fname)
{
	vector<vector<string>> content;
	vector<string> row;
	string line, word;
 
	fstream file (fname, ios::in);
	if(file.is_open())
	{
		while(getline(file, line))
		{
			row.clear();
 
			stringstream str(line);
 
			while(getline(str, word, ','))
				row.push_back(word);
			content.push_back(row);
		}
	}
	else
  {
    cout<<"Could not open the file\n";
  }
 
	for(int i=1;i<content.size();i++)
	{
    cout<<stof(content[i][2])<<" , "<<stof(content[i][3])<<"\n";
	}
}
 
int main()
{
  read_csv("vertices.csv");
	return 0;
}
 
