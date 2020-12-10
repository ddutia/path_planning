#include <iostream>
#include <unistd.h>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "a_star.h"

using namespace std;
using namespace cv;

#define GetCurrentDir getcwd
#define DEBUG true


std::string GetCurrentWorkingDir( void ) {
  char buff[FILENAME_MAX];
  GetCurrentDir( buff, FILENAME_MAX );
  std::string current_working_dir(buff);
  return current_working_dir;
}

int main()
{
    if(DEBUG) { cout << GetCurrentWorkingDir() <<endl; }
    string filePath = GetCurrentWorkingDir();
    
    vector<cv::String> fileNamesWithPath;
    glob(filePath+"/img/*.png", fileNamesWithPath, false);

    for(uint i=2; i<fileNamesWithPath.size(); i++) {
        Mat imgGrayscale = imread(fileNamesWithPath[i], IMREAD_COLOR);

        pair<int,int> start = {0,0}, end = {(int)imgGrayscale.rows-1,(int)imgGrayscale.cols-1};
        vector<pair<int, int>> shortestPath = aStarSearch(imgGrayscale, start, end);

        if(!shortestPath.empty()) {
            if(DEBUG){
                cout<<shortestPath.size()<<'\n';
                //print the shortest path
                for(auto path=shortestPath.end(); path!=shortestPath.begin(); path--){
                    cout<<path->first<<','<<path->second<<'\n';
                }
            }
        }
    } 

    return(0); 
}