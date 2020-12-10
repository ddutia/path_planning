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
#define DEBUG false


std::string GetCurrentWorkingDir() {
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

    for(uint i=0; i<fileNamesWithPath.size(); i++) {
        Mat imgGrayscale = imread(fileNamesWithPath[i], IMREAD_COLOR);
        if(imgGrayscale.empty())
        {
            cout<< "Could not read the image: " << fileNamesWithPath[i]<<endl;
            return 1;
        }
        else{
            cout<<"Planning for "<<fileNamesWithPath[i]<<endl;
        }

        if(DEBUG) { cout<<imgGrayscale.rows<<' '<<imgGrayscale.cols<<endl; }
        pair<int,int> start = {0,0}, end = {imgGrayscale.cols-5,imgGrayscale.rows-5};
        vector<pair<int, int>> shortestPath = aStarSearch(imgGrayscale, start, end);

        if(!shortestPath.empty()) {
            if(DEBUG){
                cout<<shortestPath.size()<<'\n';
                //print the shortest path
                for(auto path=shortestPath.end(); path!=shortestPath.begin(); path--){
                    cout<<path->first<<','<<path->second<<'\n';
                }
            }
            //plot the planned path
            //red: start pixel
            //green: end pixel
            //blue: planned path
            Mat imgWithPath = imread(fileNamesWithPath[i], IMREAD_COLOR);
            for (auto pixelXY = shortestPath.begin(); pixelXY!=shortestPath.end(); pixelXY++) {
                imgWithPath.at<Vec3b>(Point((uint)pixelXY->first,(uint)pixelXY->second)) = Vec3b(255, 0, 0);//blue
            }
            imgWithPath.at<Vec3b>(Point((uint)start.first,(uint)start.second)) = Vec3b(0, 0, 255);//red
            imgWithPath.at<Vec3b>(Point((uint)end.first,(uint)end.second)) = Vec3b(0, 255, 0);//green
            // imwrite(filePath+"/img/planned.png", imgWithPath);
            imshow(fileNamesWithPath[i], imgWithPath);
        }
    }
    cout<<"Press any key to exit."<<endl;
    waitKey(0); // Wait for a keystroke in the window    

    return(0); 
}