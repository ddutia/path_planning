/*
* A-star Search Algorithm Header
*
*/
#include<bits/stdc++.h> 
#include <math.h> 
#include <iostream>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

using namespace std;

vector<pair<int, int>> aStarSearch(cv::Mat image, pair<int, int> startPixel, pair<int, int> endPixel);