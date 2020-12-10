/*
* A-star Search Algorithm
*
*/
#include "a_star.h"

using namespace std;

#define INF (unsigned)!((int)0)
#define DEBUG true


struct node {
    pair<int, int> location; 
    double totalCost, heuristicCost, gridCost;
    pair<int, int> parentLocation;
    void calculateHeuristicCost(pair<int, int> endPixel) {
        heuristicCost = (double) sqrt( ((endPixel.first-location.first)*(endPixel.first-location.first))
                         +  ((endPixel.second-location.second)*(endPixel.second-location.second)) );
    }
};

bool checkIfIndexInvalid(int MAX_ROW, int MAX_COL, pair<int, int> currentPixel) {
    if(currentPixel.first<0 || currentPixel.first>=MAX_ROW ||
       currentPixel.second<0 || currentPixel.second>=MAX_COL)
        return true;

    return false;
}

vector<pair<int, int>> getAllValidIndices(int MAX_ROW, int MAX_COL, pair<int, int> currentPixel) {
    //finding all valid indices 
    vector<int> rows = {currentPixel.first-1, currentPixel.first, currentPixel.first+1};
    vector<int> columns = {currentPixel.second-1, currentPixel.second, currentPixel.second+1};
    vector<pair<int, int>> neighbours;

    for(auto row=rows.begin(); row!=rows.end(); row++) {
        for(auto col=columns.begin(); col!=columns.end(); col++) {
            if((*row==currentPixel.first) && (*col==currentPixel.second))
                continue;
            if(checkIfIndexInvalid(MAX_ROW, MAX_COL, make_pair(*row,*col)))
                continue;
            neighbours.push_back(make_pair(*row,*col));
        }
    }
    return neighbours;
}

vector<pair<int, int>> findNeighbours(cv::Mat inputImageGrid, pair<int, int> currentPixel) {
    int MAX_ROW = inputImageGrid.rows;
    int MAX_COL = inputImageGrid.cols;
    vector<pair<int, int>> neighbours = getAllValidIndices(MAX_ROW, MAX_COL,currentPixel), validNeighbours;
    
    if(DEBUG){
        cout<<"neighbours ";
    }
    
    for(uint i=0; i<neighbours.size(); i++){  
        if(DEBUG){ cout<<'('<<neighbours[i].first<<','<<neighbours[i].second<<')'<<' '; }          
        cv::Vec3b color = inputImageGrid.at<cv::Vec3b>(neighbours[i].second, neighbours[i].first);
        if(color[0]==0 && color[1]==0 && color[2]==0){
            validNeighbours.push_back(neighbours[i]);
        }              
    }

    if(DEBUG){
        cout<<'\n';
    }
    return validNeighbours;
}

vector<pair<int, int>> aStarSearch(cv::Mat inputImageGrid, pair<int, int> startPixel, pair<int, int> endPixel) {
    bool isEndReached = false;

    node startNode, endNode;
    startNode.location = startPixel;
    startNode.gridCost = 0;
    startNode.calculateHeuristicCost(endPixel);
    startNode.totalCost = startNode.heuristicCost + startNode.gridCost;
    
    vector<node> expandedNodesList;
    vector<node> visitedNodesList;

    expandedNodesList.push_back(startNode);

    while (!expandedNodesList.empty()){
        auto minimumCostNodeItr = expandedNodesList.begin();
        node minimumCostNode = expandedNodesList[0];
        for(uint i = 0; i<expandedNodesList.size(); i++){
            if(minimumCostNode.totalCost > expandedNodesList[i].totalCost){
                minimumCostNode = expandedNodesList[i];
                minimumCostNodeItr = expandedNodesList.begin() + i;
            }
        }
        expandedNodesList.erase(minimumCostNodeItr);
        visitedNodesList.push_back(minimumCostNode);


        vector<pair<int, int>> neighboursLocationList = findNeighbours(inputImageGrid, minimumCostNode.location);

        //neighbour to node, parent is the mimnumcostnode
        for(auto neighbour=neighboursLocationList.begin(); neighbour!=neighboursLocationList.end(); neighbour++) {
            node currentNeighbour;
            currentNeighbour.location = *neighbour;
            currentNeighbour.gridCost = minimumCostNode.gridCost+1;
            currentNeighbour.calculateHeuristicCost(endPixel); 
            currentNeighbour.totalCost = currentNeighbour.heuristicCost + currentNeighbour.gridCost;
            currentNeighbour.parentLocation = minimumCostNode.location;
            

            auto duplicateInExpandedNodes = find_if(expandedNodesList.begin(), expandedNodesList.end(), [neighbour](node& arg)
                    { return ((arg.location.first == neighbour->first) && (arg.location.second == neighbour->second)); });
            if(duplicateInExpandedNodes!=expandedNodesList.end()) {
                if (duplicateInExpandedNodes->totalCost <= currentNeighbour.totalCost) {
                    if(DEBUG){printf("Current neighbour already inexpanded nodes list, discarding \n");}
                    continue; 
                }            
            }

            auto duplicateInVisitedNodes = find_if(visitedNodesList.begin(), visitedNodesList.end(), [neighbour](node& arg)
                    { return ((arg.location.first == neighbour->first) && (arg.location.second == neighbour->second)); });
            if(duplicateInVisitedNodes!=visitedNodesList.end()) {
                if(duplicateInVisitedNodes->totalCost <= currentNeighbour.totalCost) {
                    if(DEBUG){printf("Current neighbour already in visited nodes list, discarding \n");}
                    continue;
                }
            }

            if(DEBUG){printf("Added current neighbour in expanded nodes list! \n");}
            expandedNodesList.push_back(currentNeighbour);

            //is neighbour the goal
            if((currentNeighbour.location.first == endPixel.first) && (currentNeighbour.location.second == endPixel.second)) {
                endNode = currentNeighbour;
                if(DEBUG){
                    printf("End is near %d,%d \n", endNode.parentLocation.first, endNode.parentLocation.second);
                }
                isEndReached = true;
                break;
            }
        }

        if(isEndReached){
            break;
        }
    }
    //trace the shortest path
    vector<pair<int, int>> shortestPath;
    shortestPath.push_back(endNode.location);
    pair<int, int> traceNodeLocation = endNode.parentLocation;
    if(DEBUG){
        printf("Endnode parent: %d,%d \n", endNode.parentLocation.first,endNode.parentLocation.second);
        printf("Visited node list size: %lu \n", visitedNodesList.size());
    }
    while((traceNodeLocation.first != startNode.location.first) || (traceNodeLocation.second != startNode.location.second)) {
        shortestPath.push_back(traceNodeLocation);
        auto traceParentItr = find_if(visitedNodesList.begin(), visitedNodesList.end(), [traceNodeLocation](node& arg)
                    { return ((traceNodeLocation.first == arg.location.first) && (traceNodeLocation.second == arg.location.second)); });
        traceNodeLocation = traceParentItr->parentLocation;
    }
    shortestPath.push_back(startNode.location);

    return shortestPath;
}



/* Standalone Test Program *
int main() 
{ 
    //0--> The cell is not blocked 
    //255--> The cell is blocked    
    vector<vector<int>> inputImageGrid = 
    { 
        { 0, 0, 0, 0, 0, 0, 255, 0, 0, 0 }, 
        { 0, 0, 0, 255, 0, 0, 0, 255, 0, 0 }, 
        { 0, 0, 0, 255, 0, 0, 255, 0, 255, 255 }, 
        { 255, 255, 0, 255, 0, 255, 255, 255, 255, 0 }, 
        { 0, 0, 0, 255, 0, 0, 0, 255, 0, 0 }, 
        { 0, 255, 0, 0, 0, 0, 255, 0, 255, 0 }, 
        { 0, 255, 255, 255, 255, 0, 255, 255, 255, 0 }, 
        { 0, 255, 0, 0, 0, 0, 255, 0, 0, 0 }, 
        { 0, 0, 0, 255, 255, 255, 0, 255, 255, 255 } 
    };
  
    pair<int, int> startPixel = make_pair(0, 0); 
    pair<int, int> endPixel = make_pair(7, 0); 
  
    vector<pair<int, int>> shortestPath = aStarSearch(inputImageGrid, startPixel, endPixel); 
    cout<<shortestPath.size()<<'\n';
    //print the shortest path
    for(auto path=shortestPath.end(); path!=shortestPath.begin(); path--){
       cout<<path->first<<','<<path->second<<'\n';
    }

    return(0); 
}
**/