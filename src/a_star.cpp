/*
* A-star Search Algorithm
*
*/
#include "a_star.h"

using namespace std;

#define INF (unsigned)!((int)0)
#define DEBUG false


struct node {
    pair<int, int> location; 
    double totalCost, heuristicCost, gridCost;
    pair<int, int> parentLocation;
    void calculateHeuristicCost(pair<int, int> endPixel) {
        //Euclidean distance
        heuristicCost = (double) sqrt( ((endPixel.first-location.first)*(endPixel.first-location.first))
                         +  ((endPixel.second-location.second)*(endPixel.second-location.second)) );
        //Manhattan distance
        // heuristicCost = fabs(endPixel.first - location.first) + fabs(endPixel.second - location.second);
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
            //Uncomment the if for 4 neighbours
            // if((*row!=currentPixel.first) && (*col!=currentPixel.second))
            //     continue;
            //check for validity
            if(checkIfIndexInvalid(MAX_ROW, MAX_COL, make_pair(*row,*col)))
                continue;
            neighbours.push_back(make_pair(*row,*col));
        }
    }
    return neighbours;
}

vector<pair<int, int>> findNeighbours(cv::Mat image, pair<int, int> currentPixel) {
    int MAX_ROW = image.cols;
    int MAX_COL = image.rows;
    vector<pair<int, int>> neighbours = getAllValidIndices(MAX_ROW, MAX_COL,currentPixel), validNeighbours;
    
    if(DEBUG){
        cout<<"neighbours ";
    }
    
    for(uint i=0; i<neighbours.size(); i++){  
        if(DEBUG){ cout<<'('<<neighbours[i].first<<','<<neighbours[i].second<<')'<<' '; }          
        //check for collision on itself and its neighbours too
        cv::Vec3b color = image.at<cv::Vec3b>(neighbours[i].second, neighbours[i].first);
        if(color[0]==0 && color[1]==0 && color[2]==0){
            vector<pair<int, int>> nNeighbours = getAllValidIndices(MAX_ROW, MAX_COL,neighbours[i]);
            bool flag = true;
            for(uint j=0; j<nNeighbours.size(); j++){ 
                cv::Vec3b ncolor = image.at<cv::Vec3b>(nNeighbours[i].second, nNeighbours[i].first);
                if(ncolor[0]!=0 || ncolor[1]!=0 || ncolor[2]!=0){
                    flag = false;
                    break;
                }
            }
            if(flag){
                validNeighbours.push_back(neighbours[i]);
            }              
        }
    }

    if(DEBUG){
        cout<<'\n';
    }
    return validNeighbours;
}

vector<pair<int, int>> aStarSearch(cv::Mat image, pair<int, int> startPixel, pair<int, int> endPixel) {
    //check if start and end are accessible
    cv::Vec3b scolor = image.at<cv::Vec3b>(startPixel.second, startPixel.first);
    cv::Vec3b ecolor = image.at<cv::Vec3b>(endPixel.second, endPixel.first);
    if(scolor[0]!=0 || scolor[1]!=0 || scolor[2]!=0 ||
       ecolor[0]!=0 || ecolor[1]!=0 || ecolor[2]!=0){
        printf("ERROR: Invalid start or end pixel value, planning is not possible. Returning. . .\n");
        return(vector<pair<int, int>> {0});
    }

    //check if start and end are valid
    if(checkIfIndexInvalid(image.cols, image.rows, startPixel) ||
       checkIfIndexInvalid(image.cols, image.rows, endPixel)){
        printf("ERROR: Out of bounds start or end pixel, planning is not possible. Returning. . .\n");
        return(vector<pair<int, int>> {0});
    }

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
        
        if(DEBUG){
            printf("********************************************************************************\n");
            printf("ParentNode: %d,%d TCost %f\n",minimumCostNode.location.first,minimumCostNode.location.second, minimumCostNode.totalCost);
        }

        vector<pair<int, int>> neighboursLocationList = findNeighbours(image, minimumCostNode.location);

        //neighbour to node, parent is the mimnumcostnode
        for(auto neighbour=neighboursLocationList.begin(); neighbour!=neighboursLocationList.end(); neighbour++) {
            node currentNeighbour;
            currentNeighbour.location = *neighbour;
            currentNeighbour.gridCost = minimumCostNode.gridCost+1;
            currentNeighbour.calculateHeuristicCost(endPixel); 
            currentNeighbour.totalCost = currentNeighbour.heuristicCost + currentNeighbour.gridCost;
            currentNeighbour.parentLocation = minimumCostNode.location;
            
            if(DEBUG){
            printf("Current neighbour: %d,%d TCost %f \n",currentNeighbour.location.first,currentNeighbour.location.second, currentNeighbour.totalCost);
            }

            auto duplicateInExpandedNodes = find_if(expandedNodesList.begin(), expandedNodesList.end(), [neighbour](node& arg)
                    { return ((arg.location.first == neighbour->first) && (arg.location.second == neighbour->second)); });
            if(duplicateInExpandedNodes!=expandedNodesList.end()) {
                if (duplicateInExpandedNodes->totalCost <= currentNeighbour.totalCost) {
                    if(DEBUG){printf("Current neighbour already inexpanded nodes list, discarding \n");}
                    continue; 
                }
                else {
                    expandedNodesList.erase(duplicateInExpandedNodes);
                }                
            }

            auto duplicateInVisitedNodes = find_if(visitedNodesList.begin(), visitedNodesList.end(), [neighbour](node& arg)
                    { return ((arg.location.first == neighbour->first) && (arg.location.second == neighbour->second)); });
            if(duplicateInVisitedNodes!=visitedNodesList.end()) {
                if(duplicateInVisitedNodes->totalCost <= currentNeighbour.totalCost) {
                    if(DEBUG){printf("Current neighbour already in visited nodes list, discarding \n");}
                    continue;
                }
                else {
                    visitedNodesList.erase(duplicateInVisitedNodes);
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
        visitedNodesList.push_back(minimumCostNode);

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