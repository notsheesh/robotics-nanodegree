#include <iostream>
#include <string.h>
#include <vector>
#include <algorithm>
#include<string>
using namespace std;

#define HEIGHT 5  			// row
#define WIDTH 6   			// col
#define START_X 0 			// row
#define START_Y 0 			// col
// #define GOAL_X HEIGHT- 1    // row
#define GOAL_X 0    // row

#define GOAL_Y WIDTH - 1	// col 

typedef vector<vector<int>> vec2d_int;
typedef vector<int> vec1d_int;

class Map{
	public:
	int mapWidth = HEIGHT;
	int mapHeight = WIDTH;
	vector<vector<int>> grid={
	   // v start
        { 0, 1, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0 },
        { 0, 0, 0, 1, 1, 0 } 
        			  // ^ goal
	};
};

class Planner : public Map{
	public:
	int start[2] = {START_X, START_Y};
	int goal[2] = {mapHeight - 1, mapWidth - 1};
	int cost = 1;
	string movements_arrows[4] = {"^", ">", "v", "<"};
	vec2d_int movements = {
		{-1, 0},
		{ 0, 1},
		{+1, 0},
		{ 0, -1}};
};

// Print function 
template <typename T>
void print2DVector(T Vec){
	for(int i = 0; i < Vec.size(); i++){
		for(int j = 0; j < Vec[0].size(); j++){
			cout << Vec[i][j] << ' ';
		}
		cout << "\n";
	}
}

template <typename T>
void print1DVector(T Vec){
	for(auto i : Vec)
		cout << i << " ";
	cout << "\n";
}

bool isGoal(int x, int y){
	return (x==GOAL_X) && (y==GOAL_Y);
}


// Check if node is valid 
bool isValid(int x, int y, vec2d_int &grid){
	bool isX 	= x < HEIGHT && x >= 0 	  ? true : false;
	bool isY 	= y < WIDTH  && y >= 0	  ? true : false;
	// Check if in grid range
	if(isX && isY){
		// Check if is free
		bool isFree = grid[x][y] == 0 ? true : false;
		return isFree;
	}	
	return false;
}

// Check if node is already in the openList 
bool isOpen(vec1d_int &test, vec2d_int &list){
	bool isX, isY;
	for(auto exist : list){
		isX = test[1] == exist[1] ? true : false;
		isY = test[2] == exist[2] ? true : false;
		bool isOpen = isX && isY;
		if(isOpen){
			// It is
			return isOpen;
		}
	}
	// Nope, go on and add it! 
	return false;
}


void describe(string action, bool verbose, vec1d_int node, vec2d_int list){
	if(!verbose){return;}
	if(action == "isOpen"){
		cout << "Node already open. Skipping: " << endl;
		print1DVector(node);
	}

	if("invalid"){
		cout << "Invalid Node" << endl;
		print1DVector(node);
	}

	if(action == "close"){
		cout << "Closing current Node" << endl;
		print1DVector(node);		
	}

	if(action == "map"){
		cout << "Explored Map: " << endl;
		print2DVector(list);
	}

	if("openList"){
		cout << "\nOpen list: " << endl;
		print2DVector(list);
		cout << "============================" << endl;

	}
}


// Search & Expansion function 
void search(Map map, Planner planner, bool verbose){
	auto closedList = map.grid; 
	int x = planner.start[0];
	int y = planner.start[1];
	int g = 0; // cost of edge
	int x_curr, y_curr, g_curr;
	int x_next, y_next, g_next;

	vec1d_int openNode = {g, x, y}; 
	vec2d_int openList;
	// Add the first open node-> start 
	openList.push_back(openNode);
	cout << "Starting @ node:" << endl; 
	print1DVector(openNode);

	bool isFound = false;
	bool isOver = false;

	while(!isFound && !isOver){

		cout << "Open list: " << endl;
		print2DVector(openList);
		cout << "============================" << endl;
		// describe("openList", verbose, openNode, openList);

		// Check if any openNodes ? explore : exit
		if(openList.size() == 0){
			isOver = true;
			cout << "Failed to reach goal :/\n"; 
		}

		/*
			Expansion strategy 
			Sort the openList prioritizing g
			Pick the node with lowest g  
				Check if goal? exit : continue
				Expand for related nodes (children)
		*/	
		else{
			// Get cheapest node 
			sort(openList.begin(), openList.end());
			reverse(openList.begin(), openList.end());
			vec1d_int currNode = openList.back();
			openList.pop_back();

			// Explore current node 
			g_curr = currNode[0];
			x_curr = currNode[1];
			y_curr = currNode[2];

			// Check if newNode == Goal node 
			if(isGoal(x_curr, y_curr)){
				isFound = true;
				cout << "Goal node found!\n";
				cout << "[g: " << g_curr << ", x: " << x_curr << ", y: " << y_curr << " ]\n";
			}

			// Expand related nodes 
			else{
				cout << "Exploring current node: " << endl;
				print1DVector(currNode);

				cout << "Explored Map: " << endl;
				print2DVector(closedList);
				
				for(auto move : planner.movements){
					x_next = x_curr + move[0];
					y_next = y_curr + move[1];

					// Check if valid 
					if(isValid(x_next, y_next, closedList)){
						
						g_next = g_curr + planner.cost;
						openNode = {g_next, x_next, y_next};

						// Check if already opened 
						if(!isOpen(openNode, openList)){
							cout << "Adding node: " << endl;
							openList.push_back(openNode);
							print1DVector(openNode);
						}

						// If already open skip
						else{
							cout << "Node already open, skipping: " << endl;
							print1DVector(openNode);
						}
					}

					else{
						cout << "Invalid node:" << endl;
						vec1d_int invalid = {-1, x_next, y_next};
						print1DVector(invalid);
					}			
				}
				
				cout << "Closing current node:" << endl;
				// Mark current explored node closed 
				closedList[x_curr][y_curr] = 8;
				print1DVector(currNode);
				cout << "----------------------------" << endl << endl;



			}
		}
	}
}


void sanityCheck();

int main()
{
	// sanityCheck();
	Map map;
	Planner planner;
	bool verbose = false;
	cout << "Verbose search?: ";
	cin >> verbose; 
	search(map, planner, verbose);
    return 0;
}

void sanityCheck(){
    Map map;
    Planner planner;

    // Sanity check
    cout << "Map:" << endl;
    print2DVector(map.grid);
    cout << endl;
    cout << "Start: " 
    	 << planner.start[0] << ", " 
    	 << planner.start[1] << endl; 
    cout << "Goal: " 
    	 << planner.goal[0] << ", " 
    	 << planner.goal[1] << endl;
    cout << "Cost: " << planner.cost << endl;
    cout << "Robot Movements: " 
    	 << planner.movements_arrows[0] 
    	 << ", " << planner.movements_arrows[1] 
    	 << ", " << planner.movements_arrows[2] 
    	 << ", " << planner.movements_arrows[3] 
    	 << endl;
    cout << "Delta:" << endl;
    print2DVector(planner.movements);

}
