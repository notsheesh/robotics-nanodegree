#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
using namespace std;

#define HEIGHT 5  			// row
#define WIDTH 6   			// col
#define START_X 0 			// row
#define START_Y 0 			// col
#define GOAL_X HEIGHT- 1    // row
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


void sanityCheck();
void describe(string, bool, vec1d_int, vec2d_int);

// Print function 
template <typename T>
void print2DVector(T Vec){
	for(int i = 0; i < Vec.size(); i++){
		for(int j = 0; j < Vec[0].size(); j++){
			if(Vec[i][j] >= 10 || Vec[i][j] < 0)
				cout << Vec[i][j] << "  ";
			else
				cout << " " << Vec[i][j] << "  ";
		}
		cout << endl;
	}
}

// Print function 
template <typename T>
void print1DVector(T Vec){
	cout << "[ ";
	for(auto i : Vec)
		cout << i << " ";
	cout << "]\n";
}

// Check if node == goal 
bool isGoal(int x, int y){
	return (x == GOAL_X) && (y == GOAL_Y);
}

// Check if node == valid 
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

vec2d_int makeExpansionList(vec2d_int map){
	for(int i = 0; i < map.size(); i++){
		for(int j = 0; j < map[0].size(); j++){
			// if(map[i][j] == 1){
				map[i][j] = -1;
			// }
		}
	}
	return map; 
}

// Search function 
void search(Map map, Planner planner, bool verbose){
	auto closedList = map.grid; 
	auto expansionList = makeExpansionList(map.grid);
	int x = planner.start[0];
	int y = planner.start[1];
	int g = 0; // cost of edge
	int x_curr, y_curr, g_curr;
	int x_next, y_next, g_next;
	int expand = 0;

	vec1d_int openNode = {g, x, y}; 
	vec2d_int openList;

	// dummy node and list
	vec1d_int dummyNode;
	vec2d_int dummyList;
	// Add the first open node-> start 
	openList.push_back(openNode);

	describe("start", verbose, openNode, dummyList);

	bool isFound = false;
	bool isOver = false;
	int count = 0;
	while(!isFound && !isOver){

		if(verbose){
			cout << "Iteration: " << ++count << endl;
			cout << "==================================" << endl;
		}

		describe("openList", verbose, dummyNode, openList);

		// Check if any openNodes ? explore : exit
		if(openList.size() == 0){
			isOver = true;
			cout << "Failed to reach goal :/\n"; 
		}

		/*
			Expansion strategy 
			------------------------------------
			Sort the openList prioritizing g
			*Pick the node with lowest g  
				Check if goal? exit : continue
				Expand for related nodes (children)
				Add valid nodes to open list 
				Go to * 
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

			// Mark 
			expansionList[x_curr][y_curr] = expand++;
			describe("expand", verbose, dummyNode, expansionList);
			// Check if newNode == Goal node 
			if(isGoal(x_curr, y_curr)){
				isFound = true;
				cout << "Goal node found!\n";
				cout << "[g: " << g_curr << ", x: " << x_curr << ", y: " << y_curr << " ]\n";
			}

			// Expand related nodes 
			else{

				describe("map", verbose, dummyNode, closedList);

				
				for(auto move : planner.movements){
					x_next = x_curr + move[0];
					y_next = y_curr + move[1];

					// Check if valid 
					if(isValid(x_next, y_next, closedList)){
						
						g_next = g_curr + planner.cost;
						openNode = {g_next, x_next, y_next};

						// Check if already opened 
						if(!isOpen(openNode, openList)){
							openList.push_back(openNode);
							describe("add", verbose, openNode, openList);
						}
						// If already open skip
						else describe("open", verbose, openNode, openList);
					}

					else{
						dummyNode = {-1, x_next, y_next};
						describe("invalid", verbose, dummyNode, dummyList);
					}			
				}
				
				// Mark current explored node closed 
				closedList[x_curr][y_curr] = 8;
				describe("close", verbose, currNode, dummyList);
			}
		}
	} 
}

int main()
{
	Map map; 
	Planner planner;
	bool verbose = false;
	cout << "Verbose search?: ";
	cin >> verbose; 
	search(map, planner, verbose);
	if(verbose) cout << "\n[-1] Nodes were never expanded" << endl;
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

void describe(string action, bool verbose, vec1d_int node, vec2d_int list){
	if(!verbose){return;}
	if(action == "isOpen"){
		cout << "Node already open. Skipping: " << endl;
		print1DVector(node);
	}

	if(action == "invalid"){
		cout << "Invalid Node:         ";
		print1DVector(node);
	}

	if(action == "close"){
		cout << "Closing Current Node: ";
		print1DVector(node);	
		cout << "\n==================================" << endl;
	}

	if(action == "map"){
		cout << "Current Node: ";
		print1DVector(node);
		cout << "Explored Map: " << endl;
		print2DVector(list);
		cout << endl;
	}

	if(action == "expand"){
		cout << "Expansion List: " << endl;
		print2DVector(list);
		cout << endl;
	}

	if(action == "openList"){
		cout << "Open list: " << endl;
		print2DVector(list);
		cout << "----------------------------------" << endl;

	}
	if(action == "start"){
		cout << "Start Node: ";
		print1DVector(node);
	}

	if(action == "start"){
		cout << "Goal Node: "; 
		vec1d_int goal = {-1, GOAL_X, GOAL_Y};
		print1DVector(goal);
	}
	if(action == "add"){
		cout << "Adding Node:\t      ";
		print1DVector(node);
	}
}
