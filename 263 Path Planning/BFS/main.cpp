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
typedef vector<vector<char>> vec2d_char;
typedef vector<char> vec1d_char;

class Map{
	public:
	const static int mapWidth = WIDTH;
	const static int mapHeight = HEIGHT;
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
	int goal[2] = {GOAL_X, GOAL_Y};
	int cost = 1;
	char movements_arrows[4] = {'^', '>', 'v', '<'};
	vec2d_int movements = {
		{-1, 0},
		{ 0, 1},
		{+1, 0},
		{ 0, -1}};
};


void sanityCheck();
void describe(string, bool, vec1d_int, vec2d_int);

// Print function 
void print2DVector(vec2d_int &Vec){
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

// Overloaded for policy grid
void print2DVector(vec2d_char &Vec){
	for(int i = 0; i < Vec.size(); i++){
		for(int j = 0; j < Vec[i].size(); j++){
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

// Given goal & action vector
vec2d_char getPolicy(vec2d_int actions, Planner planner){
	vec2d_char policy(actions.size(), vec1d_char(actions[0].size(), '-'));

	// Travel backwards 
	int to_x = planner.goal[0];
	int to_y = planner.goal[1];
	int from_x, from_y;
	
	while(!(to_x == planner.start[0] && to_y == planner.start[1])){
		// Take a step back 
		// The action that led to current map loc 
		int action = actions[to_x][to_y]; 
		from_x = to_x - planner.movements[action][0];
		from_y = to_y - planner.movements[action][1];

		// Forward policy 
		policy[from_x][from_y] = planner.movements_arrows[action];

		// Repeat
		to_x = from_x;
		to_y = from_y;
	}

	// Mark start & goal
	policy[planner.goal[0]][planner.goal[1]] 	= 'g';
	policy[planner.start[0]][planner.start[1]] 	= 's';

	return policy;
}

// Search function 
void search(Map map, Planner planner, bool verbose){

	// home node 
	int x = planner.start[0];
	int y = planner.start[1];
	int g = 0; // cost of edge
	vec1d_int openNode = {g, x, y}; 

	// x_t
	int x_curr, y_curr, g_curr;
	// x_t+1 = x_t + u_t+1
	int x_next, y_next, g_next;

	// to be explored
	vec2d_int openList;

	// to mark explored nodes - 8 
	auto closedList = map.grid; 

	// to mark expanded nodes !(-1)
	vec2d_int expansionList(map.mapHeight, vector<int>(map.mapWidth, -1));
	int expand = 0;

	// dummy node and list
	vec1d_int dummyNode;
	vec2d_int dummyList;

	// Add the first open node-> start 
	openList.push_back(openNode);

	describe("start", verbose, openNode, dummyList);

	bool isFound = false;
	bool isOver = false;
	int count = 0; // iteration 

	// For solution - actions and policy vector 
	vec2d_int actions(map.mapHeight, vector<int>(map.mapWidth, -1));

	while(!isFound && !isOver){
		if(verbose){
			cout << "Iteration: " << count++ << endl;
			cout << "==================================" << endl;
		}

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
			
			// Arrange nodes wrt cost of traversal
			sort(openList.begin(), openList.end());
			reverse(openList.begin(), openList.end());
			describe("openList", verbose, dummyNode, openList);

			// Get the cheapest node 
			vec1d_int currNode = openList.back();
			
			// Remove it from the open list
			openList.pop_back();

			// Explore current node 
			g_curr = currNode[0];
			x_curr = currNode[1];
			y_curr = currNode[2];

			// Mark step count on the current node
			expansionList[x_curr][y_curr] = expand++;
			describe("expand", verbose, dummyNode, expansionList);

			// Check if newNode == Goal node 
			if(isGoal(x_curr, y_curr)){
				isFound = true;
				cout << "Goal node found!\n";
				cout << "[g: " << g_curr << ", x: " << x_curr << ", y: " << y_curr << " ]\n";
			}

			// if not Goal expand related nodes 
			else{

				describe("map", verbose, currNode, closedList);
				for(int i = 0; i < planner.movements.size(); i++){
					x_next = x_curr + planner.movements[i][0];
					y_next = y_curr + planner.movements[i][1];

					// Check if valid 
					if(isValid(x_next, y_next, closedList)){
						g_next = g_curr + planner.cost;
						openNode = {g_next, x_next, y_next};

						// Check if already opened 
						if(!isOpen(openNode, openList)){

							// Action that lead you to map[x_next][y_next]
							actions[x_next][y_next] = i;
							openList.push_back(openNode);
							describe("add", verbose, openNode, openList);
						}

						// If already open, then skip
						else describe("open", verbose, openNode, openList);
					}

					// If not valid
					else{

						dummyNode = {-1, x_next, y_next};
						describe("invalid", verbose, dummyNode, dummyList);
					}			
				}
			}

			// Mark current explored node closed 
			closedList[x_curr][y_curr] = 8;
			describe("close", verbose, currNode, dummyList);
		}
	}

	// Print final solution 
	if(verbose){
		cout << "[-1] Nodes were never expanded" << endl;
		cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
		cout << "Action Vector: " << endl;
		cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
		print2DVector(actions); 
	}
	
	cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
	auto policy = getPolicy(actions, planner);
	cout << "Policy Vector: " << endl;
	cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
	print2DVector(policy);
}

int main()
{
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
