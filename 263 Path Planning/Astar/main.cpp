#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <math.h>
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
	vec2d_int grid={
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

void describe(string action, bool verbose, vec1d_int node, vec2d_int list);

// Check if goal
bool isGoal(int x, int y, Planner planner){
	return (
		x == planner.goal[0] && 
		y == planner.goal[1]
		);
}

// Check if valid
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
		isX = test[2] == exist[2] ? true : false;
		isY = test[3] == exist[3] ? true : false;
		bool isOpen = isX && isY;
		if(isOpen){
			// It is
			return isOpen;
		}
	}
	// Nope, go on and add it! 
	return false;
}

// Support functions for heuristic
int getManhattan(int x1, int y1, Planner planner){
	int delx = x1 - planner.goal[0];
	int dely = y1 - planner.goal[1];
	return abs(delx) + abs(dely);
}

int getEuclidean(int x1, int y1, Planner planner){
	int delx = x1 - planner.goal[0];
	int dely = y1 - planner.goal[1];
	return pow(pow(delx, 2) + pow(dely, 2), 0.5);
}

int getChebyshev(int x1, int y1, Planner planner){
	int delx = abs(x1 - planner.goal[0]);
	int dely = abs(y1 - planner.goal[1]);
	return max(delx, dely);
}

// Make heuristic 2D vector 
vec2d_int makeHeuristic(Map map, Planner planner, int distance = 0){
	vec2d_int h_map(map.mapHeight, vec1d_int(map.mapWidth, -1));
	for (int i = 0; i < h_map.size(); ++i)
	{
		for(int j = 0; j < h_map[i].size(); j++){
			switch(distance){
				case 0:
					h_map[i][j] = getManhattan(i, j, planner); break;
				case 1: 
					h_map[i][j] = getEuclidean(i, j, planner); break;
				case 2:
					h_map[i][j] = getChebyshev(i, j, planner); break;
				default:
					cout << "Specify a valid heuristic type" << endl; 
					return h_map;
			}
		}
	}
	return h_map;
}

// Given goal & action vector
vec2d_char getPolicy(vec2d_int actions, Planner planner){
	vec2d_char policy(actions.size(), vec1d_char(actions[0].size(), '-'));
	// return policy;
	int check = 0;

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

// Main search function 
void search(Map map, Planner planner, bool verbose){

	// Initialize variables and vectors 
	int x_curr, y_curr, x_next, y_next, g_curr, g_next, f_curr, f_next;
	vec2d_int openList; vec2d_int closedList = map.grid;
	vec2d_int h_map = makeHeuristic(map, planner, 0);
	vec2d_int expansionList(map.mapHeight, vector<int>(map.mapWidth, -1));
	vec1d_int dummyNode; vec2d_int dummyList; int expand = 0;
	vec2d_int actions(map.mapHeight, vector<int>(map.mapWidth, -1));


	// Get start node 
	x_curr = planner.start[0];
	y_curr = planner.start[1];
	g_curr = 0;
	f_curr = h_map[x_curr][y_curr] + g_curr;
	vec1d_int openNode = {f_curr, g_curr, x_curr, y_curr};

	// Add to open list 
	openList.push_back(openNode);
	describe("start", verbose, openNode, dummyList);

	describe("heuristic", verbose, dummyNode, h_map);

	bool isFound = false; bool isOver = false; 
	int count = 0;

	// Start exploration using the openList 
	while(!isFound && !isOver){
		++count;
		if(verbose){
			cout << "\n=====================================" << endl;
			cout << "Iteration: " << count << endl;
			cout << "=====================================" << endl;
		}

		// Check if dead end 
		if(openList.size() == 0){

			isOver = true;
			cout << "Failed to reach goal :/" << endl;

		}

		// Explore unempty openList 
		else{
	
			// Arrange prioritizing f =  g + h 
			sort(openList.begin(), openList.end());
			reverse(openList.begin(), openList.end());
			describe("openList", verbose, dummyNode, openList);

			// Get cheapest node (f)
			vec1d_int currNode = openList.back();

			// Remove from open list
			openList.pop_back();

			x_curr = currNode[2];
			y_curr = currNode[3];
			g_curr = currNode[1];
			f_curr = h_map[x_curr][y_curr] + g_curr;

			expansionList[x_curr][y_curr] = expand++;

			// Mark exploration footprint 
			describe("map", verbose, currNode, closedList);
			describe("expand", verbose, dummyNode, expansionList);

			// Check if currNode == goal 
			if(isGoal(x_curr, y_curr, planner)){
				isFound = true;
				cout << "Goal node found!" << endl;
				cout << "[f: " 	<< f_curr
					 << ", g: " << g_curr
					 << ", x: " << x_curr
					 << ", y: " << y_curr << endl; 
			}

			
			// if not Goal -> expand related nodes
			else{

				// Try all adjacent nodes
				for (int i = 0; i < planner.movements.size(); ++i)
				{	
					// Move 
					x_next = x_curr + planner.movements[i][0];
					y_next = y_curr + planner.movements[i][1];

					// Check if valid
					if(isValid(x_next, y_next, closedList)){

						g_next = g_curr + planner.cost;
						f_next = h_map[x_next][y_next] + g_next;

						// Make new open node
						openNode = {f_next, g_next, x_next, y_next};

						// Check if already open 
						if(!isOpen(openNode, openList)){
							
							// Store action that lead you to [x_next][y_next]
							actions[x_next][y_next] = i;

							// Add valid node to openList 
							openList.push_back(openNode);
							describe("add", verbose, openNode, openList);
						}

						// If already open skip
						else{
							cout << "Node already open" << endl;
							describe("open", verbose, openNode, openList);
						}}

					// If not valid -> try next move command
					else{
						dummyNode = {-1, -1, x_next, y_next};
						describe("invalid", verbose, dummyNode, dummyList);

					}}}

			// Close current explored node 
			closedList[x_curr][y_curr] = 8;
			describe("close", verbose, currNode, dummyList);
		}}

	// Print final solution 
	if(verbose){
		cout << "[-1] Nodes were never expanded" << endl << endl;
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
	cout << "Number of iterations: " << count << endl;
}

int main(int argc, char const *argv[])
{
	Map map; Planner planner;
	bool verbose = false;
	cout << "Verbose search?: "; cin >> verbose;
	search(map, planner, verbose);
	return 0;
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
	}

	if(action == "map"){
		cout << "Explored Map: " << endl;
		print2DVector(list);
		cout << endl;
		cout << "Current Node: ";
		print1DVector(node);
		cout << endl;

	}

	if(action == "expand"){
		cout << "Expansion footprint: " << endl;
		print2DVector(list);
		cout << endl;
	}

	if(action == "openList"){
		cout << "Open list: " << endl;
		print2DVector(list);
		cout << "-------------------------------------" << endl;

	}
	if(action == "start"){
		cout << "\nStart Node: ";
		print1DVector(node);
	}

	if(action == "start"){
		cout << "Goal Node: "; 
		vec1d_int goal = {-1, -1, GOAL_X, GOAL_Y};
		print1DVector(goal);
	}
	if(action == "add"){
		cout << "Adding Node:\t      ";
		print1DVector(node);
	}
	if(action == "heuristic"){
		cout << "\nHeuristic function: " << endl;
		print2DVector(list);
	}
}
