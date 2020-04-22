/* Breadth-search implementation on simple path planning problem.

Map of environment, start = *, end = x 
* 1 0 0 0 0
0 1 0 0 0 0
0 1 0 0 0 0
0 1 0 0 0 0
0 0 0 1 1 x

*/

#include <iostream>
#include <string.h>
#include <vector>
#include <algorithm>

using namespace std;

// Map class
class Map {
public:
    const static int width = 6;
    const static int height = 5;
    vector<vector<int> > grid = {
        { 0, 1, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0 },
        { 0, 0, 0, 1, 1, 0 }
    };
};

// Planner class
class Planner : Map {
public:
    int start[2] = {0,0}; // top left corner of map
    int goal[2] = {4,5};  // bottom right corner of map
    int cost = 1;

    string movements_arrows[4] = { "^", "<", "v", ">" };

    vector<vector<int> > movements{
        { -1, 0 },
        { 0, -1 },
        { 1, 0 },
        { 0, 1 }
    };
};

// Print 2D vectors
template <typename T>
void print2DVector(T Vec)
{
    for (int i = 0; i < Vec.size(); ++i) {
        for (int j = 0; j < Vec[0].size(); ++j) {
            cout << Vec[i][j] << ' ';
        }
        cout << endl;
    }
    cout << endl;
}

// search function that generates the expansions and performs breadth-first search
void search(Map map, Planner planner)
{
    // print map
    cout << "Map: " << endl;
    print2DVector(map.grid);

    // created closed 2 array filled with 0s and first element 1
    // closed double arrays stores 1 if cell has been explored and 0 if not.
    vector<vector<int>> closed(map.height, vector<int>(map.width));
    closed[planner.start[0]][planner.start[1]] = 1;

    // create expand array filled with -1
    // expand array stores value of order in which cell was explored, unexplored cells have value of -1
    vector<vector<int>> expand(map.height, vector<int>(map.width, -1));

    // create action array filled with -1
    // action array stores which movement was used at a cell, unused cells have value of -1
    vector<vector<int>> action(map.height, vector<int>(map.width, -1));

    // define triplet values
    int x = planner.start[0]; // map row value
    int y = planner.start[1]; // map column value
    int c = 0;                // cost of expanding toward cell -> what bfs tries to minimize

    // store expansions
    vector<vector<int>> open;
    open.push_back({c,x,y});

    // flags and counters
    bool found = false;     // did I find goal?
    bool resign = false;    // is problem unsolvable?
    int count = 0;

    int x2;
    int y2;

    // while still searching and still solvable
    while (!found && !resign) {
        // resign if no values in open list and can't expand anymore
        if (open.size() == 0) {
            resign = true;
            cout << "Failed to reach goal. Problem unsolvable." << endl;
        }
        // else keep expanding
        else {
            // remove triplets from the open list
            sort(open.begin(), open.end());
            reverse(open.begin(), open.end());
            // store popped value in next;
            vector<int> next;
            next = open.back();
            open.pop_back();

            c = next[0];
            x = next[1];
            y = next[2];

            // fill expand vector with count
            expand[x][y] = count;
            count += 1;

            // check if we reached the goal
            if (x == planner.goal[0] && y == planner.goal[1]) {
                found = true;
                cout << "Final Pos: [" << x << ", " << y << "] | Final Cost: " << c << endl << endl;
            }
            // else expand new elements
            else {
                // iterate through all possible movements
                for (int i=0; i < planner.movements.size(); i++) {
                    // get next position with that movement
                    x2 = x + planner.movements[i][0];
                    y2 = y + planner.movements[i][1];
                    // make sure new position is still within map limits
                    if (x2 >= 0 && x2 < map.grid.size() && y2 >= 0 && y2 < map.grid[0].size()) {
                        // check if we have explored that cell already and it is not an obstacle
                        if (closed[x2][y2] == 0 && map.grid[x2][y2] == 0) {
                            int c2 =  c + planner.cost;
                            open.push_back({c2,x2,y2}); 
                            closed[x2][y2] = 1; 
                            action[x2][y2] = i;
                        }
                    }
                }
            }
        }
    }
    // print expansion list
    cout << "Order of cell exploration:" << endl;
    print2DVector(expand);

    // fill in policy string array with movements (^,<,v,>) corresponding to value in action array (0,1,2,3)
    vector<vector<string>> policy(map.height, vector<string>(map.width, "-"));

    // start from end and work backward
    x = planner.goal[0];
    y = planner.goal[1];
    policy[x][y] = '*'; // goal cell

    while (x != planner.start[0] or y != planner.start[1]) {
        x2 = x - planner.movements[action[x][y]][0];
        y2 = y - planner.movements[action[x][y]][1];
        policy[x2][y2] = planner.movements_arrows[action[x][y]];
        x = x2;
        y = y2;
    }

    // print the path with arrows
    cout << "Robot final path to goal:" << endl;
    print2DVector(policy);
}

int main()
{
    // Instantiate map and planner objects
    Map map;
    Planner planner;

    // Search for the expansions
    search(map, planner);

    return 0;
}
