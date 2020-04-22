/* A* search implementation on simple path planning problem.

In this case the map of the environment comes from map.txt.

Map = 300x150
start = {230,145}
goal  = {60,50}

*/

#include <iostream>
#include <math.h>
#include <vector>
#include <iterator>
#include <fstream>
#include "src/matplotlibcpp.h" //Graph Library

using namespace std;
namespace plt = matplotlibcpp;

// Map class
class Map {
public:
    const static int width = 150;
    const static int height = 300;
    vector<vector<double>> map = GetMap();                  // get map from map.txt
    vector<vector<int>> grid = MaptoGrid();                 // convert it to a grid
    vector<vector<int>> heuristic = GenerateHeuristic();    // generate heuristic as distance from goal

private:
    // read file map.txt and get map
    vector<vector<double>> GetMap()
    {
        vector<vector<double>> map(height, vector<double>(width));
        ifstream mapFile;
        mapFile.open("map.txt");

        while (!mapFile.eof()) {
            for (int i = 0; i < height; i++) {
                for (int j = 0; j < width; j++) {
                    mapFile >> map[i][j];
                }
            }
        }
        return map;
    }

    // convert map double value to binary 1's and 0's
    vector<vector<int>> MaptoGrid() {
        vector<vector<int>> grid(height, vector<int>(width));
        for (int x=0; x < height; x++) {
            for (int y=0; y < width; y++) {
                if (map[x][y] == 0)     // unknown state
                    grid[x][y] = 1;
                else if (map[x][y] > 0) // occupied state
                    grid[x][y] = 1;
                else                    // free state
                    grid[x][y] = 0;
            }
        }
        return grid;
    }

    // generate Manhattan Heuristic Vector
    vector<vector<int>> GenerateHeuristic()
    {
        vector<vector<int>> heuristic(height, vector<int>(width));
        int goal[2] = {60,50};
        for (int i = 0; i < heuristic.size(); i++) {
            for (int j = 0; j < heuristic[0].size(); j++) {
                int xd = goal[0] - i;
                int yd = goal[1] - j;
                // Manhattan Distance
                   int d = abs(xd) + abs(yd);
                // Euclidian Distance
                // double d = sqrt(xd * xd + yd * yd);
                // Chebyshev distance
                // int d = max(abs(xd), abs(yd));
                heuristic[i][j] = d;
            }
        }
        return heuristic;
    }
};

// Planner class
class Planner : Map {
public:
    int start[2] = {230,145};   
    int goal[2] = {60,50};
    int cost = 1;

    string movements_arrows[4] = { "^", "<", "v", ">" };

    vector<vector<int> > movements{
        { -1, 0 },
        { 0, -1 },
        { 1, 0 },
        { 0, 1 }
    };

    vector<vector<int> > path;
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

// search function that generates the expansions and performs A* search
Planner search(Map map, Planner planner)
{
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
    int x = planner.start[0];           // map row value
    int y = planner.start[1];           // map column value
    int c = 0;                          // cost of expanding toward cell
    int f = c + map.heuristic[x][y];    // combined heuristic and cost -> what A* tries to minimize

    // store expansions
    vector<vector<int>> open;
    open.push_back({f,c,x,y});

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
            // remove quadruplets from the open list
            sort(open.begin(), open.end());
            reverse(open.begin(), open.end());
            // store popped value in next;
            vector<int> next;
            next = open.back();
            open.pop_back();

            c = next[1];
            x = next[2];
            y = next[3];

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
                            f = c2 + map.heuristic[x2][y2];
                            open.push_back({f,c2,x2,y2}); 
                            closed[x2][y2] = 1; 
                            action[x2][y2] = i;
                        }
                    }
                }
            }
        }
    }

    // fill in policy string array with movements (^,<,v,>) corresponding to value in action array (0,1,2,3)
    vector<vector<string>> policy(map.height, vector<string>(map.width, "-"));

    // start from end and work backward
    x = planner.goal[0];
    y = planner.goal[1];
    policy[x][y] = '*'; // goal cell

    while (x != planner.start[0] or y != planner.start[1]) {
        x2 = x - planner.movements[action[x][y]][0];
        y2 = y - planner.movements[action[x][y]][1];
        // Store the  Path in a vector
        planner.path.push_back({ x2, y2 });
        policy[x2][y2] = planner.movements_arrows[action[x][y]];
        x = x2;
        y = y2;
    }
    
    return planner;
}

// visualize result using Matplotlib
void visualization(Map map, Planner planner)
{
    // graph format
    plt::title("Path");
    plt::xlim(0, map.height);
    plt::ylim(0, map.width);

    // Draw every grid of the map:
    for (double x = 0; x < map.height; x++) {
        cout << "Remaining Rows= " << map.height - x << endl;
        for (double y = 0; y < map.width; y++) {
            if (map.map[x][y] == 0) { //Green unkown state
                plt::plot({ x }, { y }, "g.");
            }
            else if (map.map[x][y] > 0) { //Black occupied state
                plt::plot({ x }, { y }, "k.");
            }
            else { //Red free state
                plt::plot({ x }, { y }, "r.");
            }
        }
    }

    // Plot the robot path
    for (int i = 0; i < planner.path.size(); i++) {
        plt::plot({ (double)planner.path[i][0] }, { (double)planner.path[i][1] }, "b.");
    }

    //Save the image and close the plot
    plt::save("path.png");
    plt::clf();
}

int main()
{
    // Instantiate map and planner objects
    Map map;
    Planner planner;

    // Search for the expansions
    planner = search(map, planner);

    // plot the map and the path generated
    visualization(map, planner);

    return 0;
}
