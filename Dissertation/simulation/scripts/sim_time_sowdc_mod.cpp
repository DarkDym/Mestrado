#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <std_msgs/Bool.h>
#include <chrono>
#include <sstream>
#include <string>
#include <fstream>

//--------------------------ADICIONADO 03/01/23--------------------------------------
#include "nav_msgs/OccupancyGrid.h"
//-----------------------------------------------------------------------------------
//--------------------------ADICIONADO 17/01/23--------------------------------------
#include "gazebo_msgs/PerformanceMetrics.h"
#include "rosgraph_msgs/Clock.h"
//-----------------------------------------------------------------------------------
//--------------------------ADICIONADO 20/01/23--------------------------------------
#include "nav_msgs/Path.h"
//-----------------------------------------------------------------------------------
//--------------------------ADICIONADO 24/01/23--------------------------------------
// #include "a_star.hpp"
#include <bits/stdc++.h>
// #include "nav_msgs/OccupancyGrid.h"

#define ROW 4000
#define COL 4000
//-----------------------------------------------------------------------------------
//--------------------------ADICIONADO 26/01/23--------------------------------------
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
//-----------------------------------------------------------------------------------

using namespace std::chrono;
using namespace std;

//--------------------------ADICIONADO 24/01/23--------------------------------------
// Creating a shortcut for int, int pair type
typedef pair<int, int> Pair;

// Creating a shortcut for pair<int, pair<int, int>> type
typedef pair<double, pair<int, int> > pPair;

// A structure to hold the necessary parameters
struct cell {
	// Row and Column index of its parent
	// Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
	int parent_i, parent_j;
	// f = g + h
	double f, g, h;
};

float vertex1_x_,vertex1_y_,vertex2_x_,vertex2_y_;
//-----------------------------------------------------------------------------------

vector<tuple<float,float>> goals;
int index_ = 0;
bool time_started_ = false;
std::chrono::steady_clock::time_point start_time_;
std::chrono::steady_clock::time_point end_time_;
std::ofstream objects_map_file_;
std::ofstream fulllog_file_;
bool first_setup = false;
fstream objects_file_;
vector<tuple<int,float,float>> object_goals_;

//--------------------------ADICIONADO 03/01/23--------------------------------------
nav_msgs::OccupancyGrid grid_map_;
nav_msgs::OccupancyGrid lane_map_;
nav_msgs::OccupancyGrid clean_map_;

bool setup_map = true;
int original_map_[4000][4000];
int modified_map_[4000][4000];

fstream ctldraw_file_;
bool enable_ctldraw_ = false;
bool disable_all_ctldraw_ = false;
bool already_drawed_ = false;
//--------------------------ADICIONADO 06/01/23--------------------------------------
bool enable_patrol_ = true;
//-----------------------------------------------------------------------------------

//--------------------------ADICIONADO 17/01/23--------------------------------------
int gazebo_secs_ = 0;
//-----------------------------------------------------------------------------------

//--------------------------ADICIONADO 20/01/23--------------------------------------
nav_msgs::Path global_path_;
nav_msgs::Path test_path_;
geometry_msgs::Pose pose_path_;
//-----------------------------------------------------------------------------------

//--------------------------ADICIONADO 26/01/23--------------------------------------
float AMCL_POSE_[3] = {0,0,0};
bool isPathInBlock_ = false;
ros::Publisher astar_path_mod_pub;
ros::Publisher astar_path_pub;
int vertexes_[4] = {0,0,0,0};
//-----------------------------------------------------------------------------------

//--------------------------ADICIONADO 30/01/23--------------------------------------
vector<tuple<int,int,int,int,float,float,float,float,int>> block_vertex_;
std::ofstream calc_file_;
//-----------------------------------------------------------------------------------

//--------------------------ADICIONADO 24/01/23--------------------------------------
int grid_a[4000][4000];
bool closedList[4000][4000];
cell cellDetails[4000][4000];
bool astar_mod_ = false;


std::tuple<float,float> cell2odom(int cell_value_x, int cell_value_y){
    float x = (cell_value_x + grid_map_.info.origin.position.x/grid_map_.info.resolution) * grid_map_.info.resolution;
    float y = (cell_value_y + grid_map_.info.origin.position.y/grid_map_.info.resolution) * grid_map_.info.resolution;
    return std::make_tuple(x,y);
}

void block_path_verification_y(float py, int block_index){
    if (!isPathInBlock_) {
        // cout << "###################DENTRO DO BLOCK_PATH_VERIFICATION###################" << endl;
        // cout << "VERTEX1_Y: " << vertex1_y_ << " | VERTEX2_Y: " << vertex2_y_ << endl;

        // if (vertex1_y_ > vertex2_y_) {
        //     if ((py - vertex2_y_) >= 0 && (py - vertex2_y_) <= 0.2) {
        //         cout << "VERTEX1_Y: " << vertex1_y_ << " | VERTEX2_Y: " << vertex2_y_ << endl;
        //         cout << "PY: " << py << " ";
        //         cout << "VALOR DA SUBSTRACAO: " << (py - vertex2_y_) << endl;
        //         isPathInBlock_ = true;
        //     }
        // } else {
        //     if ((py - vertex1_y_) >= 0 && (py - vertex1_y_) <= 0.2) {
        //         cout << "VERTEX1_Y: " << vertex1_y_ << " | VERTEX2_Y: " << vertex2_y_ << endl;
        //         cout << "PY: " << py << " ";
        //         cout << "VALOR DA SUBSTRACAO: " << (py - vertex1_y_) << endl;
        //         isPathInBlock_ = true;
        //     }
        // }

        //--------------------------ADICIONADO 30/01/23--------------------------------------
        int v1,v2,v3,v4,qnt_p;
        float ov1,ov2,ov3,ov4;
        tie(v1,v2,v3,v4,ov1,ov2,ov3,ov4,qnt_p) = block_vertex_[block_index];
        if (ov2 > ov4) {
            if ((py - ov4) >= 0 && (py - ov4) <= 0.2) {
                cout << "VERTEX1_Y: " << ov2 << " | VERTEX2_Y: " << ov4 << endl;
                cout << "PY: " << py << " ";
                cout << "VALOR DA SUBSTRACAO: " << (py - ov4) << endl;
                isPathInBlock_ = true;
                calc_file_ << "PATH GOES THROUGH BLOCKAGE!" << endl; 
            }
        } else {
            if ((py - ov2) >= 0 && (py - ov2) <= 0.2) {
                cout << "VERTEX1_Y: " << ov2 << " | VERTEX2_Y: " << ov4 << endl;
                cout << "PY: " << py << " ";
                cout << "VALOR DA SUBSTRACAO: " << (py - ov2) << endl;
                isPathInBlock_ = true;
                calc_file_ << "PATH GOES THROUGH BLOCKAGE!" << endl;
            }
        }
        //-----------------------------------------------------------------------------------
    }
}

// A Utility Function to check whether given cell (row, col)
// is a valid cell or not.
bool isValid(int row, int col){
    // cout << "VALIDATION OF CEL AND ROW VALUES" << endl;
	// Returns true if row number and column number
	// is in range
	return (row >= 0) && (row < 4000) && (col >= 0)
		&& (col < 4000);
}

// A Utility Function to check whether the given cell is
// blocked or not
tuple<bool,int> isUnBlocked(int row, int col){
    // cout << "VERIFICATION OF CELL" << endl;
	// Returns true if the cell is not blocked else false
	if (grid_a[row][col] == 0 || grid_a[row][col] == 48)
		return make_tuple(true,grid_a[row][col]);
	else
		return make_tuple(false,grid_a[row][col]);
}

// A Utility Function to check whether destination cell has
// been reached or not
bool isDestination(int row, int col, Pair dest){
	if (row == dest.first && col == dest.second)
		return true;
	else
		return false;
}

// A Utility Function to calculate the 'h' heuristics.
double calculateHValue(int row, int col, Pair dest){
	// Return using the distance formula
	return ((double)sqrt(
		(row - dest.first) * (row - dest.first)
		+ (col - dest.second) * (col - dest.second)));
}

// A Utility Function to trace the path from the source
// to destination
int tracePath(Pair dest, int block_index){
	printf("\nThe Path is ");
	int row = dest.first;
	int col = dest.second;

	stack<Pair> Path;

	while (!(cellDetails[row][col].parent_i == row
			&& cellDetails[row][col].parent_j == col)) {
		Path.push(make_pair(row, col));
		int temp_row = cellDetails[row][col].parent_i;
		int temp_col = cellDetails[row][col].parent_j;
		row = temp_row;
		col = temp_col;
	}

	Path.push(make_pair(row, col));
	geometry_msgs::PoseStamped ps;
    int tam_path = Path.size();
    while (!Path.empty()) {
		pair<int, int> p = Path.top();
		Path.pop();
        float px,py;
        tie(px,py) = cell2odom(p.second, p.first);
        // cout << "-> (" << px << "," << py << ") ";
        block_path_verification_y(py,block_index);
        pose_path_.position.x = px;
        pose_path_.position.y = py;
        ps.pose = pose_path_;
        ps.header.frame_id = "map";
        test_path_.poses.emplace_back(ps);
        test_path_.header.frame_id = "map";
		// printf("-> (%d,%d) ", p.first, p.second);
	}

	return tam_path;
}

// A Function to find the shortest path between
// a given source cell to a destination cell according
// to A* Search Algorithm
int aStarSearch_MOD(int srcx, int srcy, int gx, int gy, int block_index){
    
    cout << "INSIDE A STAR SEARCH!!!!!!!!!!!!!" << endl;
    if (!astar_mod_) {
        for (int x = 0; x < 4000; x++) {
            for (int y = 0; y < 4000; y++) {
                grid_a[x][y] = -1;
            }
        }

        for(int x = 0; x < 4000; x++){
            int multi = x * 4000;
            for(int y = 0; y < 4000; y++){
                int index = y + multi;
                grid_a[x][y] = grid_map_.data[index];
            }
        }
    }
    
    Pair src = make_pair(srcx,srcy);
    Pair dest = make_pair(gx,gy);

	// If the source is out of range
	if (isValid(src.first, src.second) == false) {
		printf("Source is invalid\n");
		return 0;
	}

	// If the destination is out of range
	if (isValid(dest.first, dest.second) == false) {
		printf("Destination is invalid\n");
		return 0;
	}

	// Either the source or the destination is blocked
    bool isUnBlocked_src_,isUnBlocked_dest_;
    int cell_value_;
    tie(isUnBlocked_src_,cell_value_) = isUnBlocked(src.first, src.second);
    tie(isUnBlocked_dest_,cell_value_) = isUnBlocked(dest.first, dest.second);
	if (isUnBlocked_src_ == false || isUnBlocked_dest_ == false) {
		printf("Source or the destination is blocked\n");
		return 0;
	}

	// If the destination cell is the same as source cell
	if (isDestination(src.first, src.second, dest) == true) {
		printf("We are already at the destination\n");
		return 0;
	}

	// Create a closed list and initialise it to false which
	// means that no cell has been included yet This closed
	// list is implemented as a boolean 2D array
	memset(closedList, false, sizeof(closedList));

	// Declare a 2D array of structure to hold the details
	// of that cell
	

	int i, j;

	for (i = 0; i < 4000; i++) {
		for (j = 0; j < 4000; j++) {
			cellDetails[i][j].f = FLT_MAX;
			cellDetails[i][j].g = FLT_MAX;
			cellDetails[i][j].h = FLT_MAX;
			cellDetails[i][j].parent_i = -1;
			cellDetails[i][j].parent_j = -1;
		}
	}

	// Initialising the parameters of the starting node
	i = src.first, j = src.second;
	cellDetails[i][j].f = 0.0;
	cellDetails[i][j].g = 0.0;
	cellDetails[i][j].h = 0.0;
	cellDetails[i][j].parent_i = i;
	cellDetails[i][j].parent_j = j;

	/*
	Create an open list having information as-
	<f, <i, j>>
	where f = g + h,
	and i, j are the row and column index of that cell
	Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
	This open list is implemented as a set of pair of
	pair.*/
	set<pPair> openList;

	// Put the starting cell on the open list and set its
	// 'f' as 0
	openList.insert(make_pair(0.0, make_pair(i, j)));

	// We set this boolean value as false as initially
	// the destination is not reached.
	bool foundDest = false;

	while (!openList.empty()) {
		pPair p = *openList.begin();

		// Remove this vertex from the open list
		openList.erase(openList.begin());

		// Add this vertex to the closed list
		i = p.second.first;
		j = p.second.second;
		closedList[i][j] = true;

		/*
		Generating all the 8 successor of this cell

			N.W N N.E
			\ | /
				\ | /
			W----Cell----E
				/ | \
				/ | \
			S.W S S.E

		Cell-->Popped Cell (i, j)
		N --> North	 (i-1, j)
		S --> South	 (i+1, j)
		E --> East	 (i, j+1)
		W --> West		 (i, j-1)
		N.E--> North-East (i-1, j+1)
		N.W--> North-West (i-1, j-1)
		S.E--> South-East (i+1, j+1)
		S.W--> South-West (i+1, j-1)*/

		// To store the 'g', 'h' and 'f' of the 8 successors
		double gNew, hNew, fNew;

		//----------- 1st Successor (North) ------------

		// Only process this cell if this is a valid one
		if (isValid(i - 1, j) == true) {
			// If the destination cell is the same as the
			// current successor
			if (isDestination(i - 1, j, dest) == true) {
				// Set the Parent of the destination cell
				cellDetails[i - 1][j].parent_i = i;
				cellDetails[i - 1][j].parent_j = j;
				printf("NORTH | The destination cell is found\n");
				int tam_path = tracePath(dest,block_index);
				foundDest = true;
				return tam_path;
			}
			// If the successor is already on the closed
			// list or if it is blocked, then ignore it.
			// Else do the following
			else if (closedList[i - 1][j] == false) {
                tie(isUnBlocked_src_,cell_value_) = isUnBlocked(i - 1, j);
                if (isUnBlocked_src_ == true) {
                    if (cell_value_ == 0) {
                        gNew = cellDetails[i][j].g + 1.0;
                    } else {
                        gNew = cellDetails[i][j].g + 2.0;
                    }
                    
                    hNew = calculateHValue(i - 1, j, dest);
                    fNew = gNew + hNew;

                    // If it isn’t on the open list, add it to
                    // the open list. Make the current square
                    // the parent of this square. Record the
                    // f, g, and h costs of the square cell
                    //			 OR
                    // If it is on the open list already, check
                    // to see if this path to that square is
                    // better, using 'f' cost as the measure.
                    if (cellDetails[i - 1][j].f == FLT_MAX
                        || cellDetails[i - 1][j].f > fNew) {
                        openList.insert(make_pair(
                            fNew, make_pair(i - 1, j)));

                        // Update the details of this cell
                        cellDetails[i - 1][j].f = fNew;
                        cellDetails[i - 1][j].g = gNew;
                        cellDetails[i - 1][j].h = hNew;
                        cellDetails[i - 1][j].parent_i = i;
                        cellDetails[i - 1][j].parent_j = j;
                    }
                }
			}
		}

		//----------- 2nd Successor (South) ------------

		// Only process this cell if this is a valid one
		if (isValid(i + 1, j) == true) {
			// If the destination cell is the same as the
			// current successor
			if (isDestination(i + 1, j, dest) == true) {
				// Set the Parent of the destination cell
				cellDetails[i + 1][j].parent_i = i;
				cellDetails[i + 1][j].parent_j = j;
				printf("SOUTH | The destination cell is found\n");
				int tam_path = tracePath(dest,block_index);
				foundDest = true;
				return tam_path;
			}
			// If the successor is already on the closed
			// list or if it is blocked, then ignore it.
			// Else do the following
			else if (closedList[i + 1][j] == false) {
                tie(isUnBlocked_src_,cell_value_) = isUnBlocked(i + 1, j);
                if (isUnBlocked_src_ == true) {
                    if (cell_value_ == 0) {
                        gNew = cellDetails[i][j].g + 1.0;
                    } else {
                        gNew = cellDetails[i][j].g + 2.0;
                    }
                    hNew = calculateHValue(i + 1, j, dest);
                    fNew = gNew + hNew;

                    // If it isn’t on the open list, add it to
                    // the open list. Make the current square
                    // the parent of this square. Record the
                    // f, g, and h costs of the square cell
                    //			 OR
                    // If it is on the open list already, check
                    // to see if this path to that square is
                    // better, using 'f' cost as the measure.
                    if (cellDetails[i + 1][j].f == FLT_MAX
                        || cellDetails[i + 1][j].f > fNew) {
                        openList.insert(make_pair(
                            fNew, make_pair(i + 1, j)));
                        // Update the details of this cell
                        cellDetails[i + 1][j].f = fNew;
                        cellDetails[i + 1][j].g = gNew;
                        cellDetails[i + 1][j].h = hNew;
                        cellDetails[i + 1][j].parent_i = i;
                        cellDetails[i + 1][j].parent_j = j;
                    }
                }
			}
		}

		//----------- 3rd Successor (East) ------------

		// Only process this cell if this is a valid one
		if (isValid(i, j + 1) == true) {
			// If the destination cell is the same as the
			// current successor
			if (isDestination(i, j + 1, dest) == true) {
				// Set the Parent of the destination cell
				cellDetails[i][j + 1].parent_i = i;
				cellDetails[i][j + 1].parent_j = j;
				printf("EAST | The destination cell is found\n");
				int tam_path = tracePath(dest,block_index);
				foundDest = true;
				return tam_path;
			}

			// If the successor is already on the closed
			// list or if it is blocked, then ignore it.
			// Else do the following
			else if (closedList[i][j + 1] == false) {
                tie(isUnBlocked_src_,cell_value_) = isUnBlocked(i, j + 1);
                if (isUnBlocked_src_ == true) {
                    if (cell_value_ == 0) {
                        gNew = cellDetails[i][j].g + 1.0;
                    } else {
                        gNew = cellDetails[i][j].g + 2.0;
                    }
                    hNew = calculateHValue(i, j + 1, dest);
                    fNew = gNew + hNew;

                    // If it isn’t on the open list, add it to
                    // the open list. Make the current square
                    // the parent of this square. Record the
                    // f, g, and h costs of the square cell
                    //			 OR
                    // If it is on the open list already, check
                    // to see if this path to that square is
                    // better, using 'f' cost as the measure.
                    if (cellDetails[i][j + 1].f == FLT_MAX
                        || cellDetails[i][j + 1].f > fNew) {
                        openList.insert(make_pair(
                            fNew, make_pair(i, j + 1)));

                        // Update the details of this cell
                        cellDetails[i][j + 1].f = fNew;
                        cellDetails[i][j + 1].g = gNew;
                        cellDetails[i][j + 1].h = hNew;
                        cellDetails[i][j + 1].parent_i = i;
                        cellDetails[i][j + 1].parent_j = j;
                    }
                }
			}
		}

		//----------- 4th Successor (West) ------------

		// Only process this cell if this is a valid one
		if (isValid(i, j - 1) == true) {
			// If the destination cell is the same as the
			// current successor
			if (isDestination(i, j - 1, dest) == true) {
				// Set the Parent of the destination cell
				cellDetails[i][j - 1].parent_i = i;
				cellDetails[i][j - 1].parent_j = j;
				printf("WEST | The destination cell is found\n");
				int tam_path = tracePath(dest,block_index);
				foundDest = true;
				return tam_path;
			}

			// If the successor is already on the closed
			// list or if it is blocked, then ignore it.
			// Else do the following
			else if (closedList[i][j - 1] == false) {
                tie(isUnBlocked_src_,cell_value_) = isUnBlocked(i, j - 1);
                if (isUnBlocked_src_ == true) {
                    if (cell_value_ == 0) {
                        gNew = cellDetails[i][j].g + 1.0;
                    } else {
                        gNew = cellDetails[i][j].g + 2.0;
                    }
                    hNew = calculateHValue(i, j - 1, dest);
                    fNew = gNew + hNew;

                    // If it isn’t on the open list, add it to
                    // the open list. Make the current square
                    // the parent of this square. Record the
                    // f, g, and h costs of the square cell
                    //			 OR
                    // If it is on the open list already, check
                    // to see if this path to that square is
                    // better, using 'f' cost as the measure.
                    if (cellDetails[i][j - 1].f == FLT_MAX
                        || cellDetails[i][j - 1].f > fNew) {
                        openList.insert(make_pair(
                            fNew, make_pair(i, j - 1)));

                        // Update the details of this cell
                        cellDetails[i][j - 1].f = fNew;
                        cellDetails[i][j - 1].g = gNew;
                        cellDetails[i][j - 1].h = hNew;
                        cellDetails[i][j - 1].parent_i = i;
                        cellDetails[i][j - 1].parent_j = j;
                    }
                }
			}
		}

		//----------- 5th Successor (North-East)
		//------------

		// Only process this cell if this is a valid one
		if (isValid(i - 1, j + 1) == true) {
			// If the destination cell is the same as the
			// current successor
			if (isDestination(i - 1, j + 1, dest) == true) {
				// Set the Parent of the destination cell
				cellDetails[i - 1][j + 1].parent_i = i;
				cellDetails[i - 1][j + 1].parent_j = j;
				printf("NORTH-EAST | The destination cell is found\n");
				int tam_path = tracePath(dest,block_index);
				foundDest = true;
				return tam_path;
			}

			// If the successor is already on the closed
			// list or if it is blocked, then ignore it.
			// Else do the following
			else if (closedList[i - 1][j + 1] == false) {
                tie(isUnBlocked_src_,cell_value_) = isUnBlocked(i - 1, j + 1);
                if (isUnBlocked_src_ == true) {
                    if (cell_value_ == 0) {
                        gNew = cellDetails[i][j].g + 1.414;
                    } else {
                        gNew = cellDetails[i][j].g + 2.414;
                    }
                    
                    hNew = calculateHValue(i - 1, j + 1, dest);
                    fNew = gNew + hNew;

                    // If it isn’t on the open list, add it to
                    // the open list. Make the current square
                    // the parent of this square. Record the
                    // f, g, and h costs of the square cell
                    //			 OR
                    // If it is on the open list already, check
                    // to see if this path to that square is
                    // better, using 'f' cost as the measure.
                    if (cellDetails[i - 1][j + 1].f == FLT_MAX
                        || cellDetails[i - 1][j + 1].f > fNew) {
                        openList.insert(make_pair(
                            fNew, make_pair(i - 1, j + 1)));

                        // Update the details of this cell
                        cellDetails[i - 1][j + 1].f = fNew;
                        cellDetails[i - 1][j + 1].g = gNew;
                        cellDetails[i - 1][j + 1].h = hNew;
                        cellDetails[i - 1][j + 1].parent_i = i;
                        cellDetails[i - 1][j + 1].parent_j = j;
                    }
                }
			}
		}

		//----------- 6th Successor (North-West)
		//------------

		// Only process this cell if this is a valid one
		if (isValid(i - 1, j - 1) == true) {
			// If the destination cell is the same as the
			// current successor
			if (isDestination(i - 1, j - 1, dest) == true) {
				// Set the Parent of the destination cell
				cellDetails[i - 1][j - 1].parent_i = i;
				cellDetails[i - 1][j - 1].parent_j = j;
				printf("NORTH-WEST | The destination cell is found\n");
				int tam_path = tracePath(dest,block_index);
				foundDest = true;
				return tam_path;
			}

			// If the successor is already on the closed
			// list or if it is blocked, then ignore it.
			// Else do the following
			else if (closedList[i - 1][j - 1] == false) {
                tie(isUnBlocked_src_,cell_value_) = isUnBlocked(i - 1, j - 1);
                if (isUnBlocked_src_ == true) {
                    if (cell_value_ == 0) {
                        gNew = cellDetails[i][j].g + 1.414;
                    } else {
                        gNew = cellDetails[i][j].g + 2.414;
                    }
                    hNew = calculateHValue(i - 1, j - 1, dest);
                    fNew = gNew + hNew;

                    // If it isn’t on the open list, add it to
                    // the open list. Make the current square
                    // the parent of this square. Record the
                    // f, g, and h costs of the square cell
                    //			 OR
                    // If it is on the open list already, check
                    // to see if this path to that square is
                    // better, using 'f' cost as the measure.
                    if (cellDetails[i - 1][j - 1].f == FLT_MAX
                        || cellDetails[i - 1][j - 1].f > fNew) {
                        openList.insert(make_pair(
                            fNew, make_pair(i - 1, j - 1)));
                        // Update the details of this cell
                        cellDetails[i - 1][j - 1].f = fNew;
                        cellDetails[i - 1][j - 1].g = gNew;
                        cellDetails[i - 1][j - 1].h = hNew;
                        cellDetails[i - 1][j - 1].parent_i = i;
                        cellDetails[i - 1][j - 1].parent_j = j;
                    }
                }
			}
		}

		//----------- 7th Successor (South-East)
		//------------

		// Only process this cell if this is a valid one
		if (isValid(i + 1, j + 1) == true) {
			// If the destination cell is the same as the
			// current successor
			if (isDestination(i + 1, j + 1, dest) == true) {
				// Set the Parent of the destination cell
				cellDetails[i + 1][j + 1].parent_i = i;
				cellDetails[i + 1][j + 1].parent_j = j;
				printf("SOUTH-EAST | The destination cell is found\n");
				int tam_path = tracePath(dest,block_index);
				foundDest = true;
				return tam_path;
			}

			// If the successor is already on the closed
			// list or if it is blocked, then ignore it.
			// Else do the following
			else if (closedList[i + 1][j + 1] == false) {
                tie(isUnBlocked_src_,cell_value_) = isUnBlocked(i + 1, j + 1);
                if (isUnBlocked_src_ == true) {
                    if (cell_value_ == 0) {
                        gNew = cellDetails[i][j].g + 1.414;
                    } else {
                        gNew = cellDetails[i][j].g + 2.414;
                    }
                    hNew = calculateHValue(i + 1, j + 1, dest);
                    fNew = gNew + hNew;

                    // If it isn’t on the open list, add it to
                    // the open list. Make the current square
                    // the parent of this square. Record the
                    // f, g, and h costs of the square cell
                    //			 OR
                    // If it is on the open list already, check
                    // to see if this path to that square is
                    // better, using 'f' cost as the measure.
                    if (cellDetails[i + 1][j + 1].f == FLT_MAX
                        || cellDetails[i + 1][j + 1].f > fNew) {
                        openList.insert(make_pair(
                            fNew, make_pair(i + 1, j + 1)));

                        // Update the details of this cell
                        cellDetails[i + 1][j + 1].f = fNew;
                        cellDetails[i + 1][j + 1].g = gNew;
                        cellDetails[i + 1][j + 1].h = hNew;
                        cellDetails[i + 1][j + 1].parent_i = i;
                        cellDetails[i + 1][j + 1].parent_j = j;
                    }
                }
			}
		}

		//----------- 8th Successor (South-West)
		//------------

		// Only process this cell if this is a valid one
		if (isValid(i + 1, j - 1) == true) {
			// If the destination cell is the same as the
			// current successor
			if (isDestination(i + 1, j - 1, dest) == true) {
				// Set the Parent of the destination cell
				cellDetails[i + 1][j - 1].parent_i = i;
				cellDetails[i + 1][j - 1].parent_j = j;
				printf("SOUTH-WEST | The destination cell is found\n");
				int tam_path = tracePath(dest,block_index);
				foundDest = true;
				return tam_path;
			}

			// If the successor is already on the closed
			// list or if it is blocked, then ignore it.
			// Else do the following
			else if (closedList[i + 1][j - 1] == false) {
                tie(isUnBlocked_src_,cell_value_) = isUnBlocked(i + 1, j - 1);
                if (isUnBlocked_src_ == true) {
                    if (cell_value_ == 0) {
                        gNew = cellDetails[i][j].g + 1.414;
                    } else {
                        gNew = cellDetails[i][j].g + 2.414;
                    }
                    hNew = calculateHValue(i + 1, j - 1, dest);
                    fNew = gNew + hNew;

                    // If it isn’t on the open list, add it to
                    // the open list. Make the current square
                    // the parent of this square. Record the
                    // f, g, and h costs of the square cell
                    //			 OR
                    // If it is on the open list already, check
                    // to see if this path to that square is
                    // better, using 'f' cost as the measure.
                    if (cellDetails[i + 1][j - 1].f == FLT_MAX
                        || cellDetails[i + 1][j - 1].f > fNew) {
                        openList.insert(make_pair(
                            fNew, make_pair(i + 1, j - 1)));

                        // Update the details of this cell
                        cellDetails[i + 1][j - 1].f = fNew;
                        cellDetails[i + 1][j - 1].g = gNew;
                        cellDetails[i + 1][j - 1].h = hNew;
                        cellDetails[i + 1][j - 1].parent_i = i;
                        cellDetails[i + 1][j - 1].parent_j = j;
                    }
                }
			}
		}
	}

	// When the destination cell is not found and the open
	// list is empty, then we conclude that we failed to
	// reach the destination cell. This may happen when the
	// there is no way to destination cell (due to
	// blockages)
	if (foundDest == false)
		printf("Failed to find the Destination Cell\n");

	return 0;
}
//-----------------------------------------------------------------------------------

void mission_goals(){
    tuple<float,float> inv_goals;
    float input_goal_x, input_goal_y;
    // fulllog_file_ << "-----------GOALS--------------" << endl;    

    // inv_goals = make_tuple(16.83527374267578,-4.076648712158203);
    // goals.emplace_back(inv_goals);
    // tie(input_goal_x,input_goal_y) = goals[0];
    // fulllog_file_ << "GOAL 0: [" << input_goal_x << " | " << input_goal_y << "]" << endl;

    // inv_goals = make_tuple(16.635780334472656,2.5355117321014404);
    // goals.emplace_back(inv_goals);
    // tie(input_goal_x,input_goal_y) = goals[1];
    // fulllog_file_ << "GOAL 1: [" << input_goal_x << " | " << input_goal_y << "]" << endl;

    // inv_goals = make_tuple(10.314708709716797,43.58147430419922);
    // goals.emplace_back(inv_goals);
    // tie(input_goal_x,input_goal_y) = goals[2];
    // fulllog_file_ << "GOAL 2: [" << input_goal_x << " | " << input_goal_y << "]" << endl;

    // inv_goals = make_tuple(13.89610767364502,2.775299072265625);
    // goals.emplace_back(inv_goals);
    // tie(input_goal_x,input_goal_y) = goals[3];
    // fulllog_file_ << "GOAL 3: [" << input_goal_x << " | " << input_goal_y << "]" << endl;

    // inv_goals = make_tuple(13.953405380249023,43.73188781738281);
    // goals.emplace_back(inv_goals);
    // tie(input_goal_x,input_goal_y) = goals[4];
    // fulllog_file_ << "GOAL 4: [" << input_goal_x << " | " << input_goal_y << "]" << endl;

    // inv_goals = make_tuple(12.793737411499023,2.642454147338867);
    // goals.emplace_back(inv_goals);
    // tie(input_goal_x,input_goal_y) = goals[5];
    // fulllog_file_ << "GOAL 5: [" << input_goal_x << " | " << input_goal_y << "]" << endl;

    // fulllog_file_ << "-----------GOALS--------------" << endl;

    inv_goals = make_tuple(3.0571606159210205,6.464037895202637);
    goals.emplace_back(inv_goals);

    inv_goals = make_tuple(7.30134391784668,12.985001564025879);
    goals.emplace_back(inv_goals);

    inv_goals = make_tuple(1.979628562927246,18.99486541748047);
    goals.emplace_back(inv_goals);
    
    inv_goals = make_tuple(-0.04924583435058594,8.970008850097656);
    goals.emplace_back(inv_goals);

    inv_goals = make_tuple(10.125747680664062,2.951190948486328);
    goals.emplace_back(inv_goals);

    inv_goals = make_tuple(10.06579303741455,-7.0252604484558105);
    goals.emplace_back(inv_goals);

    inv_goals = make_tuple(16.586750030517578,-6.57015323638916);
    goals.emplace_back(inv_goals);

    inv_goals = make_tuple(25.265361785888672,-11.598235130310059);
    goals.emplace_back(inv_goals);

    inv_goals = make_tuple(28.645061492919922,-20.9417667388916);
    goals.emplace_back(inv_goals);

    inv_goals = make_tuple(15.961433410644531,-15.847126007080078);
    goals.emplace_back(inv_goals);

    inv_goals = make_tuple(11.151702880859375,-6.412162780761719);
    goals.emplace_back(inv_goals);

    inv_goals = make_tuple(9.609451293945312,0.26978492736816406);
    goals.emplace_back(inv_goals);

    inv_goals = make_tuple(1.9898185729980469,5.972867488861084);
    goals.emplace_back(inv_goals);

}

void mission_goals_from_file(){
    tuple<float,float> inv_goals;
    float input_goal_x, input_goal_y;
    int cell;
    fulllog_file_ << "-----------GOALS--------------" << endl;    
    for (int x = 0; x < object_goals_.size(); x++) {
        tie(cell,input_goal_x,input_goal_y) = object_goals_[x];
        fulllog_file_ << "GOAL 0: [" << input_goal_x << " | " << input_goal_y << "]" << endl;
        cout << "INDICE: " << x << " CELL: " << cell << " PX: " << input_goal_x << " PY: " << input_goal_y << endl;
        // inv_goals = make_tuple(input_goal_x-1.0,input_goal_y-1.0);
        /*
            Para os testes com os goals randômicos, estou retirando a substração de 1 do goal
            para que não acabe dentro da parede o goal.
        */
        inv_goals = make_tuple(input_goal_x,input_goal_y);
        goals.emplace_back(inv_goals);
    }
    fulllog_file_ << "-----------GOALS--------------" << endl;
}

void init_file(std::string arq_name){

    std::stringstream name_stream;
    name_stream << "./sim_time_" << arq_name << ".txt";
    std::string file_name = name_stream.str();
    objects_map_file_.open(file_name,ios::app);
}

void init_fulllog_file(std::string arq_name){

    std::stringstream name_stream;
    name_stream << "./full_log_" << arq_name << ".txt";
    std::string file_name = name_stream.str();
    fulllog_file_.open(file_name,ios::app);
}

void init_CALCS_file(std::string arq_name){

    std::stringstream name_stream;
    name_stream << "./formulas_" << arq_name << ".txt";
    std::string file_name = name_stream.str();
    calc_file_.open(file_name,ios::app);
}

void open_file(){
    objects_file_.open("./objects_in_map.txt", ios::in);
    if (objects_file_.is_open()) {
        cout << "FILE OPENED SUCCEFULLY!" << endl;
    } else {
        cout << "COULD NOT OPEN CHOOSEN FILE!" << endl;
    }
}

void read_file(){
    string line, line_aux;
    int cell_value = 0;
    bool first_colum = false;
    tuple<int,float,float> aux_goals;
    vector<float> aux_pos;
    while(getline(objects_file_, line)){
        stringstream st(line);
        while(getline(st, line_aux, ';')){
            cout << line_aux << endl;
            if (!first_colum) {
                cell_value = stoi(line_aux);
                first_colum = true;    
            } else {
                aux_pos.emplace_back(stof(line_aux));
            }
        }
        cout << "----------------------" << endl;
        first_colum = false;
        aux_goals = make_tuple(cell_value,aux_pos[0],aux_pos[1]);
        object_goals_.emplace_back(aux_goals);
        cell_value = 0;
        aux_pos.clear();
    }
    objects_file_.close();
}

void write_in_file(int index, int last_index, std::chrono::steady_clock::time_point start_time, std::chrono::steady_clock::time_point end_time, int size){
    if (objects_map_file_.is_open()) {
        if (index < size) {
            std::cout << "[REAL TIME] | GOAL [" << last_index << "-> " << index << "]" << " | Time elapsed = " << std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count() << "[s]" << std::endl;
            objects_map_file_ << "[REAL TIME] | GOAL [" << last_index << "-> " << index << "]" << " | Time elapsed = " << std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count() << "[s]" << std::endl;
            fulllog_file_ << "[REAL TIME] | GOAL [" << last_index << "-> " << index << "]" << " | Time elapsed = " << std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count() << "[s]" << std::endl;
        } else {
            last_index = 0;
            std::cout << "[REAL TIME] | GOAL [" << last_index << "-> " << index << "]" << " | Time elapsed = " << std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count() << "[s]" << std::endl;
            objects_map_file_ << "[REAL TIME] | GOAL [" << last_index << "-> " << index << "]" << " | Time elapsed = " << std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count() << "[s]" << std::endl;
            fulllog_file_ << "[REAL TIME] | GOAL [" << last_index << "-> " << index << "]" << " | Time elapsed = " << std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count() << "[s]" << std::endl;
        }
    }else{
        cout << "POR ALGUM MOTIVO O ARQUIVO NAO PODE SER ABERTO" << endl;
    }
}

void write_in_file_stime(int index, int last_index, int start_time, int end_time, int size){
    if (objects_map_file_.is_open()) {
        if (index < size) {
            std::cout << "[SIMULATION TIME] | GOAL [" << last_index << "-> " << index << "]" << " | Time elapsed = " << (end_time - start_time) << "[s]" << std::endl;
            objects_map_file_ << "[SIMULATION TIME] | GOAL [" << last_index << "-> " << index << "]" << " | Time elapsed = " << end_time - start_time << "[s]" << std::endl;
            fulllog_file_ << "[SIMULATION TIME] | GOAL [" << last_index << "-> " << index << "]" << " | Time elapsed = " << end_time - start_time << "[s]" << std::endl;
        } else {
            last_index = 0;
            std::cout << "[SIMULATION TIME] | GOAL [" << last_index << "-> " << index << "]" << " | Time elapsed = " << end_time - start_time << "[s]" << std::endl;
            objects_map_file_ << "[SIMULATION TIME] | GOAL [" << last_index << "-> " << index << "]" << " | Time elapsed = " << end_time - start_time << "[s]" << std::endl;
            fulllog_file_ << "[SIMULATION TIME] | GOAL [" << last_index << "-> " << index << "]" << " | Time elapsed = " << end_time - start_time << "[s]" << std::endl;
        }
    }else{
        cout << "POR ALGUM MOTIVO O ARQUIVO NAO PODE SER ABERTO" << endl;
    }
}

//---------------------------------------------------ADICIONADO 03/01/23--------------------------------------------------------------------------------
/*
    Vou adicionar o componente de teste com barreiras dinâmicas obtidas através da análise da quantidade de pessoas no corredor.
    Neste momento estou utilizando as barreiras de maneira fixa, que são adicionadas por meio da leitura do arquivo de barreiras ('ctldraw_teste.txt'),
    que foi feito pelo Husky em uma run separada. Assim que o conceito estiver testado, vou realizar um teste para que seja feito em tempo de execução.

    Configs do Teb para trocar e fazer funcionar estes testes:
        --> dynamic_obstacle_dist = 2.1
        --> weight_dynamic_obstacle = 850.00
        --> weight_dynamic_obstacle_inflation = 8.0
        --> max_global_plan_lookahead_dist = 10.0



    AJUSTAR O SISTEMA DE LOOP E LIMPEZA DO CAMINHO DESENHADO NO LANE_MAP

    A --> B
    160;23.437129974365234;2.652937650680542
    160;20.03060531616211;43.382686614990234
*/

void init_map(){
    for (int x = 0; x < 4000; x++) {
        for (int y = 0; y < 4000; y++) {
            original_map_[x][y] = -1;
            modified_map_[x][y] = -1;
        }
    }
}

void copy_original_map(){
    cout << "ENTREI NO ORIGINAL" << endl;
    for(int x = 0; x < grid_map_.info.width; x++){
        int multi = x * grid_map_.info.width;
        for(int y = 0; y < grid_map_.info.height; y++){
            int index = y + multi;
            original_map_[x][y] = grid_map_.data[index];
        }
    }
}

void copy_modified_map(){
    cout << "ENTREI NO MODIFICADO" << endl;
    for(int x = 0; x < grid_map_.info.width; x++){
        int multi = x * grid_map_.info.width;
        for(int y = 0; y < grid_map_.info.height; y++){
            int index = y + multi;
            modified_map_[x][y] = grid_map_.data[index];
        }
    }
}

void grid_update_custom(){
    for(int x = 0; x < grid_map_.info.width; x++){
        int multi = x * grid_map_.info.width;
        for(int y = 0; y < grid_map_.info.height; y++){
            int index = y + multi;
            lane_map_.data[index] = modified_map_[x][y];
        }
    }
}

void grid_update_clear(){
    for(int x = 0; x < grid_map_.info.width; x++){
        int multi = x * grid_map_.info.width;
        for(int y = 0; y < grid_map_.info.height; y++){
            int index = y + multi;
            clean_map_.data[index] = original_map_[x][y];
        }
    }
}

void grid_astar_update_clear(){
    for(int x = 0; x < grid_map_.info.width; x++){
        for(int y = 0; y < grid_map_.info.height; y++){
            grid_a[x][y] = original_map_[x][y];
        }
    }
}

void grid_callback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg){
    grid_map_.header.frame_id = map_msg->header.frame_id;
    grid_map_.header.seq = map_msg->header.seq;
    grid_map_.header.stamp = map_msg->header.stamp;
    grid_map_.info.resolution = map_msg->info.resolution;
    grid_map_.info.origin = map_msg->info.origin;
    grid_map_.info.height = map_msg->info.height;
    grid_map_.info.width = map_msg->info.width;
    // cout << "CHEGUEI AQUI NESTE MOMENTO DEPOIS DO GRID_MAP RECEBER AS INFORMACOES" << endl;

    lane_map_.header.frame_id = map_msg->header.frame_id;
    lane_map_.header.seq = map_msg->header.seq;
    lane_map_.header.stamp = map_msg->header.stamp;
    lane_map_.info.resolution = map_msg->info.resolution;
    lane_map_.info.origin = map_msg->info.origin;
    lane_map_.info.height = map_msg->info.height;
    lane_map_.info.width = map_msg->info.width;
    // cout << "CHEGUEI AQUI NESTE MOMENTO DEPOIS DO LANE_MAP RECEBER AS INFORMACOES" << endl;
    if (setup_map) {
        grid_map_.data = map_msg->data;
        lane_map_.data = map_msg->data;
        copy_original_map();
        copy_modified_map();
        cout << "COPIEI OS MAPAS EM AMBOS OS VETORES!!!!!!!!!!!!" << endl;
        setup_map = false;
    } else {
        // if (!can_publish){
            // for (int x = 0; x < map_msg->info.width*map_msg->info.height; x++) {
                // if (grid_map_.data[x] != SUITCASE_VALUE || grid_map_.data[x] != PERSON_VALUE || grid_map_.data[x] != VASE_VALUE || grid_map_.data[x] != BICYCLE_VALUE) {
                    // lane_map_.data[x] = map_msg->data[x];
                // }
            // }
        // }
    }
}

std::tuple<int,int> odom2cell(float odom_pose_x, float odom_pose_y){
    int i = odom_pose_x/grid_map_.info.resolution - grid_map_.info.origin.position.x/grid_map_.info.resolution;
    int j = odom_pose_y/grid_map_.info.resolution - grid_map_.info.origin.position.x/grid_map_.info.resolution;
    return std::make_tuple(j,i);
}

void enable_ctldraw_obstacles(int block_index){
    // --abre o arquivo;--
    // ctldraw_file_.open("./ctldraw_teste.txt", ios::in);
    // if (ctldraw_file_.is_open()) {
        // cout << "FILE OPENED SUCCEFULLY!" << endl;
        
        // --le o arquivo;--
        // string line, line_aux;
        // int cont = 0;
        // int vertexes[4];
        // while(getline(ctldraw_file_, line)){
            // stringstream st(line);
            // while(getline(st, line_aux, ';')){
            //     cout << line_aux << endl;
            //     vertexes[cont] = stoi(line_aux);
            //     cont++;
            // }
            // cont = 0;
            // // --desenha as barreiras;--
            // cout << "VERTEXES: [ " << vertexes[0] << " | " << vertexes[1] << " | " << vertexes[2] << " | " << vertexes[3] << " ]" << endl;
            
            // //---------------------------------------------------------------ADICIONADO 23/01/23---------------------------------------------------------------
            // /*
            //     Para fazer a verficação se o caminho passou pela marcação, neste caso do exemplo, tem que utilizar o vertex[0] e vertex[1].
            // */
            // float cx,cy;
            // tie(cx,cy) = cell2odom(vertexes[2],vertexes[0]);
            // cout << "POSICAO NO MAPA DA MARCACAO | [ CX , CY ] : [ " << cx << " , " << cy << " ]" << endl;
            // vertex1_x_ = cx;
            // vertex1_y_ = cy;
            // tie(cx,cy) = cell2odom(vertexes[3],vertexes[1]);
            // cout << "POSICAO NO MAPA DA MARCACAO | [ CX , CY ] : [ " << cx << " , " << cy << " ]" << endl;
            // vertex2_x_ = cx;
            // vertex2_y_ = cy;
            //-------------------------------------------------------------------------------------------------------------------------------------------------

            // for (int y = vertexes[0]; y < vertexes[1]; y++) {
            //     for (int x = vertexes[2]; x < vertexes[3]; x++) {
            //         modified_map_[y][x] = 100;
            //     }
            // }

            //--------------------------ADICIONADO 30/01/23--------------------------------------
            int v1,v2,v3,v4,qnt_p;
            float ov1,ov2,ov3,ov4;
            tie(v1,v2,v3,v4,ov1,ov2,ov3,ov4,qnt_p) = block_vertex_[block_index];
            cout << "[ V1 , V2 , V3 , V4 ] : [ " << v1 << " , " << v2 << " , " << v3 << " , " << v4 << " ]" << endl; 
            for (int y = v2; y < v4; y++) {
                for (int x = v1; x < v3; x++) {
                    modified_map_[y][x] = 100;
                }
            }
            //-----------------------------------------------------------------------------------

            // -------------------------
        // }
        already_drawed_ = true;
        // -----------------
    // } else {
    //     cout << "COULD NOT OPEN CHOOSEN FILE!" << endl;
    // }
    // -------------------
}

void read_vertex(){
    ctldraw_file_.open("./ctldraw_teste.txt", ios::in);
    if (ctldraw_file_.is_open()) {  
        cout << "FILE OPENED SUCCEFULLY!" << endl;
        string line, line_aux;
        // int cont = 0;
        int vertexes[4];
        while(getline(ctldraw_file_, line)){
            int cont = 0;
            int qnt_person = 0;
            stringstream st(line);
            while(getline(st, line_aux, ';')){
                if (cont <= 3) {
                    cout << line_aux << endl;
                    vertexes[cont] = stoi(line_aux);
                    cont++;
                } else {
                    cout << line_aux << endl;
                    qnt_person = stoi(line_aux);
                }
            }
            cont = 0;
            cout << "VERTEXES: [ " << vertexes[0] << " | " << vertexes[1] << " | " << vertexes[2] << " | " << vertexes[3] << " ]" << endl;
            float cx,cy;
            // tie(cx,cy) = cell2odom(vertexes[2],vertexes[0]);
            //TESTE COM OS VALORES RANDOMICOS
            tie(cx,cy) = cell2odom(vertexes[0],vertexes[1]);
            cout << "POSICAO NO MAPA DA MARCACAO | [ CX , CY ] : [ " << cx << " , " << cy << " ]" << endl;
            vertex1_x_ = cx;
            vertex1_y_ = cy;
            // tie(cx,cy) = cell2odom(vertexes[3],vertexes[1]);
            //TESTE COM OS VALORES RANDOMICOS
            tie(cx,cy) = cell2odom(vertexes[2],vertexes[3]);
            cout << "POSICAO NO MAPA DA MARCACAO | [ CX , CY ] : [ " << cx << " , " << cy << " ]" << endl;
            vertex2_x_ = cx;
            vertex2_y_ = cy;
            vertexes_[0] = vertexes[0];
            vertexes_[1] = vertexes[1];
            vertexes_[2] = vertexes[2];
            vertexes_[3] = vertexes[3];
            tuple<int,int,int,int,float,float,float,float,int> v_aux;
            v_aux = make_tuple(vertexes[0],vertexes[1],vertexes[2],vertexes[3],vertex1_x_,vertex1_y_,vertex2_x_,vertex2_y_,qnt_person);
            block_vertex_.emplace_back(v_aux);
        }
    } else {
        cout << "COULD NOT OPEN CHOOSEN FILE!" << endl;
    }
    ctldraw_file_.close();
}

void grid_block_for_astar(int block_index){
    // --abre o arquivo;--
    // ctldraw_file_.open("./ctldraw_teste.txt", ios::in);
    // if (ctldraw_file_.is_open()) {
    //     cout << "FILE OPENED SUCCEFULLY!" << endl;
        
        // --le o arquivo;--
        // string line, line_aux;
        // int cont = 0;
        // int vertexes[4];
        // while(getline(ctldraw_file_, line)){
        //     stringstream st(line);
        //     while(getline(st, line_aux, ';')){
        //         cout << line_aux << endl;
        //         vertexes[cont] = stoi(line_aux);
        //         cont++;
        //     }
        //     cont = 0;
        //     // --desenha as barreiras;--
            // cout << "#################################################################################################################" << endl;
            // cout << "GRID_BLOCK_FOR_ASTAR" << endl;
            // cout << "VERTEXES: [ " << vertexes_[0] << " | " << vertexes_[1] << " | " << vertexes_[2] << " | " << vertexes_[3] << " ]" << endl;
            // cout << "#################################################################################################################" << endl;
            
        //     //---------------------------------------------------------------ADICIONADO 23/01/23---------------------------------------------------------------
        //     /*
        //         Para fazer a verficação se o caminho passou pela marcação, neste caso do exemplo, tem que utilizar o vertex[0] e vertex[1].
        //     */
        //     float cx,cy;
        //     tie(cx,cy) = cell2odom(vertexes[2],vertexes[0]);
        //     cout << "POSICAO NO MAPA DA MARCACAO | [ CX , CY ] : [ " << cx << " , " << cy << " ]" << endl;
        //     vertex1_x_ = cx;
        //     vertex1_y_ = cy;
        //     tie(cx,cy) = cell2odom(vertexes[3],vertexes[1]);
        //     cout << "POSICAO NO MAPA DA MARCACAO | [ CX , CY ] : [ " << cx << " , " << cy << " ]" << endl;
        //     vertex2_x_ = cx;
        //     vertex2_y_ = cy;
            //-------------------------------------------------------------------------------------------------------------------------------------------------

            // for (int y = vertexes_[0]; y < vertexes_[1]; y++) {
            //     for (int x = vertexes_[2]; x < vertexes_[3]; x++) {
            //         grid_a[y][x] = 100;
            //     }
            // }
            // -------------------------
            //--------------------------ADICIONADO 30/01/23--------------------------------------
            int v1,v2,v3,v4,qnt_p;
            float ov1,ov2,ov3,ov4;
            tie(v1,v2,v3,v4,ov1,ov2,ov3,ov4,qnt_p) = block_vertex_[block_index];
            for (int y = v2; y < v4; y++) {
                for (int x = v1; x < v3; x++) {
                    grid_a[y][x] = 100;
                }
            }
            //-----------------------------------------------------------------------------------
        // }
        already_drawed_ = true;
        astar_mod_ = true;
        // -----------------
    // } else {
    //     cout << "COULD NOT OPEN CHOOSEN FILE!" << endl;
    // }
    // -------------------
}

// void disable_all_ctldraw_obstacles(){
    
// }

// void disable_point_ctldraw_obstacle(){
    
// }

void enable_ctldraw_callback(const std_msgs::Bool& ectl_msg){
    enable_ctldraw_ = ectl_msg.data;
}

void disable_all_ctldraw_callback(const std_msgs::Bool& dctl_msg){
    disable_all_ctldraw_ = dctl_msg.data;
}

void enable_patrol_callback(const std_msgs::Bool& epatrol_msg){
    enable_patrol_ = epatrol_msg.data;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------

//--------------------------ADICIONADO 17/01/23--------------------------------------
void gazebo_sim_time_callback(const gazebo_msgs::PerformanceMetrics::ConstPtr& time_msg){
    // cout << "####[GAZEBO TIME]#### || " << time_msg->header.seq << endl;
    // cout << "####[GAZEBO TIME]#### || " << time_msg->sensors[0] << endl;
    // cout << "####[GAZEBO TIME]#### || " << time_msg->header.stamp.sec << endl;
    gazebo_secs_ = time_msg->header.stamp.sec;
}

void clock_callback(const rosgraph_msgs::Clock::ConstPtr& time_msg){
    // cout << "###[CLOCK]### || " << time_msg->clock.sec;
}
//-----------------------------------------------------------------------------------

//--------------------------ADICIONADO 20/01/23--------------------------------------
void global_path_callback(const nav_msgs::Path::ConstPtr& path_msgs){
    global_path_.poses = path_msgs->poses;
    // for (int x = 0; x < global_path_.poses.size(); x++) {
    //     cout << "POSE_X [" << x << "] - " << global_path_.poses[x].pose.position.x << endl;
    //     cout << "POSE_Y [" << x << "] - " << global_path_.poses[x].pose.position.y << endl;
    // }
}
//-----------------------------------------------------------------------------------

//--------------------------ADICIONADO 24/01/23--------------------------------------
float calc_theta(int block_index){
    float theta;
    int v1,v2,v3,v4,qnt_p;
    float ov1,ov2,ov3,ov4;
    tie(v1,v2,v3,v4,ov1,ov2,ov3,ov4,qnt_p) = block_vertex_[block_index];

    if (ov2 > ov4) {
        theta = ov2 - ov4;
    } else {
        theta = ov4 - ov2;
    }

    return theta;
}

float calc_threshold_path(int block_index){
    float alpha,theta,beta,gama;

    theta = calc_theta(block_index);

    // for (int phi = 0; phi < 20; phi++) {
        int phi = 4;
        alpha = 1 - (phi/theta);
        beta = (phi/theta) - phi;
        /*
            VOU UTILIZAR A FORMULA DE GAMA, JÁ QUE ENQUANTO O RESULTADO FOR NEGATIVO É POSSIVEL CONSIDERAR QUE HÁ UMA PEQUENA QUANTIDADE DE PHI DENTRO DE THETA*.
            *OBS.: AIND É NECESSÁRIO REALIZAR MAIS TESTES EM RELAÇÃO A ESTA FÓRMULA, DEVIDO AINDA NÃO SER CONSIDERADO A DISTÂNCIA DO OUTRO CAMINHO QUE FOI GERADO. ATÉ 
            O MOMENTO ESTE CÁLCULO ESTA CONSIDERANDO SOMENTE SE O ESPAÇO ESTÁ MUITO POPULOSO.
        */
        gama = (phi/theta) - 1;
        cout << "PHI : " << phi << " | THETA : " << theta << " | ALPHA : " << alpha << " | BETA : " << beta << " | GAMA : " << gama << " | PHI/THETA : " << (phi/theta) << endl;
    // }

    return gama;
}
//-----------------------------------------------------------------------------------

//--------------------------ADICIONADO 25/01/23--------------------------------------
void copy_modified_to_astar_grid(){
    for (int x = 0; x < 4000; x++) {
        for (int y = 0; y < 4000; y++) {
            grid_a[x][y] = modified_map_[x][y];
        }
    }
    astar_mod_ = true;
}
//-----------------------------------------------------------------------------------

//--------------------------ADICIONADO 26/01/23--------------------------------------
void amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_msg){
    AMCL_POSE_[0] = amcl_msg->pose.pose.position.x;
    AMCL_POSE_[1] = amcl_msg->pose.pose.position.y;
    Quaternionf q;
    q.x() = amcl_msg->pose.pose.orientation.x;
    q.y() = amcl_msg->pose.pose.orientation.y;
    q.z() = amcl_msg->pose.pose.orientation.z;
    q.w() = amcl_msg->pose.pose.orientation.w;
    auto euler = q.toRotationMatrix().eulerAngles(0,1,2);
    AMCL_POSE_[2] = euler[2];
}

bool path_verification(float goal_x, float goal_y, int block_index){
    int gcx,gcy,scx,scy;
    float gamma;

    tie(gcx,gcy) = odom2cell(goal_x,goal_y);
    tie(scx,scy) = odom2cell(AMCL_POSE_[0],AMCL_POSE_[1]);

    astar_mod_ = false;
    int beta = aStarSearch_MOD(scx,scy,gcx,gcy,block_index);
    astar_path_pub.publish(test_path_);

    calc_file_ << "BETA: " << beta << endl;

    if (isPathInBlock_) {
        gamma = calc_threshold_path(block_index);

        grid_block_for_astar(block_index);

        int omega = aStarSearch_MOD(scx,scy,gcx,gcy,block_index);
        astar_path_mod_pub.publish(test_path_);

        float epsilon = beta + (beta*gamma);
        float lambda = abs(beta - omega) * abs(gamma);
        float epsilon_mod = beta + lambda;

        cout << "BETA : " << beta << " | OMEGA : " << omega << " | EPSILON : " << epsilon << " | LAMBDA : " << lambda << " | EPSILON_MOD : " << epsilon_mod << endl;
        calc_file_ << "BETA : " << beta << " | OMEGA : " << omega << " | EPSILON : " << epsilon << " | LAMBDA : " << lambda << " | EPSILON_MOD : " << epsilon_mod << endl;

        isPathInBlock_ = false;

        if (epsilon > omega) {
            return true;
        } else {
            return false;
        }
    } else {
        calc_file_ << "PATH IS FREE!" << endl;
        return false;
    }
}
//-----------------------------------------------------------------------------------

int main(int argc, char **argv){
    
    ros::init(argc, argv, "SOWDC_move");
    ros::NodeHandle node;
    // std::ofstream sim_time_file;

    //--------------------------ADICIONADO 03/01/23--------------------------------------
    init_map();
    //-----------------------------------------------------------------------------------

    init_file((std::string)argv[2]);
    init_CALCS_file((std::string)argv[2]);
    init_fulllog_file((std::string)argv[2]);
    
    // mission_goals();
    open_file();
    read_file();
    mission_goals_from_file();

    

    std::stringstream move_base_topic_stream;
    move_base_topic_stream << "/" << (std::string)argv[1] << "/move_base";
    std::string move_base_topic = move_base_topic_stream.str();

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_(move_base_topic);

    move_base_msgs::MoveBaseGoal goals_output;

    //--------------------------ADICIONADO 03/01/23--------------------------------------
    ros::Subscriber grid_map = node.subscribe("/lane_map", 1, grid_callback);
    ros::Publisher lane_map_pub = node.advertise<nav_msgs::OccupancyGrid>("/lane_map",10);
    ros::Publisher map_pub = node.advertise<nav_msgs::OccupancyGrid>("/map",10);

    ros::Subscriber enable_ctldraw = node.subscribe("/enable_ctldraw", 1, enable_ctldraw_callback);
    ros::Subscriber disable_all_ctldraw = node.subscribe("/disable_all_ctldraw", 1, disable_all_ctldraw_callback);
    ros::Subscriber enable_patrol_sub = node.subscribe("/enable_patrol", 1, enable_patrol_callback);
    //-----------------------------------------------------------------------------------

    //--------------------------ADICIONADO 17/01/23--------------------------------------
    ros::Subscriber gazebo_sim_time_sub = node.subscribe("/gazebo/performance_metrics", 1000, gazebo_sim_time_callback);
    //-----------------------------------------------------------------------------------

    //--------------------------ADICIONADO 20/01/23--------------------------------------
    std::stringstream move_base_path_topic_stream;
    move_base_path_topic_stream << "/" << (std::string)argv[1] << "/move_base/TebLocalPlannerROS/global_plan";
    std::string move_base_path_topic = move_base_path_topic_stream.str();
    ros::Subscriber global_path_sub = node.subscribe(move_base_path_topic, 1000, global_path_callback);
    //-----------------------------------------------------------------------------------

    //--------------------------ADICIONADO 24/01/23--------------------------------------
    astar_path_pub = node.advertise<nav_msgs::Path>("/astar_path",10);
    //-----------------------------------------------------------------------------------
    //--------------------------ADICIONADO 25/01/23--------------------------------------
    astar_path_mod_pub = node.advertise<nav_msgs::Path>("/astar_path_mod",10);
    //-----------------------------------------------------------------------------------
    //--------------------------ADICIONADO 26/01/23--------------------------------------
    std::stringstream amcl_topic_stream;
    amcl_topic_stream << "/" << (std::string)argv[1] << "/amcl_pose";
    std::string amcl_topic = amcl_topic_stream.str();
    ros::Subscriber amcl_sub = node.subscribe(amcl_topic, 1000, amcl_pose_callback);
    //-----------------------------------------------------------------------------------

    ros::Rate rate(10);

    float input_goal_x, input_goal_y;
    bool setup = true;
    int last_index = 0;
    int sim_laps = 0;
    bool finished_lap = false;
    bool enable_function = true;
    int gt_start, gt_end = 0;
    bool teste_astar = false;

    while(ros::ok()){
        if (grid_map_.info.width > 0) {
            if (!move_base_client_.isServerConnected()){
                ros::spinOnce();
                rate.sleep();
            } else {

                //--------------------------ADICIONADO 26/01/23--------------------------------------
                read_vertex();
                //-----------------------------------------------------------------------------------

                // if (!grid_map_.data.empty()){
                //     if (!teste_astar) {
                //         cout << "ENTREI PARA FAZER O A*" << endl;
                //         int gcx,gcy,scx,scy;
                //         float gamma;
                //         tie(gcx,gcy) = odom2cell(19.88,43.43);
                //         tie(scx,scy) = odom2cell(16.89,-3.94);
                //         Pair goal = make_pair(gcx,gcy);
                //         Pair start = make_pair(scx,scy);
                //         cout << "CELSS: [SX ; SY] - [GX ; GY] | [ " << scx << " ; " << scy << " ] - [ " << gcx << " ; " << gcy << " ]" << endl;
                //         cout << "JA TRANSFORMEI AS POSICOES, VOU EXECUTAR O A*" << endl;
                        
                //         enable_ctldraw_obstacles();
                //         grid_update_custom();
                //         int c = 0;
                //         while (c < 30) {
                //             lane_map_pub.publish(lane_map_);
                //             // map_pub.publish(lane_map_);
                //             c++;
                //         }
                //         astar_mod_ = false;
                //         int beta = aStarSearch_MOD(scx,scy,gcx,gcy);
                //         gamma = calc_threshold_path();
                //         // int as = as_mod(scx,scy,gcx,gcy);
                //         // bool tV = isValid(scx,scy);
                //         // cout << "TESTE DE OUTRAS FUNCOES: " << tV << endl;
                //         teste_astar = true;
                //         enable_function = false;
                //         astar_path_pub.publish(test_path_);

                //         copy_modified_to_astar_grid();
                //         int omega = aStarSearch_MOD(scx,scy,gcx,gcy);
                //         // calc_threshold_path();
                //         astar_path_mod_pub.publish(test_path_);

                //         cout << "BETA : " << beta << " | OMEGA : " << omega << " | EPSILON : " << (beta + (beta*gamma)) << endl;

                //     }
                // }
                //--------------------------ADICIONADO 03/01/23--------------------------------------
                // cout << "INICIOOOOOOOOOOO" << endl;
                if (enable_function) {
                    if (sim_laps < 20) {
                        if (enable_patrol_) {
                            for (int index = 0; index < goals.size(); index++) {
                                tie(input_goal_x,input_goal_y) = goals[index];
                                cout << "GOAL [" << index-1 << " -> " << index << "] FOR " << (std::string)argv[1] << ": [ " << input_goal_x << " | " << input_goal_y << " ] " << endl;
                                fulllog_file_ << "GOAL [" << index-1 << " -> " << index << "] FOR " << (std::string)argv[1] << ": [ " << input_goal_x << " | " << input_goal_y << " ] " << endl;
                                goals_output.target_pose.header.frame_id = "map";
                                goals_output.target_pose.pose.position.x = input_goal_x;
                                goals_output.target_pose.pose.position.y = input_goal_y;
                                goals_output.target_pose.pose.position.z = 0;
                                goals_output.target_pose.pose.orientation.x = 0.0;
                                goals_output.target_pose.pose.orientation.y = 0.0;
                                goals_output.target_pose.pose.orientation.z = 0.70;
                                goals_output.target_pose.pose.orientation.w = 0.70;

                                //--------------------------ADICIONADO 26/01/23--------------------------------------
                                    //--------------------------ADICIONADO 30/01/23--------------------------------------
                                    calc_file_ << "GOAL [" << index-1 << " -> " << index << "] FOR " << (std::string)argv[1] << ": [ " << input_goal_x << " | " << input_goal_y << " ] " << endl;
                                    int v1,v2,v3,v4,qnt_p;
                                    float ov1,ov2,ov3,ov4;
                                    tie(v1,v2,v3,v4,ov1,ov2,ov3,ov4,qnt_p) = block_vertex_[index];
                                    calc_file_ << "VERTEXES_CELL: [ " << v1 << " | " << v2 << " | " << v3 << " | " << v4 << " ] | " << "VERTEXES_ODOM: [ " << ov1 << " | " << ov2 << " | " << ov3 << " | " << ov4 << " ]" << " | QNT_PEOPLE: " << qnt_p << endl;
                                    //-----------------------------------------------------------------------------------
                                if (path_verification(input_goal_x,input_goal_y,index)) {
                                    cout << "EPSILON MAIOR QUE OMEGA, REALIZANDO O FECHAMENTO DO LOCAL E UTILIZANDO O CAMINHO ALTERNATIVO!" << endl;
                                    enable_ctldraw_obstacles(index);
                                    grid_update_custom();
                                    int c = 0;
                                    while (c < 30) {
                                        lane_map_pub.publish(lane_map_);
                                        // map_pub.publish(lane_map_);
                                        c++;
                                    }
                                    for (int cr = 0; cr < 11; cr++){rate.sleep();}
                                    ros::spinOnce();
                                    rate.sleep();
                                } else {
                                    cout << "EPSILON MENOR QUE OMEGA, MANTENDO O CAMINHO ORIGINAL!" << endl;
                                    grid_astar_update_clear();
                                    //SÓ PRA VISUALIZAR O QUE ESTA ACONTECENDO NO MAPA
                                    enable_ctldraw_obstacles(index);
                                    grid_update_custom();
                                    int c = 0;
                                    while (c < 30) {
                                        lane_map_pub.publish(lane_map_);
                                        // map_pub.publish(lane_map_);
                                        c++;
                                    }
                                    for (int cr = 0; cr < 11; cr++){rate.sleep();}
                                    ros::spinOnce();
                                    rate.sleep();
                                }
                                //-----------------------------------------------------------------------------------
        
                                move_base_client_.sendGoal(goals_output);
                                last_index = index;
                                // index_++;
                                setup = false;
                                if (!time_started_) {
                                    time_started_ = true;
                                    start_time_ = std::chrono::steady_clock::now();
                                    //--------------------------ADICIONADO 17/01/23--------------------------------------
                                    cout << "[START] | GAZEBO SIMULATION SECS: " << gazebo_secs_ << endl;
                                    gt_start = gazebo_secs_; 
                                    //-----------------------------------------------------------------------------------
                                }

                                //--------------------------ADICIONADO 17/01/23--------------------------------------
                                cout << "[START] | GAZEBO SIMULATION SECS: " << gazebo_secs_ << endl;
                                gt_start = gazebo_secs_;
                                //-----------------------------------------------------------------------------------

                                if (move_base_client_.waitForResult()) {
                                    if (!time_started_) {
                                        time_started_ = true;
                                        start_time_ = std::chrono::steady_clock::now();
                                        //--------------------------ADICIONADO 17/01/23--------------------------------------
                                        cout << "[START] | GAZEBO SIMULATION SECS: " << gazebo_secs_ << endl;
                                        gt_start = gazebo_secs_;
                                        //-----------------------------------------------------------------------------------
                                    } else {
                                        ros::spinOnce();
                                        rate.sleep();
                                        end_time_ = std::chrono::steady_clock::now();
                                        //--------------------------ADICIONADO 17/01/23--------------------------------------
                                        cout << "[END] | GAZEBO SIMULATION SECS: " << gazebo_secs_ << endl;
                                        gt_end = gazebo_secs_;
                                        cout << "[ELAPSED TIME] | SIMULATION TIME DIFF: " << gt_end - gt_start << endl;
                                        //-----------------------------------------------------------------------------------
                                        // time_started_ = false;
                                        std::cout << "TERMINEI VOU GRAVAR - GOAL [" << last_index << "]" << " | Time elapsed = " << std::chrono::duration_cast<std::chrono::seconds>(end_time_ - start_time_).count() << "[s]" << std::endl;
                                        write_in_file(last_index,last_index-1,start_time_,end_time_,goals.size());
                                        write_in_file_stime(last_index,last_index-1,gt_start,gt_end,goals.size());
                                        start_time_ = std::chrono::steady_clock::now();
                                        // //--------------------------ADICIONADO 17/01/23--------------------------------------
                                        // cout << "[START] | GAZEBO SIMULATION SECS: " << gazebo_secs_ << endl;
                                        // gt_start = gazebo_secs_;
                                        // //-----------------------------------------------------------------------------------
                                    }
                                    cout << "LAST_INDEX: " << last_index << " INDEX_: " << index+1 << endl;
                                    fulllog_file_ << "LAST_INDEX: " << last_index << " INDEX_: " << index+1 << endl;
                                    cout << "GOAL [" << last_index << " -> " << index+1 << "] FOR HUSKY: [ " << input_goal_x << " | " << input_goal_y << " ] " << endl;
                                    fulllog_file_ << "GOAL [" << last_index << " -> " << index+1 << "] FOR HUSKY: [ " << input_goal_x << " | " << input_goal_y << " ] " << endl;

                                    //--------------------------ADICIONADO 30/01/23--------------------------------------
                                    for (int cr = 0; cr < 20; cr++){rate.sleep();}
                                    clean_map_ = grid_map_;
                                    grid_update_clear();
                                    grid_astar_update_clear();
                                    lane_map_pub.publish(clean_map_);
                                    //-----------------------------------------------------------------------------------
                                }
                            }

                            // if (!global_path_.poses.empty()) {
                            //     for (int x = 0; x < global_path_.poses.size(); x++) {
                            //         cout << "POSE_X [" << x << "] - " << global_path_.poses[x].pose.position.x << endl;
                            //         cout << "POSE_Y [" << x << "] - " << global_path_.poses[x].pose.position.y << endl;
                            //     }
                            // }

                            if (sim_laps % 2 != 0) {
                                clean_map_ = grid_map_;
                                grid_update_clear();
                                lane_map_pub.publish(clean_map_);
                                already_drawed_ = false;
                                cout << "----------------------------------- WB: "<< sim_laps << " ------------------------------------" << std::endl;
                                objects_map_file_ << "----------------------------------- WB: "<< sim_laps << " ------------------------------------" << std::endl;
                                fulllog_file_ << "----------------------------------- WB: "<< sim_laps << " ------------------------------------" << std::endl;
                            } else {
                                enable_ctldraw_ = true;
                                cout << "----------------------------------- NB: "<< sim_laps << " ------------------------------------" << std::endl;
                                objects_map_file_ << "----------------------------------- NB: "<< sim_laps << " ------------------------------------" << std::endl;
                                fulllog_file_ << "----------------------------------- NB: "<< sim_laps << " ------------------------------------" << std::endl;
                            }
                            //-----------------------------------------------------------------------------------
                            cout << "----------------------------------- END OF LAP: "<< sim_laps << " ------------------------------------" << std::endl;
                            objects_map_file_ << "----------------------------------- END OF LAP: "<< sim_laps << " ------------------------------------" << std::endl;
                            fulllog_file_ << "----------------------------------- END OF LAP: "<< sim_laps << " ------------------------------------" << std::endl;
                            sim_laps++;
                            if (enable_ctldraw_) {
                                if (!already_drawed_) {
                                    cout << "###################ENTREI NA PARTE DO DRAW####################" << endl;
                                    // enable_ctldraw_obstacles(index);
                                    grid_update_custom();
                                    int c = 0;
                                    while (c < 30) {
                                        lane_map_pub.publish(lane_map_);
                                        // map_pub.publish(lane_map_);
                                        c++;
                                    }
                                    for (int cr = 0; cr < 11; cr++){rate.sleep();}
                                    
                                    enable_ctldraw_ = false;
                                } 
                                // else {
                                //     clean_map_ = grid_map_;
                                //     grid_update_clear();
                                //     lane_map_pub.publish(clean_map_);
                                //     already_drawed_ = false;
                                // }
                            }
                            enable_patrol_ = false;
                        } else {
                            for (int wait = 0; wait < 20; wait++){rate.sleep();}
                            enable_patrol_ = true;
                        }
                    } else {
                        cout << "FINALIZADO O PROCESSO DE SIMULACAO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
                        objects_map_file_.close();
                        fulllog_file_.close();
                    }

                } 
                //--------------------------ADICIONADO 03/01/23--------------------------------------
                // else {
                //     if (disable_all_ctldraw_) {
                //         clean_map_ = grid_map_;
                //         grid_update_clear();
                //         lane_map_pub.publish(clean_map_);
                //         disable_all_ctldraw_ = false;
                //     }
                // }
                //-----------------------------------------------------------------------------------
            }
        }else{
            cout << "AINDA NAO RECEBI O MAPA" << endl;
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}