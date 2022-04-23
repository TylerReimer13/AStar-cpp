#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include "a_star.h"


using namespace std;


Node::Node(vector<int> pos, float f, float g, float h) {
	this->pos = pos;
	this->f = f;
	this->g = g;
	this->h = h;
};


AStar::AStar(int n_rows, int n_cols, float greedy) {
	vector<vector<int>> zeros_grid(n_rows, vector<int> (n_cols, 0));
	this->grid = zeros_grid;
	this->n_cols = n_cols;
	this->n_rows = n_rows;
	this->greedy = greedy;
};


bool AStar::check_in_bounds(vector<int>& pos) {
	if (pos[0] < 0 || pos[1] < 0 || pos[0] > n_rows || pos[1] > n_cols) {
		return false;
	}

	return true;
};


inline float AStar::heuristic(vector<int>& a, vector<int>& b, string type) {
	if (type == "manhattan")
		return abs(b[0] - a[0]) + abs(b[1] - a[1]);

	// Dont need to compute the square root here, since we are just comparing relative distances
	else
		return pow(b[0] - a[0], 2) + pow(b[1] - a[1], 2);

};


void AStar::add_successor_to_open(Node& n) {
	for (Node& open_node : open) {
		if (n.pos == open_node.pos) {
			if (open_node.f <= n.f) {
				return;
			}
		}
	}

	for (Node& closed_node : closed) {
		if (n.pos == closed_node.pos) {
			if (closed_node.f <= n.f) {
				return;
			}
		}
	}

	//std::cout << "Adding Successor to Open: [" << n.pos[0] << ", " << n.pos[1] << "]" << endl;

	open.push_back(n);
	
};


void AStar::generate_successors(Node parent_node) {
	//std::cout << "Parent Pos: [" << parent_node.pos[0] << ", " << parent_node.pos[1] << "]" << endl;
	for (vector<int> adj : adjacents) {
		std::transform(adj.begin(), adj.end(), parent_node.pos.begin(), adj.begin(), std::plus<int>());

		if (!check_in_bounds(adj)) {
			continue;
		}

		Node successor_node(adj);
		successor_node.parent_pos = parent_node.pos;
		successor_node.g = parent_node.g + heuristic(adj, parent_node.pos, "euclidean");
		successor_node.h = greedy * heuristic(successor_node.pos, goal, "euclidean");
		successor_node.f = successor_node.g + successor_node.h;

		//std::cout << "Candidate Successor Pos: [" << successor_node.pos[0] << ", " << successor_node.pos[1] << "] ";
		//std::cout << "f: " << successor_node.f << ", g: " << successor_node.g << ", h: " << successor_node.h << endl;

		add_successor_to_open(successor_node);

	}

};


void AStar::backtrack(Node node) {
	cout << " --- Final Path --- " << endl;

	Node start_node(start);
	vector<Node> path = {};

	while (node.pos != start) {
		for (Node& closed_node : closed) {
			if (closed_node.pos == node.parent_pos) {
				path.push_back(node);
				grid[node.pos[0]][node.pos[1]] = 2;
				node = closed_node;
				break;
			}
		}
	}

	path.push_back(start_node);

	grid[start[0]][start[1]] = 1;
	grid[goal[0]][goal[1]] = 3;

	for (int i = path.size(); i --> 0;) {
    	cout << "Path Node Pos: [" << path[i].pos[0] << ", " << path[i].pos[1] << "]" << endl;
    	//cout << "[" << path[i].pos[0] << ", " << path[i].pos[1] << "], " << endl;
	}
};


void AStar::show_grid() {
	cout << "## Grid ##" << endl;
	for (int r = 0; r < n_rows; r++) {
		for (int c = 0; c < n_cols; c++) {
			cout << grid[c][r] << " ";
		}
		cout << endl;
	}
};


void AStar::solve(vector<int> start, vector<int> goal) {
	this->start = start;
	this->goal = goal;
	Node start_node(start);
	open.push_back(start_node);

	int n_iter = 0;
	while (open.size() != 0) {
		sort(open.begin(), open.end(), compare_f);  // Sort Nodes by f.
		Node q = open[0];  // Node with smallest 'f' score.

		if (q.pos == goal) {
			goal_found = true;
			cout << "*** GOAL FOUND ***" << endl;
			cout << "Took " << n_iter << " iterations to find goal" << endl;
			backtrack(q);
			break;
		}

		generate_successors(q);
		closed.push_back(q);
		open.erase(open.begin());  // Remove first node from open
		n_iter++;

	}
};


int main() {
	int n_rows = 20;
	int n_cols = 20;
	float greedy  = 1.;
	AStar a_star(n_rows, n_cols, greedy);

	vector<int> s{0, 0};
	vector<int> g{10, 15};
	a_star.solve(s, g);

	//a_star.show_grid();

	return 0;
};