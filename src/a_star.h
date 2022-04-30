#pragma once
#include <vector>


using namespace std;


struct Node {
	vector<int> pos;
	float f = 0.;
	float g = 0.;
	float h = 0.;
	vector<int> parent_pos = {-1, -1};
	Node(vector<int> pos, float f=0., float g=0., float h=0.);
};


class AStar {
	private:
		vector<Node> open = {};
		vector<Node> closed = {};
		vector<vector<int>> grid;
		vector<int> start;
		vector<int> goal;
		int n_rows;
		int n_cols;
		vector<vector<int>> obstacles;
		vector<vector<int>> adjacents = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
		bool goal_found = false;
		float greedy = 1.;

		static inline bool compare_f(Node& a, Node& b) {return a.f < b.f;};
		inline float heuristic(vector<int>& a, vector<int>& b, string type);
		void add_successor_to_open(Node& n);
		bool check_legal_move(vector<int>& pos);
		void generate_successors(Node parent_node);
		void backtrack(Node node);

	public:
		AStar(int n_rows, int n_cols, vector<vector<int>> obs, float greedy=1.);
		void solve(vector<int> start, vector<int> goal);
		void show_grid();
		void save_grid_to_file();
};
