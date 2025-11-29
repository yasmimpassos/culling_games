#include "graph_adj_matrix.h"
using namespace std;

void build_adjacency_matrix(const vector<vector<string>>& grid, vector<vector<int>>& adj_matrix, int& start_id, int& goal_id) {
    start_id = -1;
    goal_id = -1;

    int rows = static_cast<int>(grid.size());
    int cols = static_cast<int>(grid[0].size());
    
    int N = rows * cols;

    adj_matrix.assign(N, vector<int>(N, 0));

    auto id_of = [cols](int r, int c) {
        return r * cols + c;
    };

    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            string cell = grid[r][c];
            char ch = cell.empty() ? 'b' : cell[0];

            int id = id_of(r,c);
            if (ch == 'b') continue;

            if (ch == 'r') start_id = id;
            if (ch == 't') goal_id = id;

            for (int k = 0; k < 4; ++k) {
                int nr = r + dr[k];
                int nc = c + dc[k];
                if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
                string cell2 = grid[nr][nc];
                char ch2 = cell2.empty() ? 'b' : cell2[0];
                if (ch2 == 'b') continue;

                int id2 = id_of(nr, nc);
                adj_matrix[id][id2] = 1;
                adj_matrix[id2][id] = 1;
            }
        }
    }
}
