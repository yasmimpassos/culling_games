#include "map_graph.h"

using namespace std;

void map_graph(const shared_ptr<cg_interfaces::srv::GetMap::Response>& response, vector<vector<string>>& matrix_out) {
    if (!response) {
        cerr << "map_graph: response é nullptr\n";
        return;
    }

    auto& flat = response->occupancy_grid_flattened;
    auto& shape = response->occupancy_grid_shape;

    if (shape.size() != 2) {
        cerr << "map_graph: shape inválido — esperava 2 dimensões, shape.size() = " << shape.size() << "\n";
        return;
    }

    int rows = static_cast<int>(shape[0]);
    int cols = static_cast<int>(shape[1]);

    if ((int)flat.size() != rows * cols) {
        cerr << "map_graph: tamanho flat (" << flat.size() << ") diferente de rows*cols (" << (rows * cols) << ")\n";
        return;
    }

    for (int idx = 0; idx < rows * cols; ++idx) {
        int r = idx / cols;
        int c = idx % cols;

        matrix_out[r][c] = flat[idx];
    }
}
