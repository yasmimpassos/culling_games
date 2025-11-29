#include "map_printer.h"

using namespace std;

void print_map(const vector<vector<string>>& matrix) {
    for (const auto& row : matrix) {
        for (size_t j = 0; j < row.size(); ++j) {
            const auto& cell = row[j];
            cout << cell;
            if (j + 1 < row.size()) {
                cout << " ";
            }
        }
        cout << endl;
    }
}
