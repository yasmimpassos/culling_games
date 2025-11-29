#include "path_moves.h"

using namespace std;

vector<string> path_to_moves(const vector<pair<int,int>>& coords) {
    vector<string> moves;

    for (size_t i = 1; i < coords.size(); ++i) {
        int pr = coords[i-1].first;
        int pc = coords[i-1].second;
        int r  = coords[i].first;
        int c  = coords[i].second;

        if (r == pr && c == pc + 1) {
            moves.push_back("right");
        } else if (r == pr && c == pc - 1) {
            moves.push_back("left");
        } else if (c == pc && r == pr + 1) {
            moves.push_back("down");
        } else if (c == pc && r == pr - 1) {
            moves.push_back("up");
        } else {
            moves.push_back("unknown");
        }
    }

    return moves;
}
