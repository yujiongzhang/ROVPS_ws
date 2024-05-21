#ifndef GRIDMAP_H_
#define GRIDMAP_H_

#include <iostream>
#include <vector>

class GridMap {
public:
    GridMap(int width, int height) : width_(width), height_(height) {
        grid_.resize(width, std::vector<int>(height, 0));
    }

    void OccupyCell(int x, int y) {
        if (IsValidCell(x, y)) {
            grid_[x][y] = 1;
        }
    }

    void FreeCell(int x, int y) {
        if (IsValidCell(x, y)) {
            grid_[x][y] = 0;
        }
    }

    bool IsCellOccupied(int x, int y) const {
        if (IsValidCell(x, y)) {
            return (grid_[x][y] == 1);
        }
        return false;
    }

    void PrintMap() const {
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                std::cout << grid_[x][y] << " ";
            }
            std::cout << std::endl;
        }
    }

private:
    bool IsValidCell(int x, int y) const {
        return (x >= 0 && x < width_ && y >= 0 && y < height_);
    }

    int width_;
    int height_;
    std::vector<std::vector<int>> grid_;
};


#endif