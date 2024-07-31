#include <vector>
#include <unordered_map>

#include "raylib.h"
#define RAYGUI_IMPLEMENTATION
#pragma warning(push, 0)
#include "raygui.h"
#pragma warning(pop)
#include "tinystd/queue.h"

#define CELL_ROWS 10
#define CELL_COLS 10
#define CELL_NULL -1
#define CELL_EMPTY 0
#define CELL_WALL  1
#define CELL_START 2
#define CELL_GOAL  3

struct ivec2
{
    int x;
    int y;
};

char cells[CELL_ROWS][CELL_COLS];
ivec2 CurrentStart = {CELL_NULL, CELL_NULL};
ivec2 CurrentGoal = {CELL_NULL, CELL_NULL};

int DefaultCellSize = 75;
int DefaultCellSpacing = 15;
ivec2 OriginOffset = {300, 100};

inline bool SameCell(ivec2 S, ivec2 T) { return S.x == T.x && S.y == T.y; }
inline bool CellIsValid(ivec2 S) { return S.x != CELL_NULL && S.y != CELL_NULL; }

bool IsMouseOverCell(int MouseX, int MouseY, int PosX, int PosY, int CellSize) {
    return
    (MouseX >= PosX && MouseX <= (PosX + CellSize)) &&
    (MouseY >= PosY && MouseY <= (PosY + CellSize));
}

void HandleInput() {
    int MouseX = GetMouseX();
    int MouseY = GetMouseY();
    for (int i = 0; i < CELL_ROWS; i++) {
        for (int j = 0; j < CELL_COLS; j++) {
            int PosX = (j * DefaultCellSize) + (j * DefaultCellSpacing) + (int)OriginOffset.x;
            int PosY = (i * DefaultCellSize) + (i * DefaultCellSpacing) + (int)OriginOffset.y;
            bool Over = IsMouseOverCell(MouseX, MouseY, PosX, PosY, DefaultCellSize); 
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && Over) {
                if (IsKeyDown(KEY_LEFT_SHIFT)) {
                    if (CellIsValid(CurrentStart)) {
                        cells[CurrentStart.y][CurrentStart.x] = CELL_EMPTY;
                    }
                    cells[i][j] = CELL_START; 
                    CurrentStart = {j, i};
                } else if (IsKeyDown(KEY_LEFT_CONTROL)) {
                    if (CellIsValid(CurrentGoal)) {
                        cells[CurrentGoal.y][CurrentGoal.x] = CELL_EMPTY;
                    }
                    cells[i][j] = CELL_GOAL; 
                    CurrentGoal = {j, i};
                }
                else {
                    cells[i][j] = !cells[i][j];
                }
            }
        }
    }
}

struct cell_hash {
    size_t operator()(ivec2 v) const { 
        return v.x * 31 + v.y;
    }
};

struct cells_compare {
    bool operator()(ivec2 s, ivec2 t) const { 
        return SameCell(s, t);
    }
};

double Distance(ivec2 s, ivec2 t) {
    return sqrt(pow(s.x - t.x, 2) + pow(s.y - t.y, 2));
}

int SearchHeuristic (ivec2 v) { 
    if (cells[v.y][v.x] == CELL_WALL)
        return 10000;
    else
        return (int)Distance(CurrentGoal, v);
}

bool OpenSetCompare(ivec2 parent, ivec2 child) {
    return SearchHeuristic(parent) > SearchHeuristic(child);
}

std::vector<ivec2> ReconstructPath(std::unordered_map<ivec2, ivec2, cell_hash, cells_compare> CameFrom,
                                   ivec2 Current) {
    std::vector<ivec2> Path = {Current};
    while (CameFrom.find(Current) != CameFrom.end()) {
        Current = CameFrom[Current];
        Path.push_back(Current);
    }
    return Path;
}

// Rudimentary implementation of A* search algorithm.
// Reference: https://en.wikipedia.org/wiki/A*_search_algorithm#Pseudocode
std::vector<ivec2> AStarSearch(ivec2 Start, ivec2 Goal, int (*SearchHeuristic)(ivec2)) {
    priority_queue<ivec2> OpenSet;
    std::unordered_map<ivec2, bool, cell_hash, cells_compare> OpenSetItems;
    std::unordered_map<ivec2, ivec2, cell_hash, cells_compare> CameFrom;
    std::unordered_map<ivec2, int, cell_hash, cells_compare> G_Score;
    std::unordered_map<ivec2, int, cell_hash, cells_compare> F_Score;
    
    HeapInit(&OpenSet, 500, OpenSetCompare);
    HeapPush(&OpenSet, Start);
    OpenSetItems[Start] = true;
    
    for (int i = 0; i < CELL_ROWS; i++) {
        for (int j = 0; j < CELL_COLS; j++) {
            ivec2 cell = {j, i};
            if (SameCell(cell, Start)) {
                G_Score[cell] = 0;
                F_Score[cell] = SearchHeuristic(cell);
            } else {
                G_Score[cell] = INT_MAX;
                F_Score[cell] = INT_MAX;
            }
        }
    }
    
    while (OpenSet.Size > 0) {
        ivec2 Current = HeapTop(&OpenSet);
        if (SameCell(Current, Goal)) {
            HeapDestroy(&OpenSet);
            return ReconstructPath(CameFrom, Current);
        }
        HeapPop(&OpenSet);
        OpenSetItems[Current] = false;
        
        int Adjacency[] = {-1, 1, 0, 0};
        for (int i = 0; i < 4; i++) {
            ivec2 Neighbor = { Current.x + Adjacency[i], Current.y + Adjacency[3 - i] };
            if (Neighbor.x < 0 || Neighbor.y < 0) continue;
            int Tentative_G_Score = G_Score[Current] + (int)Distance(Current, Neighbor);
            if (Tentative_G_Score < G_Score[Neighbor]) {
                CameFrom[Neighbor] = Current;
                G_Score[Neighbor] = Tentative_G_Score;
                F_Score[Neighbor] = Tentative_G_Score + SearchHeuristic(Neighbor);
                if (OpenSetItems.find(Neighbor) == OpenSetItems.end() || !OpenSetItems[Neighbor]) {
                    HeapPush(&OpenSet, Neighbor);
                    OpenSetItems[Current] = true;
                }
            }
        }
    }
    HeapDestroy(&OpenSet);
    return {};
}

int main() {
    SetTraceLogLevel(LOG_ERROR);
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    int WindowWidth = 1500;
    int WindowHeight = 1100;
    InitWindow(WindowWidth, WindowHeight, "A* Search");
    
    for (int i = 0; i < CELL_ROWS; i++) {
        for (int j = 0; j < CELL_COLS; j++) {
            cells[i][j] = CELL_EMPTY;
        }
    }
    
    std::vector<ivec2> FinalPath;
    while (!WindowShouldClose()) {
        BeginDrawing();
        Color clear_color = {28, 48, 37, 255};
        ClearBackground(clear_color);
        
        HandleInput();
        
        if (CurrentStart.x >= 0 && CurrentStart.y >= 0 && CurrentGoal.x >= 0 && CurrentGoal.y >= 0) {
            FinalPath = AStarSearch(CurrentStart, CurrentGoal, SearchHeuristic);
            /*for (auto k: path) {
                printf("[%d, %d]\n", k.y, k.x);
            }*/
        }
        
        for (int i = 0; i < CELL_ROWS; i++) {
            for (int j = 0; j < CELL_COLS; j++) {
                Color CellColor;
                if (cells[i][j] == CELL_WALL) {
                    CellColor = {194, 48, 61, 255};
                }
                else if (cells[i][j] == CELL_START) {
                    CellColor = {48, 75, 175, 255};
                }
                else if (cells[i][j] == CELL_GOAL) {
                    CellColor = {175, 175, 48, 255};
                }
                else {
                    CellColor = {68, 117, 91, 255};
                }
                DrawRectangle((j * DefaultCellSize) + (j * DefaultCellSpacing) + (int)OriginOffset.x,
                              (i * DefaultCellSize) + (i * DefaultCellSpacing) + (int)OriginOffset.y,
                              DefaultCellSize,
                              DefaultCellSize,
                              CellColor);
            }
        }
        
        if (!FinalPath.empty()) {
            for (int k = 0; k < FinalPath.size(); k++) {
                int i = FinalPath[k].y;
                int j = FinalPath[k].x;
                Color CellColor = {0, 191, 163, 255};
                DrawRectangle((j * DefaultCellSize) + (j * DefaultCellSpacing) + (int)OriginOffset.x,
                              (i * DefaultCellSize) + (i * DefaultCellSpacing) + (int)OriginOffset.y,
                              DefaultCellSize,
                              DefaultCellSize,
                              CellColor);
            }
        }
        EndDrawing();
    }
    CloseWindow();
}