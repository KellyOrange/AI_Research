# Funnel Algorithm Path Smoothing on Navigation Meshes

**Chee Qing Tan · Jia Min Tan · Rou Hui Chan**
CS380/CS580 — Artificial Intelligence for Games

A C++ implementation of the funnel algorithm for smoothing paths on a
triangulated navigation mesh, compared against rubber-banding and
Catmull-Rom spline interpolation. The C++ program builds a nav mesh,
runs A* over it, extracts the portal sequence, and runs all three
smoothing methods; results are exported to `paths.json` and rendered
by the browser-based visualizer in `index.html`.

## Build and run
in Visual Studio Code, run the following in Linux
g++ -std=c++17 main.cpp pathsmooth.cpp metrics.cpp jsonExport.cpp navMesh.cpp -o main.exe
./main.exe
python3 -m http.server 8000

Copy and paste the following URL in Chrome
http://localhost:8000/index.html

## File overview

| File | Contents |
|---|---|
| `main.cpp` | Sets up test corridors, runs each smoothing method, exports JSON |
| `pathsmooth.cpp/h` | Funnel, rubber-band, and Catmull-Rom implementations |
| `navMesh.cpp/h` | Grid → CCW triangle mesh, A* over triangle adjacency, portal extraction |
| `metrics.cpp/h` | Path length and turn-count calculations |
| `jsonExport.cpp/h` | Writes `paths.json` for the visualizer |
| `vec2.h`, `portal.h`, `rect.h` | Small structs |
| `index.html` | Browser-based visualizer |
| `paths.json` | Generated at runtime by the C++ program |