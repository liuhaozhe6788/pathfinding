# Pathfinding Algorithms

Implementation of Pathfinding Algorithms on road map of Wuhan.

## Three algorithms

**Uniform Cost Search**: aka Dijkstra Algorithm.

**A\* Search**: with euclidean bounds.

**ALT Search**: with triangular inequality lower bounds in [this](https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/GH05.pdf), the landmark is selected using avoid methods in [this](https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/GW05.pdf).

## How to run it

1. Install [node.js](https://nodejs.org/en/download/)

2. Install live-server with npm command

```
npm i live-server
```

3. Run the website with

```
live-server
```

## Rebind C++ code[optional]

The C++ code is binded with javascript using Emscripten and Embind, so you need to install Emscripten if you want to modify the C++ code.
Install [Emscripten](https://emscripten.org/docs/getting_started/downloads.html)

After modifying the C++ code, run 
```
./build.sh
```

## Troubleshooting
1. Please refresh the page a couple of times if the map and markers are not rendered
2. Please use a VPN if the map is not rendered completely


