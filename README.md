# Pathfinding Algorithms

Implementation of Pathfinding Algorithms on road map of Wuhan for my 2022AI course final project. You can have a shot at it at https://liuhaozhe6788.github.io/pathfinding.

## Demo

Here is the webpage for Pathfinding Algorithms: 

![alt text](https://github.com/liuhaozhe6788/pathfinding/blob/master/screenshots/webpage.jpeg?raw=true)

After selecting the algorithm and moving the start and target nodes, click "Start progress". After that, a window would show the execution time and the map would display extended edges and the final path. For ALT Search and ALT Search improved, the map would also show two landmarks, which are inportant for pathfinding. A demo result of ALT Search improved looks like this:

![alt text](https://github.com/liuhaozhe6788/pathfinding/blob/master/screenshots/altim.jpeg?raw=true)

## Four algorithms

**Uniform Cost Search**: aka Dijkstra Algorithm.

**A\* Search**: with euclidean bounds.

**ALT Search**: with triangular inequality lower bounds in [this](https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/GH05.pdf), the landmarks are selected using avoid methods in [this](https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/GW05.pdf).

**ALT Search Improved**: based on ALT search, one can select a subset of two landmarks that are close to and after the target node from all the landmarks before each pathfinding search. 

Generally, ALT Search Improved is the fastest among all four algorithms.

## How to run it with live-server

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


