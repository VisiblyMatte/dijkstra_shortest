# Dijkstra's Algorithm Implemention

A python implementation of Dijkstra's Algorithm done across an ASCII map.
A starting and goal point are both designated in the my_map.txt file. This is
parsed and searched with the shortest path from one to another returned with a graphical representation.

Implemented as a project for a class.

### To Run

Simply download and run 

```
Python P1.py
```

## File Descriptions
### p1.py

This file contains all of my work on this project. here you can find my implementation of Dijkstra's
algorithm along with several helper functions, as well as functions created for debugging.

## p1_support.py

This file contains code provided to me as a starting point for the class assignment.
This code is not my own work.

## my_maze.txt

This is the small ASCII map the algorithm will be running on. The project works on any files
of varying size, however this one is small so the shortest path can be easily seen.

X - Wall, cannot be passed through

a - starting point

d - destination point

1,2,3 - "length" of tile