from p1_support import load_level, show_level, save_level_costs
from math import sqrt
from heapq import heappop, heappush

def dijkstras_shortest_path(initial_position, destination, graph, adj):
    """ Searches for a minimal cost path through a graph using Dijkstra's algorithm.

    Args:
        initial_position: The initial cell from which the path extends.
        destination: The end location for the path.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        If a path exits, return a list containing all cells from initial_position to destination.
        Otherwise, return None.
    """

    #Initialize Lists/Dicts for search
    queue = [(0,initial_position)]
    dist = {}
    prev = {}
    path = []
    prev[initial_position] = None
    dist[initial_position] = 0

    #while priority queue still valid
    while queue:
        running_cost, current_cell = heappop(queue)
        #If at destination backtrack appending shortest path
        if current_cell == destination:
            backtrack = current_cell
            while prev[backtrack] != None:
                path.insert(0,backtrack)
                backtrack = prev[backtrack]
            path.insert(0,backtrack)

            #print the total cost of the path and return formed path
            print ('total cost =  ', running_cost,'\n')
            return path

        #If not goal, explore neighbor edges
        else:
            #For each neighbor edge
            for cell, cost in navigation_edges(graph, current_cell):
                #Calculate pathcost and add to queue if appropriate
                pathcost = cost + dist[current_cell]
                if cell not in prev.keys() or pathcost < dist[cell]:
                    dist[cell] = pathcost
                    prev[cell] = current_cell
                    heappush(queue, (pathcost,cell))

    return None

def dijkstras_shortest_cost(initial_position, destination, graph, adj):
    """ Searches for a minimal cost path through a graph using Dijkstra's algorithm.
        Implemented for the calculation of cost to all tiles. Primarily for de-bugging.
        Essentially the same function as above with less work and different return.

    Args:
        initial_position: The initial cell from which the path extends.
        destination: The end location for the path.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        If a path exits, return the cost.
        Otherwise, return None.
    """

    #Initialize Lists/Dicts for search
    queue = [(0,initial_position)]
    dist = {}
    prev = {}

    prev[initial_position] = None
    dist[initial_position] = 0

    #while priority queue still valid
    while queue:
        running_cost, current_cell = heappop(queue)

        #If at destination return cost
        if current_cell == destination:
            return running_cost

        #If not goal, explore neighbor edges
        else:
            #For each neighbor edge
            for cell, cost in navigation_edges(graph, current_cell):
                #Calculate pathcost and add to queue if appropriate
                pathcost = cost + dist[current_cell]
                if cell not in prev.keys():
                    dist[cell] = pathcost
                    prev[cell] = current_cell
                    heappush(queue, (pathcost,cell))
                elif pathcost < dist[cell]:
                    dist[cell] = pathcost
                    prev[cell] = current_cell
                    heappush(queue, (pathcost,cell))
    return None


def dijkstras_shortest_path_to_all(initial_position, graph, adj):

    """ Calculates the minimum cost to every reachable cell in a graph from the initial_position.
        Used for debugging/ensuring dijkstra works in every case on map.

    Args:
        initial_position: The initial cell from which the path extends.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        A dictionary, mapping destination cells to the cost of a path from the initial_position.
    """

    xs, ys = zip(*(list(graph['spaces'].keys()) + list(graph['walls'])))
    x_lo, x_hi = min(xs), max(xs)
    y_lo, y_hi = min(ys), max(ys)
    finaldict = {}

    #Iterate through each cell and find the cost to get there
    for j in range(y_lo, y_hi + 1):
        for i in range(x_lo, x_hi + 1):
            #cell we are calculating for
            cell = (i, j)
            totalcost = dijkstras_shortest_cost(initial_position, cell, graph, adj)

            if totalcost != None:
                finaldict[cell] = totalcost
            else:
                finaldict[cell] = "inf"
    return finaldict


def navigation_edges(level, cell):

    """ Provides a list of adjacent cells and their respective costs from the given cell.

    Args:
        level: A loaded level, containing walls, spaces, and waypoints.
        cell: A target location.

    Returns:
        A list of tuples containing an adjacent cell's coordinates and the cost of the edge joining it and the
        originating cell.

        E.g. from (0,0):
            [((0,1), 1),
             ((1,0), 1),
             ((1,1), 1.4142135623730951),
             ... ]
    """
    #contain adj coordinates
    final = []

    # left
    templist = list(cell) # change to list to edit
    templist[0] -= 1 # change left coord (col)
    tempcell = tuple(templist) # revert back to tuple
    init_cost = cell_cost(level, cell) # cost of current cell
    tempcost = calculate_cost(level, tempcell, False, init_cost) #cost to get there

    if tempcost != None:
        temptuple = (tempcell, tempcost)
        final.append(temptuple) # add to final list

    # top-left
    templist = list(cell) # change to list to edit
    templist[0] -= 1 # change left coord (col)
    templist[1] -= 1 # change right coord (row)
    tempcell = tuple(templist) # revert back to tuple
    init_cost = cell_cost(level, cell) # cost of current cell
    tempcost = calculate_cost(level, tempcell, True, init_cost) #cost to get there
    if tempcost != None:
        temptuple = (tempcell, tempcost)
        final.append(temptuple) # add to final list

    # top
    templist = list(cell) # change to list to edit
    templist[1] -= 1 # change right coord (row)
    tempcell = tuple(templist) # revert back to tuple
    init_cost = cell_cost(level, cell) # cost of current cell
    tempcost = calculate_cost(level, tempcell, False, init_cost) #cost to get there
    if tempcost != None:
        temptuple = (tempcell, tempcost)
        final.append(temptuple) # add to final list

    # top-right
    templist = list(cell) # change to list to edit
    templist[0] += 1 # change left coord (col)
    templist[1] -= 1 # change right coord (row)
    tempcell = tuple(templist) # revert back to tuple
    init_cost = cell_cost(level, cell) # cost of current cell
    tempcost = calculate_cost(level, tempcell, True, init_cost) #cost to get there
    if tempcost != None:
        temptuple = (tempcell, tempcost)
        final.append(temptuple) # add to final list

    # right
    templist = list(cell) # change to list to edit
    templist[0] += 1 # change left coord (col)
    tempcell = tuple(templist) # revert back to tuple
    init_cost = cell_cost(level, cell) # cost of current cell
    tempcost = calculate_cost(level, tempcell, False, init_cost) #cost to get there
    if tempcost != None:
        temptuple = (tempcell, tempcost)
        final.append(temptuple) # add to final list

    # bottom-right
    templist = list(cell) # change to list to edit
    templist[0] += 1 # change left coord (col)
    templist[1] += 1 # change right coord (row)
    tempcell = tuple(templist) # revert back to tuple
    init_cost = cell_cost(level, cell) # cost of current cell
    tempcost = calculate_cost(level, tempcell, True, init_cost) #cost to get there
    if tempcost != None:
        temptuple = (tempcell, tempcost)
        final.append(temptuple) # add to final list

    # bottom
    templist = list(cell) # change to list to edit
    templist[1] += 1 # change right coord (row)
    tempcell = tuple(templist) # revert back to tuple
    init_cost = cell_cost(level, cell) # cost of current cell
    tempcost = calculate_cost(level, tempcell, False, init_cost) #cost to get there
    if tempcost != None:
        temptuple = (tempcell, tempcost)
        final.append(temptuple) # add to final list

    # bottom-left
    templist = list(cell) # change to list to edit
    templist[0] -= 1 # change left coord (col)
    templist[1] += 1 # change right coord (row)
    tempcell = tuple(templist) # revert back to tuple
    init_cost = cell_cost(level, cell) # cost of current cell
    tempcost = calculate_cost(level, tempcell, True, init_cost) #cost to get there
    if tempcost != None:
        temptuple = (tempcell, tempcost)
        final.append(temptuple) # add to final list

    finals = tuple(final)
    return finals

def test_route(filename, src_waypoint, dst_waypoint):
    """ Loads a level, searches for a path between the given waypoints, and displays the result.

    Args:
        filename: The name of the text file containing the level.
        src_waypoint: The character associated with the initial waypoint.
        dst_waypoint: The character associated with the destination waypoint.

    """

    # Load and display the level.
    level = load_level(filename)

    # Retrieve the source and destination coordinates from the level.
    src = level['waypoints'][src_waypoint]
    dst = level['waypoints'][dst_waypoint]

    print("\nRoute from {0} to {1}".format(src_waypoint, dst_waypoint))

    # Search for and display the path from src to dst.
    path = dijkstras_shortest_path(src, dst, level, navigation_edges)
    if path:
        show_level(level, path)
    else:
        print("No path possible!\n")

def cost_to_all_cells(filename, src_waypoint, output_filename):
    """ Loads a level, calculates the cost to all reachable cells from
    src_waypoint, then saves the result in a csv file with name output_filename.

    Args:
        filename: The name of the text file containing the level.
        src_waypoint: The character associated with the initial waypoint.
        output_filename: The filename for the output csv file.
    """

    # Retrieve the source coordinates from the level.
    src = level['waypoints'][src_waypoint]

    # Calculate the cost to all reachable cells from src and save to a csv file.
    costs_to_all_cells = dijkstras_shortest_path_to_all(src, level, navigation_edges)
    save_level_costs(level, costs_to_all_cells, output_filename)

def cell_cost(level, cell):
    """ Checks the level for the cost of the cell passed in.

    Args:
        level: A loaded level, containing walls, spaces, and waypoints.
        cell: A target location.

    Returns:
        The cost of the tile passed in.
        returns None if wall.
    """

    space = level.get("spaces")
    waypoint = level.get("waypoints")

    if cell in space.keys():
        cost = space.get(cell)
    elif cell in waypoint.keys():
        cost = 1
    else:
        return None

    return cost

def calculate_cost(level, cell, is_diagonal, init_cost):
    """ Calculates the cost from one cell to another.

    Args:
        level: A loaded level, containing walls, spaces, and waypoints.
        cell: A starting location.
        is_diagonal: A value indicating if the step taken is diagonal or not
        init_cost: The initial cost of the cell

    Returns:
        The cost of the tile passed in.
    """

    space = level.get("spaces")
    waypoint = level.get("waypoints")

    #start as none
    tempcost = None

    if cell in space.keys():
        tempcost = space.get(cell)
    elif cell in waypoint.keys():
        tempcost = 1
    elif cell in waypoint.keys():
        pass

    
    if tempcost != None:
        #adj is not diagonal:
        if not is_diagonal:
            cost = (init_cost*0.5)+(tempcost*0.5)
            return cost
        #adj is  diagonal:
        else:
            cost = (init_cost*sqrt(2)*0.5)+(tempcost*0.5*sqrt(2))
            return cost



if __name__ == '__main__':

    filename = 'my_maze.txt'

    waypoints = 'abcd'

    # Load and display the level.
    level = load_level(filename)
    print("\nMaze:\n")
    show_level(level)

    for x in waypoints:
        for y in waypoints:
            #finds best route
            test_route(filename, x, y)

    for x in waypoints:
        # Calculates cost to all cells in dungeon and returns a csv file showing those costs. For debugging.
        cost_to_all_cells(filename, x, 'maze_costs-'+x+'.csv')
