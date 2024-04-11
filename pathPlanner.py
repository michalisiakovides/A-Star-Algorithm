# Function to find the node with the minimum F-score from a list of open nodes
def get_min_f_score_node(open_nodes, f_score):
    min_node = None
    min_score = float('inf')
    for node in open_nodes:
        # Check if the F-score of the current node is less than the minimum score found so far
        if f_score.get(node) < min_score:             
            # If yes, update the minimum node and score
            min_node = node
            min_score = f_score[node]
    return min_node

# Function to find the neighbor nodes of a given node
def get_neighbors(node,grid,ROW,COL): 
    # Initialise an empty list to store the coordinates of valid neighboring nodes
    neighbors = [] 
    directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # up, down, right, left

    for direction in directions:
        neighbor = (node[0] + direction[0], node[1] + direction[1])                 # Calculate the coordinates of the potential neighbor
        # Check if the potential neighbor is within the grid boundaries and not an obstacle(value 0)
        if 0 <= neighbor[0] < COL and 0 <= neighbor[1] < ROW and grid[neighbor[0]][neighbor[1]] != 0:
            # If yes, add the neighbor to the list of valid neighbors
            neighbors.append(neighbor)
    return neighbors

# Function to calculate the Euclidean distance between a node and the goal
def euclidean_distance(node,goal):
    distance = ((goal[0] - node[0])**2 + (goal[1] - node[1])**2) ** 0.5
    return distance
    
# The main path planning function. Additional functions, classes, 
# variables, libraries, etc. can be added to the file, but this
# function must always be defined with these arguments and must 
# return an array ('list') of coordinates (col,row).
#DO NOT EDIT THIS FUNCTION DECLARATION
def do_a_star(grid, start, end, display_message):
    #EDIT ANYTHING BELOW HERE
    
    # Get the size of the grid
    COL = len(grid)
    ROW = len(grid[0])
       
    # Initialise a list containing the start node as the only open node
    open_nodes = [start]
    # Initialise an empty list for closed nodes
    closed_nodes = []

    # Initialise dictionaries to keep track of parents, g_score,h_score and f_score
    # Dictionary to store the parent node for each node
    parent = {start: None}                                      
    g_score = {start: 0}                                        
    h_score = {start: euclidean_distance(start, end)}           
    f_score = {start: g_score[start] + h_score[start]}

    # List to store the final path 
    path = []                             

    # Loop until all open nodes are explored or path is found
    while open_nodes:
        current_node = get_min_f_score_node(open_nodes, f_score)                  # Get the node with the minimum f-score
        if current_node == end:
            # Reconstruct the path
            while current_node is not None:
                path.append(current_node)
                current_node = parent[current_node]
            # Send the path points back to the gui to be displayed
            return path[::-1]  # Reverse path to start -> end order
        
        open_nodes.remove(current_node)                    # Remove the node from open nodes
        closed_nodes.append(current_node)                  # Add the node to closed nodes

        # Explore neighbors of the current node
        for neighbor in get_neighbors(current_node, grid, ROW, COL):
            # Cost of moving from current to neighbor is 1
            next_g_score = g_score[current_node] + 1  
            
            # If neighbor is already in the closed or open lists
            if neighbor in closed_nodes or neighbor in open_nodes:
                # Check if the new path to neighbor is shorter and update the shortest path 
                if next_g_score < g_score.get(neighbor):
                    g_score[neighbor] = next_g_score
                    parent[neighbor] = current_node

                    # If node was on closed list, move it to open list
                    if neighbor in closed_nodes:
                        closed_nodes.remove(neighbor)
                        open_nodes.append(neighbor)
            
            # If the neighbor is neither in open or closed lists, add the node to open list and update its cost from start 
            else:                                      
                parent[neighbor] = current_node
                g_score[neighbor] = next_g_score
                h_score[neighbor] = euclidean_distance(neighbor, end)
                f_score[neighbor] = g_score[neighbor] + h_score[neighbor]
                open_nodes.append(neighbor)
    
    # Send the path points back to the gui to be displayed
    #FUNCTION MUST ALWAYS RETURN A LIST OF (col,row) COORDINATES
    # Return an empty list if no path is found
    return path    
#end of file