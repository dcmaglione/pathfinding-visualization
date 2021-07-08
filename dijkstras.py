#### Visualization of Dijkstra's Pathfinding Algorithm ###
#### Dominic C. Maglione - 07/01/2021 ###


### CONTROLS ###
# Left-click: Specifics listed below
#   - First click: Places start node
#   - Second click: Places end node
#   - Third and all other clicks: Places barriers
# Right-click: Eraser, hold down and hover over to erase nodes
# Space: Starts algorithm
# r: Clears/resets grid


### SETUP ###
## IMPORTS ##
import pygame
import math
from queue import PriorityQueue
from pygame import fastevent


# Initializes window and sets size #
WIDTH = 800
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("Dijkstra's Algorithm Visualization")


## COLORS ##
RED = pygame.Color("#bf616a")
ORANGE = pygame.Color("#d08770")
YELLOW = pygame.Color("#ebcb8b")
GREEN = pygame.Color("#a3be8c")
BLUE = pygame.Color("#81a1c1")
LIGHTBLUE = pygame.Color("#88c0d0")
PURPLE = pygame.Color("#b48ead")
WHITE = pygame.Color("#eceff4")
GREY = pygame.Color("#4c566a")
BLACK = pygame.Color("#2e3440")


## NODE ##
# Defines nodes used in visualization
class Node:
    # Initializes #
    def __init__(self, row, col, width, total_rows):
        # Determines grid
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        
        # Additional values
        self.color = WHITE
        self.neighbors = []
        self.width = width
        self.total_rows = total_rows
        
        
    # Accessor Methods #
    # Returns position of node
    def get_pos(self):
        return self.row, self.col
    
    # Determines if node is closed
    def is_closed(self):
        return self.color == PURPLE
    
    # Determines if node is open
    def is_open(self):
        return self.color == LIGHTBLUE
    
    # Determines if node is barrier
    def is_barrier(self):
        return self.color == BLACK
    
    # Determines if node is starting node
    def is_start(self):
        return self.color == GREEN
    
    # Determines if node is ending node
    def is_end(self):
        return self.color == RED
    
    
    # Modifier methods #
    # Resets node
    def reset(self):
        self.color = WHITE
    # Sets node to closed
    def make_closed(self):
        self.color = PURPLE
        
    # Sets node to open
    def make_open(self):
        self.color = LIGHTBLUE
        
    # Sets node as barrier
    def make_barrier(self):
        self.color = BLACK
        
    # Sets node as starting node
    def make_start(self):
        self.color = GREEN
        
    # Sets node as ending node
    def make_end(self):
        self.color = RED
        
    # Sets the node as path
    def make_path(self):
        self.color = YELLOW
        
        
    # Additional methods #
    # Draws node onto window
    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))
        
    # Updates neighboring nodes
    def update_neighbors(self, grid):
        # Initializes list
        self.neighbors = []
        
        # Checks if neighboring node below is barrier
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier(): # DOWN
            self.neighbors.append(grid[self.row + 1][self.col])
            
        # Checks if neighboring node above is barrier
        if self.row > 0 and not grid[self.row - 1][self.col].is_barrier(): # UP
            self.neighbors.append(grid[self.row - 1][self.col])
            
        # Checks if neighboring node to the right is barrier
        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier(): # RIGHT
            self.neighbors.append(grid[self.row][self.col + 1])
            
        # Checks if neighboring node to the left is barrier
        if self.col > 0 and not grid[self.row][self.col - 1].is_barrier(): # LEFT
            self.neighbors.append(grid[self.row][self.col - 1])
    
    
    # Less than #
    def __lt__(self, other):
        return False
    
    
### ALGORITHM ###
# Reconstructs path after end is found
def reconstruct_path(came_from, current, draw):
    # Iterates through list
    while current in came_from:
        current = came_from[current]
        
        # Draws node
        current.make_path()
        draw()
        
# Defines the heuristic function (h-score) w/ Manhattan distance
def h(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2) + abs(y1 - y2)

# Defines algorithm (A*)
def algorithm(draw, grid, start, end):
    # Initializes counter and open set
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start))
    came_from = {}  # Prior node
    
    # Sets g-score, current shortest distance from start to node
    g_score = {node: float("inf") for row in grid for node in row}
    g_score[start] = 0
    
    # Sets f-score, predicted distance from current node to end 
    f_score = {node: float("inf") for row in grid for node in row}
    f_score[start] = h(start.get_pos(), end.get_pos())
    
    # Keeps track of items in priority queue
    open_set_hash = {start}
    
    # Runs until open set is empty (every node has been considered)
    while not open_set.empty():
        # Quits visualization if user closes
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
        
        # Gets smallest possible node and syncs with hash by removing
        current = open_set.get()[2]
        open_set_hash.remove(current)
        
        # If current node is end path has been found
        if current == end:
            # Reconstructs path using helper method
            reconstruct_path(came_from, end, draw)
            
            # Recolors start and end 
            start.make_start()
            end.make_end()
            return True
        
        # Considers neighbors of current node
        for neighbor in current.neighbors:
            # Checks next g-score
            temp_g_score = g_score[current] + 1
            
            # If g-score is shorter save path
            if temp_g_score < g_score[neighbor]:
                # Updates prior node and scores
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())
                
                # Checks if neighbor is already in hash
                if neighbor not in open_set_hash:
                    count += 1
                    
                    # Store neighbor into open set and mark open
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.make_open()
               
        # Draws path so far     
        draw()
        
        # If the current node is not start mark closed
        if current != start:
            current.make_closed()
            
    # Only returns if path not found
    return False


### HELPER FUNCTIONS ###
# Creates grid
def make_grid(rows, width):
    # Defines grid and gap (width of nodes)
    grid = []
    gap = width // rows
    
    # Appends to list to form grid
    for i in range(rows):
        grid.append([])
        for j in range (rows):
            node = Node(i, j, gap, rows)
            grid[i].append(node)
            
    return grid
                
# Draws the grid lines
def draw_grid(win, rows, width):
    # Defines gap
    gap = width // rows
    
    # Draws lines
    for i in range(rows):
        pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap))
        for j in range(rows):
            pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, width))
            
# Draws complete grid
def draw(win, grid, rows, width):
    # Refills win
    win.fill(WHITE)
    
    # Returns win to state
    for row in grid:
        for node in row:
            node.draw(win)
            
    # Draws gridlines
    draw_grid(win, rows, width)
    pygame.display.update()
       
# Determines position of mouse in row, col format
def get_clicked_position(pos, rows, width):
    # Determines gap
    gap = width // rows
    y, x = pos
    
    # Determines row and column
    row = y // gap
    col = x // gap
    
    return row, col


### MAIN ###
# Main function loop
def main(win, width):
    # Define number of rows and grid
    ROWS = 50
    grid = make_grid(ROWS, width)
    
    start = None
    end = None
    
    run = True
    started= False
    
    # Keeps program running
    while run:
        # Calls draw helper function
        draw(win, grid, ROWS, width)
        
        # Checks for events in pygame
        for event in pygame.event.get():
            # Checks if user tries to exit
            if event.type == pygame.QUIT:
                run = False
            
            # Checks if left or right mouse button were pressed
            if pygame.mouse.get_pressed()[0]: # LEFT
                # Determines position of mouse within grid
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_position(pos, ROWS, width)
                node = grid[row][col]
                
                # Checks for starting node, if not create it
                if not start and node != end:
                    start = node
                    start.make_start()
                    
                # Checks for ending node, if not create it
                elif not end and node != start:
                    end = node
                    end.make_end()
                    
                # Checks if node is neither, creates barrier
                elif node != start and node != end:
                    node.make_barrier()
                
            elif pygame.mouse.get_pressed()[2]: # RIGHT
                # Determines position of mouse within grid
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_position(pos, ROWS, width)
                node = grid[row][col]
                
                # Resets node
                node.reset()
                
                # Resets start node
                if node == start:
                    start = None
                
                # Resets end node
                elif node == end:
                    end = None
                    
            # Primes algorithm on keypress
            if event.type == pygame.KEYDOWN:
                # If it hasnt started then update all neighbors
                if event.key == pygame.K_SPACE and start and end: # SPACE
                    for row in grid:
                        for node in row:
                            node.update_neighbors(grid)
                            
                    # Starts algorithm
                    algorithm(lambda: draw(win, grid, ROWS, width), grid, start, end)
                    
                # Clears and resets grid
                if event.key == pygame.K_r: # R
                    # Resets values
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)
        
    # Quits when event is exited    
    pygame.quit()
    
# Creates window
main(WIN, WIDTH)