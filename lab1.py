# Lab 1: 8-puzzle. Artificial Intelligence 1
# Name(s): Aidan Glickman, Anthony Lekan
# Email(s): aidgli20@bergen.org, antlek21@bergen.org
from collections import deque
from heapq import *
import copy

INF = float('inf')

#### Part 1: Problem Representation #################################################

### 1a. Implement PuzzleBoard, which represents a configuration of an n-by-n sliding puzzle.
class PuzzleBoard:

    """Use the attribute board to store state data. 
    A tuple is our recommendation, but any *immutable* type will do.
    """
    
    board = ()
    goal = ()

    def __init__(self, tiles, empty=None) :
        """Creates a PuzzleBoard from an n-by-n array of tiles, where
        tiles[row][col] = tile at (row, col). 
        NOTE: You may alter this function's signature as long as the only required
        parameter is tiles and it works as described above when supplied with only that
        """

        self.size = len(tiles)

        temp_board = []

        for row in range(self.size):
            temp_board.append(tuple(tiles[row]))
        self.board = tuple(temp_board)

        if empty is None:
            self.empty = self.find_empty()
        else:
            self.empty = empty


        solved_list = []
        num = 0
        for x in range(0,self.size):
            solved_list.append([])
            for y in range(0,self.size):
                solved_list[x].append(num)
                num += 1

        solved_board = []

        for i in range(self.size):
            solved_board.append(tuple(solved_list[i]))
        self.goal = tuple(solved_board)


    def find_empty(self):
        for y in range(self.size):
            for x in range(self.size):
                if self.board[y][x] == 0:
                    return (y,x)

    def __str__(self):
        """Returns a string representation of the board. 
        Implemented for you; you're welcome!
        """
        str_list = []
        for row in range(self.size):
            for col in range(self.size):
                str_list.append("{:2d} ".format(self.get_tile_at(row,col)))
            str_list.append("\n")
        return "".join(str_list)

    def __eq__(self, other) :
        """Leave this function alone unless you have essential attributes beyond board.
        This is important to make PuzzleBoard immutable and thus usable in sets.
        """
        return (isinstance(other, PuzzleBoard)
                and (self.board == other.board))

    def __hash__(self) :
        """Leave this function alone unless you have essential attributes beyond board.
        This is important to make PuzzleBoard hashable and thus usable in sets
        """
        return hash(self.board)

    def get_goal(self):
        return self.goal

    def get_tile_at(self, row, col):
        """Returns the number of the tile at position (row, col), or 0 if blank
        """
        return self.board[row][col]

    def get_size(self):
        """Returns the board size n
        """
        return self.size

    def is_goal(self):
        """Is this board the goal board? Return a boolean
        """
        return self.goal == self.board

    def get_neighbors(self):
        """Generate and return all neighboring boards in an iterable (e.g. list)
        """
        offs = [[-1, 0], [0, -1], [1, 0], [0, 1]]
        for off in offs:
            switchy = off[0] + self.empty[0]
            switchx = off[1] + self.empty[1]
            if 0 <= switchy < self.size and 0 <= switchx < self.size:
                newboard = list(list(x) for x in self.board)
                newboard[self.empty[0]][self.empty[1]] = newboard[switchy][switchx]
                newboard[switchy][switchx] = 0
                newboard = tuple(tuple(x) for x in newboard)
                yield PuzzleBoard(newboard, empty=(switchy, switchx))

    """Feel free to write additional helper methods. Keep in mind, however, that AbstractState 
    will be used for all of our algorithms, so make sure that its functionality is 
    straightforward enough to be generally useful.
    """

""" You can use boardchecker.py to check that your PuzzleBoard works properly
Writing your own tests below is also a good idea.
"""

### 1b. Implement AbstractState, which serves as a general wrapper for PuzzleBoard. 

class AbstractState:

    def __init__(self, snapshot, parent, path_length):
        """Creates an abstract state which represents a path. Takes a snapshot, the 
        board at the end of the path, its parent which is the preceding AbstractState of
        the path (None if initial state), and path_length which is the number of states 
        from the initial state to this one (zero if initial state)"""

        self.snapshot = snapshot
        self.parent = parent
        self.path_length = path_length

    def __lt__(self, other):
        """Leave this method as is. It is needed to make tuple comparison work with heapq"""
        return True

    def __eq__(self, other):
        """Leave this function alone unless you have essential attributes beyond board.
        This is important to make PuzzleBoard immutable"""
        return (isinstance(other, AbstractState)
                and (self.snapshot == other.snapshot))

    def __hash__(self):
        """Leave this function alone unless you have essential attributes beyond board.
        This is important to make PuzzleBoard hashable"""
        return hash(self.snapshot)

    def get_snapshot(self):
        """ Returns the PuzzleBoard at the end of the path
        """
        return self.snapshot

    def get_parent(self):
        """ Returns the AbstractState that generated this one; 
        that is, the state node one level up the search tree
        """
        return self.parent

    def get_path_length(self):
        """ Returns the length of the path, not counting the initial state.
        AKA search depth
        """
        return self.path_length

    def is_goal(self):
        return self.snapshot.is_goal()

    def get_neighbors(self):
        """Generate and return all neighboring boards in an iterable (e.g. list)
        However, do NOT include the parent's board. This is a useful trick that will
        help improve efficiency for all search algorithms.
        """
        neighbors = []
        boards = self.snapshot.get_neighbors()

        for board in boards:
            neighbors.append(AbstractState(board, self, self.path_length+1))
        return neighbors

    def notailbite(self):
        """
        Searches through this nodes children to check whether any tailbiting has occurred
        """

        node = self
        while node.snapshot != self.snapshot: 
            if not self.parent:
                return False

            node = node.parent
        return True


    """Feel free to write additional helper methods. Keep in mind, however, that AbstractState 
    will be used for all of our algorithms, so make sure that its functionality is 
    straightforward enough to be generally useful.
    """


""" There are no provided tests specifically for AbstractState; its functionality will be 
tested indirectly in the search algorithm checkers
Writing your own tests below is also a good idea.
"""


#### Part 2: Uninformed Search #################################################



### 2a. Implement DFSPuzzleSolver, which performs Depth-First Search 
### with backtracking to a limited depth.

""" Built-in lists are great to use as LIFO queues (aka stacks). 
The append() and pop() methods will be handy.
"""

class DFSPuzzleSolver:

    counts = {"enqueues":0, "extends":0}
    solution = AbstractState(None, None, 0)

    def __init__(self, initial_board, graph_search, max_depth) :
         """Find a solution to the initial puzzle board, up to max_depth, using depth-limited DFS (with backtracking). 
         If graph_search is True, avoid re-exploring paths **see Part 2c** 
         """
         stack = []
         if graph_search: closed = set()
         initial_board = AbstractState(initial_board, None, 0)

         if not initial_board.is_goal():
             stack = [initial_board]
             self.counts["enqueues"] += 1
         else:
            solution = initial_board
            return

         while stack != []:
            curr = stack.pop()
            if graph_search:
                if curr in closed:
                    continue
                closed.add(curr)
            self.counts["extends"] += 1
            if self.counts["extends"] > max_depth:
                self.solution = None
                return

            
            if curr.is_goal():
                self.solution = curr
                return

            for neighbor in curr.get_neighbors():
                if neighbor.notailbite():
                    stack.append(neighbor)
                    self.counts["enqueues"] += 1


    def num_moves(self) :
        """ return number of moves in solution to initial board. If no solution found, return None."""
        try:
            return self.solution.get_path_length()
        except Exception as e:
            return None
        

    def num_enqueues(self) :
        """ return number of nodes enqueued during search, successful or not. """
        return self.counts["enqueues"]

    def num_extends(self) :
        """ return number of nodes extended/expanded during search, successful or not. """
        return self.counts["extends"]

    def get_solution(self) :
        """ returns sequence of PuzzleBoards, initial board to goal board. If no solution found, return None."""
        solution_list = []
        curr = copy.deepcopy(self.solution)
        while curr != None:
            solution_list.insert(0, curr.get_snapshot())
            curr = curr.get_parent()
        return tuple(solution_list)



""" Use dfschecker.py to check that your DFSPuzzleSolver works properly.
Writing your own tests below is also a good idea.
"""



### 2b. Implement BFSPuzzleSolver, which performs Breadth-First Search 

"""
Built-in lists don't implement FIFO Queues very efficiently. A deque is recommended:
https://docs.python.org/3/library/collections.html#collections.deque
"""

class BFSPuzzleSolver:

    def __init__(self, initial_board, graph_search = False) :
        """find a solution to the initial puzzle board using BFS. 
        If graph_search is True, avoid re-exploring paths **see Part 2c**
        """

        # FIFO Queue
        queue = deque()

        # enqueue initial node
        queue.append(AbstractState(initial_board, None, 0))

        # set of visited boards
        visited = set()

        meta = {'enqueues': 1, 'extends': 0}

        while len(queue) > 0:

            node = queue.popleft()

            meta['extends'] += 1

            if node.is_goal():
                self.solution = node
                break

            if graph_search and node in visited:
                continue

            for neighbor in node.get_neighbors():
                if graph_search and neighbor in visited:
                    continue

                if neighbor.notailbite():
                    queue.append(neighbor)
                    meta['enqueues'] += 1

            if graph_search:
                visited.add(node)

        self.meta = meta


    def num_moves(self) :
        """ return number of moves in solution to initial board. If no solution found, return None."""
        try:
            return self.solution.get_path_length()
        except Exception as e:
            return None

    def num_enqueues(self) :
        """ return number of nodes enqueued during search, successful or not. """
        return self.meta['enqueues']

    def num_extends(self) :
        """ return number of nodes extended/expanded during search, successful or not. """
        return self.meta['extends']

    def get_solution(self) :
        """ returns sequence of boards, initial board to goal board. If no solution found, return None."""
        solution_list = []
        curr = copy.deepcopy(self.solution)
        while curr != None:
            solution_list.insert(0, curr.get_snapshot())
            curr = curr.get_parent()
        return tuple(solution_list)

""" Use bfschecker.py to check that your BFSPuzzleSolver works properly.
Writing your own tests below is also a good idea.
"""



### 2c. Go back and add optional "graph-search" to both DFSPuzzleSolver and BFSSolver; if the graph_search 
### parameter is set to True, the algorithm should avoid re-exploring states. 

""" To implement graph search, it may be helpful to use the Set class to 
track nodes that are in the frontier/extended/enqueued.
https://docs.python.org/3/tutorial/datastructures.html#sets
"""
""" Graph search's details can be tricky; exactly what "explored" and "exploring" means is a bit fuzzy, and 
deliberately so! You'll want to mind the distinction between **extending** and **enqueueing**
states. For some algorithms, the distinction is a minor efficiency matter; for others, it could make or 
break the algorithm! 
"""



#### Part 3: Informed Search #################################################

### 3a. First, implement these two heuristic functions. 

def hamming(board):
    """ Return the Hamming distance (number of tiles out of place) of the PuzzleBoard """

    goal = board.goal
    board = board.board

    i = 0

    for col in range(len(goal)):
        for row in range(len(goal)):
            if goal[col][row] is not board[col][row] and goal[col][row] is not 0:
                i += 1

    return i


def manhattan(board):
    """ Return the sum of Manhattan distances between tiles and goal of the PuzzleBoard """
    n = len(board.board)

    h = 0
    for y in range(n):
        for x in range(n):
            if board.board[y][x] is not 0:
                y_, x_ = divmod(board.board[y][x], n)
                h += abs(y_- y) + abs(x_- x)
    return h


### 3b. Implement HllClimbingPuzzleSolver, which perform Hill-Climbing search
### with backtracking to a limited depth.

"""
The sort() method of lists may come in handy here. If you sort a list of tuples, 
it will sort by the value of the first element. 
"""

class HillClimbingPuzzleSolver:

    counts = {"enqueues":0, "extends":0}
    solution = AbstractState(None, None, 0)

    def __init__(self, initial_board, graph_search, heuristic_fn = manhattan, max_depth = INF):
        finished = False
        stack = []

        if graph_search: closed = set()
        initial_board = AbstractState(initial_board, None, 0)

        if not initial_board.is_goal():
            stack = [initial_board]
            self.counts["enqueues"] += 1
        else:
            solution = initial_board

        while stack != [] and not finished:
            curr = stack.pop()
            if graph_search:
                if curr in closed:
                    continue
                closed.add(curr)
            self.counts["extends"] += 1
            if self.counts["extends"] > max_depth:
                self.solution = None
                return


            if curr.is_goal():
                self.solution = curr
                finished = True
                return

            neighbors = curr.get_neighbors()
            neighbors.sort(key = lambda s: heuristic_fn(s.get_snapshot()), reverse = True)
            for neighbor in neighbors:
                if neighbor.notailbite():
                    stack.append(neighbor)
                    self.counts["enqueues"] += 1


    def num_moves(self):
        """ return number of moves in solution to initial board. If no solution found, return None."""
        try:
            return self.solution.get_path_length()
        except Exception as e:
            return None

    def num_enqueues(self) :
        """ return number of nodes enqueued during search, successful or not. """
        return self.counts["enqueues"]

    def num_extends(self) :
        """ return number of nodes extended/expanded during search, successful or not. """
        return self.counts["extends"]

    def get_solution(self) :
        """ returns sequence of boards, initial board to goal board. If no solution found, return None."""
        solution_list = []
        curr = copy.deepcopy(self.solution)
        while curr != None:
            solution_list.insert(0, curr.get_snapshot())
            curr = curr.get_parent()
        return tuple(solution_list)


""" Use hillclimbingchecker.py to check that your HillClimbingPuzzleSolver works properly.
Writing your own tests below is also a good idea.
"""

### 3c. Implement GreedyBestPuzzleSolver, which perform Greedy Best-first search
### using a given heuristic.

"""
You will want to become familiar with the heapq module, which is imported for you. 
It operates on lists and makes them work as priority queues.
Namely, the methods heapq.heapappend, heapq.heappop, and possibly heapq.nsmallest 
(which with n =1 works as a "peek" at the minimum without popping it) will be useful. 
https://docs.python.org/3/library/heapq.html
"""

class GreedyBestPuzzleSolver:

    counts = {"enqueues":0, "extends":0}
    solution = AbstractState(None, None, 0)

    def __init__(self, initial_board, graph_search = False, heuristic_fn = manhattan, max_depth = INF) :
        """find a solution to the initial puzzle board, up to max_depth, using Greedy Best-first search.
        heuristic_fn should be used to evaluate the order of states to extend. 
        If graph_search is True, avoid re-exploring paths. 
        """
        finished = False
        stack = []
        if graph_search: closed = set()
        initial_board = AbstractState(initial_board, None, 0)
        if not initial_board.is_goal():
            heappush(stack, (heuristic_fn(initial_board.get_snapshot()), initial_board))
            self.counts["enqueues"] += 1
        else:
            solution = initial_board

        while stack != [] and not finished:
            curr = heappop(stack)[1]
            if graph_search:
                if curr in closed:
                    continue
                closed.add(curr)
            self.counts["extends"] += 1
            if self.counts["extends"] > max_depth:
                self.solution = None
                return

            
            if curr.is_goal():
                self.solution = curr
                finished = True
                return

            for neighbor in curr.get_neighbors():
                if neighbor.notailbite():
                    heappush(stack, (heuristic_fn(neighbor.get_snapshot()), neighbor))
                    self.counts["enqueues"] += 1


    def num_moves(self):
        """ return number of moves in solution to initial board. If no solution found, return None."""
        try:
            return self.solution.get_path_length()
        except Exception as e:
            return None

    def num_enqueues(self) :
        """ return number of nodes enqueued during search, successful or not. """
        return self.counts["enqueues"]

    def num_extends(self) :
        """ return number of nodes extended/expanded during search, successful or not. """
        return self.counts["extends"]

    def get_solution(self) :
        """ returns sequence of boards, initial board to goal board. If no solution found, return None."""
        solution_list = []
        curr = copy.deepcopy(self.solution)
        while curr != None:
            solution_list.insert(0, curr.get_snapshot())
            curr = curr.get_parent()
        return tuple(solution_list)


""" Use greedybestchecker.py to check that your GreedyBestPuzzleSolver works properly.
Writing your own tests below is also a good idea.
"""



### Part 4: Optimal Search ###################################################


def zero_heuristic(board):
    """ A very unhelpful heuristic. Returns 0"""
    return 0

### 4a. Implement AStarPuzzleSolver, which perform A* Search
### using a given heuristic.

class AStarPuzzleSolver:

    counts = {"enqueues":0, "extends":0}
    solution = AbstractState(None, None, 0)

    def __init__(self, initial_board, graph_search = False, heuristic_fn = manhattan) :
        """find a OPTIMAL solution to the initial puzzle board, using A* search. 
        heuristic_fn should be used in the evaluation of order of states to extend.
        If graph_search is True, the algorithm should avoid enqueueing previously *extended* (not enqueued) states.
        Do not worry about updating already enqueued states; simply re-enqueue with the new values.
        """
        stack = []
        if graph_search: 
            closed = set()
        initial_board = AbstractState(initial_board, None, 0)

        if not initial_board.is_goal():
            heappush(stack, (heuristic_fn(initial_board.get_snapshot()) + initial_board.get_path_length(), initial_board))
            self.counts["enqueues"] += 1
        else:
            solution = initial_board
            return

        while True:
            curr = heappop(stack)[1]
            if graph_search:
                if curr in closed:
                    continue
                closed.add(curr)
            self.counts["extends"] += 1

            
            if curr.is_goal():
                self.solution = curr
                return

            for neighbor in curr.get_neighbors():
                if graph_search or neighbor.notailbite():
                    heappush(stack, (heuristic_fn(neighbor.get_snapshot()) + neighbor.get_path_length(), neighbor))
                    self.counts["enqueues"] += 1


    def num_moves(self):
        """ return number of moves in solution to initial board. If no solution found, return None."""
        try:
            return self.solution.get_path_length()
        except Exception as e:
            return None

    def num_enqueues(self) :
        """ return number of nodes enqueued during search, successful or not. """
        return self.counts["enqueues"]

    def num_extends(self) :
        """ return number of nodes extended/expanded during search, successful or not. """
        return self.counts["extends"]

    def get_solution(self) :
        """ returns sequence of boards, initial board to goal board. If no solution found, return None."""
        solution_list = []
        curr = copy.deepcopy(self.solution)
        while curr != None:
            solution_list.insert(0, curr.get_snapshot())
            curr = curr.get_parent()
        return tuple(solution_list)

""" Write tests to check that your AStarPuzzleSolver works properly """


### Part 5: Optional Extensions ########################################################

""" Try any of the following extensions for honors points! 
These are only ideas; feel free to suggest and pursue a different extension.
"""

""" A) Investigate a way to determine if a given puzzle is solvable without exhausting the whole state space.
 You may want to research the concept of parity. Implement is_solvable and make sure that it returns True on all the 
 solvable test boards while returning False on the unsolvable ones.
 """

def is_solvable(board) :
    """is this board solvable? return a boolean"""
    raise NotImplementedError


""" B) Implement beam search for a variable beam size k. Remember that beam search extends 
only the best k nodes at each level/depth of the tree. Although it is not a complete algorithm,
it can be useful for curbing memory usage while less prone to getting stuck than basic hill-climbing.
"""


""" C) Informed search algorithms can be used as "anytime" algorithms because,
if terminated early, they can provide a path to the state closest to the goal that its
seen so far - which, when on a time crunch, is much better than nothing! 
Implement and add the function get_closest_path to any/all of the informed solvers, 
then alter the algorithm so that if it does not find the solution within a 
given "time" (perhaps measured by enqueues/extends) , it returns a path
to the node with the lowest heuristic value it has seen yet.
"""
    # def get_closest_path(self) :
    #     """ If solution is found, returns the solution. Otherwise, returns a path to the 
    #     node with the lowest heuristic value found. """
    #     raise NotImplementedError


""" D) Implement a better heuristic function than hamming or manhattan. There is some literature
on this, but an original one would be extra impressive! Test any/all of the informed algorithms with it 
to see if it finds better solutions or optimal solutions faster than manhattan!
Remember, if being used in A*, the heuristic needs to be admissible/consistent to
guarantee optimality!
"""

def better_heuristic(board) :
    raise NotImplementedError


""" E) A* is optimal, but its memory usage is still prohibitive for large state spaces. 
Investiate and Implement either Recursive Best-First Search (RBFS), memory-bounded A* (MA*)
or simplified memory-bounded A* (SMA*). Be sure to comment it well to demonstrate you understand it!
"""
