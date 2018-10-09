from sys import argv
from time import perf_counter as timer
from lab1 import *
INF = float('inf')
"""
This program creates an initial board from each filename specified
on the command line (or a set) and runs HillClimbingPuzzleSolver on it.
"""
### Alter these flags to set parameters for search #########################

GRAPH_SEARCH = False

MAX_DEPTH = INF

### Change this flag to True to automatically set max_depth to the optimal 
### (that is, minimum) solution depth of the test puzzles when running 
### a puzzle set (not for individual command line puzzles)
AUTO_MAX_DEPTH = False

HEURISTIC_FN = hamming

### Set this flag to print out the full solution found to the puzzles
VERBOSE_SOLUTIONS = False
############################################################################

if len(argv) < 2:
    print("Please give either a puzzle set name or a list of individual puzzle files")
    exit()

PUZZLE_SETS = {"test_puzzle":51,"test_puzzle2x2":7,"test_puzzle3x3":31,"test_puzzle4x4":51}
num_puzzles = len(argv)
puzz_set = None
# if given a puzzle set, prepare to do that set
if argv[1] in PUZZLE_SETS:
    puzz_set = argv[1] 
    num_puzzles = PUZZLE_SETS[puzz_set]
    if puzz_set != "test_puzzle":
        puzz_set += "-"

# print header
print("{:25} {:>7s} {:>8s} {:>10} {:>10}".format("filename", "moves", "time", "enqueues", "extends"));
print("----------------------------------------------------------------");

# for each command-line argument, or member in a set
for i in range(1 if (puzz_set is None) else 0, num_puzzles):
    filename  =  argv[i] if (puzz_set is None) else ("{}{:02d}.txt".format(puzz_set,i))

    # read in the board specified in the filename
    tiles = []
    n = None
    with open(filename, 'r') as file :
        n = int(file.readline())
        for j in range(n):            
            row = [int(x) for x in file.readline().split()]
            tiles.append(row)
    board = PuzzleBoard(tiles)

    # run the solver 
    start = timer()
    if (puzz_set is not None) and AUTO_MAX_DEPTH :
        solver = HillClimbingPuzzleSolver(initial_board = board, graph_search = GRAPH_SEARCH, heuristic_fn = HEURISTIC_FN, max_depth = i)  
    else :
        solver = HillClimbingPuzzleSolver(initial_board = board, graph_search = GRAPH_SEARCH, heuristic_fn = HEURISTIC_FN, max_depth = MAX_DEPTH) 
    
    end = timer()
    elapsed_time = end - start
    # print results
    print("{:25} {:7d} {:8.4f} {:10d} {:10d}\n".format(filename, solver.num_moves(), elapsed_time, solver.num_enqueues(), solver.num_extends()));
    
    if VERBOSE_SOLUTIONS :
        for board in solver.get_solution():
            print(board)
