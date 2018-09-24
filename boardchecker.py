from sys import argv
from lab1 import *

"""
This program creates an initial board from each filename specified
on the command line and outputs various checks.
"""

### You can turn this test on when you've implemented hamming and manhattan
TEST_HEURISTICS = False
###########################################################################

# for each command-line argument

for i in range(1,len(argv)):
    filename = argv[i]
    # read in the board specified in the filename
    tiles = []
    n = None
    with open(filename, 'r') as file:
        n = int(file.readline())
        for i in range(n):            
            row = [int(x) for x in file.readline().split()]
            tiles.append(row)
    board = PuzzleBoard(tiles)

    # show initial board
    print("{}'s initial board:\n{}".format(filename,str(board)))
    
    # is it the goal board?
    print("This is the goal!")

    if not board.is_goal():
        print("This is NOT the goal!")

    # show all neighbors
    print("Neighbors of the initial board:\n")
    for neighbor in board.get_neighbors():
        print(str(neighbor))

    # print heuristic values
    if(TEST_HEURISTICS):
        print("Hamming distance is:   {:3d}".format(hamming(board)))
        print("Manhattan distance is: {:3d}".format(manhattan(board)))


