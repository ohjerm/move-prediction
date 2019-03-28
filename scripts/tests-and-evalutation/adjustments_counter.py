import sys
import os

if(len(sys.argv) != 2):
    print("please provide an argument corresponding to the name of the file")
    
file_name = sys.argv[1]

path = os.path.join(os.path.dirname(os.path.realpath('__file__')), 'logs/' + file_name + '.txt')

with open(path) as _file:
    for line in _file:
        if len(line) > 0:
            l = list(line)
            adjustments = 0
            curr = l[0]
            for c in l:
                if c != curr and c != '0':
                    adjustments += 1
                    curr = c
            if adjustments > 0:
                print(adjustments)