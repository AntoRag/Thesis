import sys
import math
import numpy
import matlab.engine

matlab_function='test' #name of the matlab script to be launched

def main():
    global matlab_function
    eng = matlab.engine.start_matlab()
    eng.matlab_function(nargout=0)


if __name__ == '__main__':
    main()