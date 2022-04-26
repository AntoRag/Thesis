import sys
import math
import numpy
import matlab.engine


def main():
    eng = matlab.engine.start_matlab()
    s = eng.genpath('/home/antonio/Thesis/Matlab/Script') #add to Matlab path the scripts
    eng.addpath(s, nargout=0)
    eng.test(nargout=0) #test.m script to be launched
    input("Press Enter to continue...") #wait for Enter from keyboard to close the figures
    eng.quit()


if __name__ == '__main__':
    main()