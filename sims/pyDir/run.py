#!/usr/bin/env python 

'''
Simple script to test the sensitivity of pose-graph optimization to 
m-estimator kernel width.
'''

__author__ = 'ryan'
__email__ = "rwatso12@gmail.com"


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os, glob, subprocess, progressbar, argparse


# Add command line interface
parser = argparse.ArgumentParser(description="Simple script to test the "
                                "sensitivity of pose-graph optimization to"   
                                "m-estimator kernel width")

parser.add_argument('-i', '--inputFile', dest='input', 
                   default='../../poseGraphs/manhattanOlson3500.g2o', 
                   help="Define the input pose graph")
parser.add_argument('-o', '--outFile', dest='output', 
                    help="Define the output file")
parser.add_argument('-s', '--script', dest='script', 
                    default='./../../gtsam/build/examples/processG2O', 
                    help="What's the GTSAM script used to process the graph")
parser.add_argument('-k', '--kernel', dest='kernel', default='huber',
                     help="define the kernel to be used")
parser.add_argument('--maxWidth', dest='maxWidth', default=3, type=int,
                    help="What's the maximum kernel width you would like to test?")
parser.add_argument('--kerInc', dest='kernelIncrement', default=0.1, type=float,
                    help="What's the kernel increment for testing sensivity?")
parser.add_argument('--saveGraph',action='store_true', 
                    help="would you like to save the graph to the cur. dir.?")
args = parser.parse_args()

index = []
totalError = []

progress = progressbar.ProgressBar()
print('\n\n')
for k in progress(list(xrange(1, int(args.maxWidth/args.kernelIncrement)))):

    kernelWidth = str(float(k)*args.kernelIncrement)
    cmd = [args.script, '-i', args.input, '-k', args.kernel, '-w', kernelWidth]

    proc1 = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = proc1.communicate()
    index.append( float(kernelWidth) )    
    totalError.append( out )

plt.plot(index, totalError, 'k',label=args.kernel, linewidth=3.0)
plt.ylabel('Final Graph Error')
plt.xlabel('Kernel Width')
plt.grid()
font = { 'size'  : 22}
plt.rc('font', **font)
plt.legend()
plt.show()

if (args.saveGraph):
    plt.savefig(args.kernel+".eps", format="eps", close=False, verbose=True)
if (args.output):
    f=open(args.output,'w')
    for line in zip(map(str,index),totalError):
        f.write(' '.join(line)+'\n')
    f.close()

