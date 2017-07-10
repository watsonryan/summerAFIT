#!/usr/bin/env python 

'''
Simple script to test the sensitivity of pose-graph optimization to 
switch factors.
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
                                "switch factors")

parser.add_argument('-i', '--inputFile', dest='input', 
                   default='../../poseGraphs/manhattanOlson3500.g2o', 
                   help="Define the input pose graph")
parser.add_argument('-t', '--truePose', dest='true', 
                   default='', 
                   help="Define the true pose graph")
parser.add_argument('-o', '--outFile', dest='output', 
                    help="Define the output file")
parser.add_argument('-s', '--script', dest='script', 
                    default='./../../gtsam/build/examples/switchFactorG2O', 
                    help="What's the GTSAM script used to process the graph")
parser.add_argument('--Inc', dest='Inc', default=0.1, type=float,
                    help="What's the kernel increment for testing sensivity?")
parser.add_argument('--saveGraph',action='store_true', 
                    help="would you like to save the graph to the cur. dir.?")
args = parser.parse_args()

prior = []
init = []
totalError = []

progress = progressbar.ProgressBar()
print('\n\n')
for k in progress(list(xrange(1, int(1. /args.Inc)))):

    priorValue = str(float(k)*args.Inc)

    for j in list(xrange(1, int(1. / args.Inc))):

      initValue = str(float(j)*args.Inc)
      cmd = [args.script, '-i', args.input, '--initSwitch', initValue, '--priorSwitch', priorValue, '-t', args.true]
      proc1 = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
      out, err = proc1.communicate()
      prior.append( float(priorValue) )
      init.append( float(initValue) )    
      totalError.append( out )

#plt.plot(index, totalError, 'k',label=args.kernel, linewidth=3.0)
#plt.ylabel('Final Graph Error')
#plt.xlabel('Kernel Width')
#plt.grid()
#font = { 'size'  : 22}
#plt.rc('font', **font)
#plt.legend()
#plt.show()

if (args.output):
    f=open(args.output,'w')
    for line in zip(map(str,init), map(str,prior) ,totalError):
        f.write(' '.join(line)+'\n')
    f.close()
#if (args.saveGraph):
#    plt.savefig(args.kernel+".eps", format="eps", close=False, verbose=True)
