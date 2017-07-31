#!/usr/bin/env python

'''
Simple script to test the sensitivity of pose-graph optimization to
inital measurement covariance
'''

__author__ = 'ryan'
__email__ = "rwatso12@gmail.com"


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os, glob, subprocess, progressbar, argparse

def getRMS( truePose, estPose ):
    print truePose
    print estPose
    f = file(truePose,'r')
    lines=f.readlines()
    mode = 2
    # build a dictionary of vertices and edges
    trueVertex=[]
    estVertex=[]

    for line in lines:
        if line.startswith('VERTEX_SE2'):
            idx=line.split()[2:]
            trueVertex.append(idx)

    f.close()
    f = file(estPose, 'r')
    lines=f.readlines()
    for line in lines:
        if line.startswith('VERTEX_SE2'):
            idx=line.split()[2:]
            estVertex.append(idx)

    f.close()

    rsos = []
    i = 0
    for n in estVertex:
        diff = np.asarray( map( float, trueVertex[i][1:3]) ) - np.asarray(map(float, estVertex[i][1:3]) )
        rsos.append( np.sqrt( np.inner(diff,diff) ) )
        i = i + 1
    return np.median(rsos)

# Add command line interface
parser = argparse.ArgumentParser(description="Simple script to test the "
                                "sensitivity of pose-graph optimization to"
                                "m-estimator kernel width")

parser.add_argument('-i', '--inputFile', dest='input',
                   default='../../poseGraphs/manhattanOlson3500.g2o',
                   help="Define the input pose graph")
parser.add_argument('-t', '--truePose', dest='true',
                   default='',
                   help="Define the true pose graph")
parser.add_argument('-o', '--outFile', dest='output',
                    help="Define the output file")
parser.add_argument('-s', '--script', dest='script',
                    default='g2o',
                    help="What's the GTSAM script used to process the graph")
parser.add_argument('--maxWidth', dest='maxWidth', default=5, type=int,
                    help="What's the maximum kernel width you would like to test?")
parser.add_argument('--kerInc', dest='kernelIncrement', default=0.1, type=float,
                    help="What's the kernel increment for testing sensivity?")
args = parser.parse_args()

index = []
totalError = []

progress = progressbar.ProgressBar()
print('\n\n')
for k in progress(list( np.arange(1,args.maxWidth,args.kernelIncrement))):

    # generate noisy graph
    cmd = ['./genNoisyGraph.py', '-i', args.input, '-s', str(k)]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    # run l2 optimization
    cmd = [args.script, '-o', 'output.g2o', 'noisy.g2o']
    proc1 = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    out, err = proc1.communicate()
    index.append( float(k) )
    totalError.append( getRMS( args.input, 'output.g2o' ) )

plt.plot(index, totalError, 'k', linewidth=3.0)
plt.ylabel('Final Graph Error')
plt.xlabel('Kernel Width')
plt.grid()
font = { 'size'  : 22}
plt.rc('font', **font)
plt.legend()
plt.show()
