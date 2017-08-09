#!/usr/bin/env python

'''
Simple script to test the sensitivity of pose-graph optimization to
inital measurement covariance
'''

__author__ = 'Ryan Watson'
__email__ = "rwatso12@gmail.com"


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os, glob, subprocess, progressbar, argparse

def getRMS( truePose, estPose ):
    f1 = file(truePose,'r')
    lines=f1.readlines()
    mode = 2
    # build a dictionary of vertices and edges
    trueVertex=[]
    estVertex=[]

    for line in lines:
        if line.startswith('VERTEX_SE2'):
            idx=line.split()[2:]
            trueVertex.append(idx)

    f1.close()
    f2 = file(estPose, 'r')
    lines=f2.readlines()
    for line in lines:
        if line.startswith('VERTEX_SE2'):
            idx=line.split()[2:]
            estVertex.append(idx)

    f2.close()

    rsos = []
    i = 0
    for n in estVertex:
        diff = np.asarray( map( float, trueVertex[i][1:3]) ) - np.asarray(map(float, estVertex[i][1:3]) )
        rsos.append( np.sqrt( np.dot(diff,diff) ) )
        i = i + 1
    print np.median(rsos)
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
parser.add_argument('--maxWidth', dest='maxWidth', default=2, type=int,
                    help="What's the maximum kernel width you would like to test?")
parser.add_argument('--kerInc', dest='kernelIncrement', default=0.01, type=float,
                    help="What's the kernel increment for testing sensivity?")
args = parser.parse_args()

index = []
totalError = []

progress = progressbar.ProgressBar()
print('\n\n')
for k in progress(list( np.arange(0,args.maxWidth,args.kernelIncrement))):

    try:
        scale = repr(k)
        # generate noisy graph
        cmd = ['./genNoisyGraph.py', '-i', args.input, '-s', scale]
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out, err = proc1.communicate()

        # clean data file
#        vertex = "^VERTEX"
#        print vertex
#        cmd = ['grep', vertex, 'noisy.g2o', '>', 'vertex']
#        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
#        os.system('grep "^VERTEX" noisy.g2o > vertex;')
#        os.system('grep "^EDGE" noisy.g2o > edge;')
#        os.system("cat edge >> vertex; mv vertex noisy.g2o")
#        # run l2 optimization
        cmd2 = [args.script, '-o','output.g2o', '-i', '10 ', 'noisy.g2o']
        proc1 = subprocess.Popen(cmd2, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        out, err = proc1.communicate()
        if 'Warning' in err:
            print err, k
            pass

        index.append( float(k) )
        rms = getRMS( args.input, 'output.g2o' )
        totalError.append( rms )

    except(RuntimeError, TypeError, NameError):
        pass

print totalError

plt.plot(index, totalError, 'k', linewidth=3.0)
plt.ylabel('Median RSOS Error (m)')
plt.xlabel('Noise Scale Factor')
plt.grid()
font = { 'size'  : 22}
plt.rc('font', **font)
plt.legend()
plt.show()
