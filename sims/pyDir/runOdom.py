#!/usr/bin/env python 

'''
Simple script to test the sensitivity of odo.
'''

__author__ = 'ryan'
__email__ = "rwatso12@gmail.com"


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os, glob, subprocess, progressbar, argparse


# Add command line interface
parser = argparse.ArgumentParser(description="Simple script to test the "
                                "sensitivity of odo.")

parser.add_argument('-n', '--numIter', dest='iterations', 
                   default=100, 
                   help="Define the number of iterations")
parser.add_argument('-s', '--script', dest='script', 
                    default='./../../gtsam/build/examples/OdometryExample', 
                    help="What's the GTSAM script used to process the graph")
parser.add_argument('-o', '--outFile', dest='output', 
                    help="Define the output file")
args = parser.parse_args()

index = []
pose = []

progress = progressbar.ProgressBar()
print('\n\n')
for k in progress(list(xrange(0, int(args.iterations)))):

    cmd = [args.script]

    proc1 = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = proc1.communicate()
    
    index.append(float(k))
    pose.append(' '.join(out.split()[-3:]) )
   
if (args.output):
    f=open(args.output,'w')
    for line in zip(map(str,index),pose):
        f.write(' '.join(line)+'\n')
    f.close()
