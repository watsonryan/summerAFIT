#!/usr/bin/env python 

'''
Simple script to test the sensitivity of pose-graph optimization to 
m-estimator kernel width.
'''

__author__ = 'ryan'
__email__ = "rwatso12@gmail.com"

import os, glob, subprocess, progressbar
import matplotlib.pyplot as plt


kernel = 'huber'
kernelWindow = 1000
outputFile = 'huberTest.txt'
inputFile = '../../poseGraphs/manhattanOlson3500.g2o'
script = './../../gtsam/build/examples/processG2O'

index = []
totalError = []

progress = progressbar.ProgressBar()
print('\n\n')
for k in progress(range(1, kernelWindow)):

    kernelWidth = str(float(k)/kernelWindow)
    cmd = ['./../../gtsam/build/examples/processG2O', '-i', inputFile, 
        '-k', kernel, '-w', kernelWidth]

    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = proc.communicate()
    index.append( float(kernelWidth) )    
    totalError.append( out )

plt.plot(index, totalError)
plt.show()
