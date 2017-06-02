#!/usr/bin/env python 

'''
Simple script to test the sensitivity of pose-graph optimization to 
m-estimator kernel width.
'''

__author__ = 'ryan'
__email__ = "rwatso12@gmail.com"

import os, glob, subprocess, progressbar
import matplotlib.pyplot as plt


kernel = ['huber','tukey']
maxWidth = 2
kernelIncrement = 0.01
outputFile = 'huberTest.txt'
inputFile = '../../poseGraphs/intel.g2o'
script = './../../gtsam/build/examples/processG2O'

index = []
totalError1 = []
totalError2 = []

progress = progressbar.ProgressBar()
print('\n\n')
for k in progress(range(1, int(maxWidth/kernelIncrement))):

    kernelWidth = str(float(k)*kernelIncrement)
    cmd = ['./../../gtsam/build/examples/processG2O', '-i', inputFile, 
        '-k', kernel[0], '-w', kernelWidth]

    cmd2 = ['./../../gtsam/build/examples/processG2O', '-i', inputFile, 
        '-k', kernel[1], '-w', kernelWidth]

    proc1 = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    proc2 = subprocess.Popen(cmd2, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out1, err = proc1.communicate()
    out2, err = proc2.communicate()
    index.append( float(kernelWidth) )    
    totalError1.append( out1 )
    totalError2.append( out2 )

print totalError1
print totalError2
plt.plot(index, totalError1, 'k',label='Huber', linewidth=3.0)
plt.plot( index, totalError2, 'b', label='Tukey', linewidth=3.0)
plt.ylabel('Final Graph Error')
plt.xlabel('Kernel Width')
plt.grid()
plt.autoscale(tight=True)
font = { 'weight' : 'bold', 'size'   : 22}
plt.rc('font', **font)
plt.legend()
plt.show()
