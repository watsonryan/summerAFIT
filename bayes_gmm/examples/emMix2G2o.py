#!/usr/bin/env python

'''
Scrit to test pose graph optimization using Max-Mixtures robust noise model
with E.M. estimated mixture components.
'''

__author__ = 'ryan'
__email__ = "rwatso12@gmail.com"

import warnings
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys, os, glob, subprocess, progressbar, argparse

sys.path.append("..")

from bayes_gmm.niw import NIW
from bayes_gmm.igmm import IGMM

# Used to check if covariance is P.D. before feeding into Max-Mix.
def is_pos_def(x):
  return np.all(np.linalg.eigvals(x) > 0)

def main(argv):

  warnings.filterwarnings("ignore")

  # Add command line interface
  parser = argparse.ArgumentParser(description="Scrit to test pose graph  "
                                  "optimization using Max-Mixtures robust noise"
                                  "model with E.M. estimated mixture components")
  parser.add_argument('-i', '--inputFile', dest='input',
                     default='../../poseGraphs/manhattanOlson3500.g2o',
                     help="Define the input pose graph")
  parser.add_argument('-t', '--truePose', dest='true',
                     default='',
                     help="Define the true pose graph")
  parser.add_argument('-s', '--script', dest='script',
                    default='./../../gtsam/build/examples/processG2O',
                    help="What's the GTSAM script used to process the graph")
  parser.add_argument('-a', '--alpha', dest='alpha', default=1e-6, type=float,
                      help="What's the value of the Dirichlet hyper-parameter?")
  parser.add_argument('-n', '--iterations', dest='iterations', default=10, type=int,
                      help="How many iterations of IGMM will be used to calculate mixture model?")
  parser.add_argument('-k', '--init_comp', dest='initComp', default=2, type=int,
                      help="How many inital components in model?")
  parser.add_argument('-o','--output',dest='out',default='mixture.g2o', type=str,
                      help ="where would you like to save the mixture g2o file?")
  args = parser.parse_args()

  ##############################################################################
  # 1) Run L2 optimization over initial pose graph
  ##############################################################################

  residuals = []
  if args.true :
      cmd = [args.script, '-i', args.input, '-t', args.true]
  else :
      cmd = [args.script, '-i', args.input]

  proc1 = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
  residuals, err = proc1.communicate()

  ##############################################################################
  # 2) Use IGMM To Calculate Mixture Model
  ##############################################################################

  residuals = (map(float, residuals.split() ))
  residuals = np.reshape( residuals, (len(residuals)/3, 3) )
  np.savetxt( 'residuals.txt', residuals)
  D = min( residuals.shape )
  # Intialize prior
  m_0 = np.mean(residuals,axis=0)
  k_0 = 1.
  v_0 = D + 3
  covar_scale = 1.
#  v_0 = 3. * np.sqrt( np.std( residuals, axis=0 ) )
  S_0 = covar_scale**2*v_0*np.eye(D)
#  S_0 = np.array([ [0.02,0,0],[0,0.02,0],[0,0,0.01] ])
# S_0 = np.array([ [0.02,0,0],[0,0.02,0],[0,0,0.01] ] )
  prior = NIW(m_0, k_0, v_0, S_0)

  # Setup IGMM
  igmm = IGMM(residuals, prior, args.alpha, assignments="rand", K=args.initComp)

  # Perform Gibbs sampling
  record = igmm.gibbs_sample(args.iterations)

  mixture = np.array([]).reshape(0,9)
  for k in xrange(igmm.components.K):
    mu, sigma = igmm.components.rand_k(k)
    mixture =  np.vstack( [mixture, np.reshape( sigma, ( 1, sigma.size ) ) ] )
  mixture =  np.array(sorted(mixture,key=tuple))
  np.savetxt( 'mixtureModel.txt', mixture, delimiter=',' )

  mixInv = np.array([]).reshape(0,6)
  for i in range(0, mixture.shape[0]):
    mixTmp = np.linalg.inv( mixture[i].reshape(3,3)  )
    mixTmp = np.reshape( mixTmp, (1, mixTmp.size) )
    mixTmp = np.array([mixTmp[0][0],mixTmp[0][1],mixTmp[0][2],mixTmp[0][4],mixTmp[0][5],mixTmp[0][8]])
    mixInv =  np.vstack( [mixInv, mixTmp] )

  count = 0
  outKey = []
  residuals = np.loadtxt('residuals.txt')
  keys = np.loadtxt('factor.keys')
  for lines in residuals:
    rsos = np.inner( residuals[count], residuals[count] )
    if (rsos > 0.0):
      outKey.append(keys[count])
    count = count + 1
  outKey = [map(int, x) for x in outKey]

  ##############################################################################

  f = open(args.input,'r')
  g = open(args.out,'w')
  count = 0
  for lines in f:
    splitLine = lines.split()
    if splitLine[0] == "EDGE_SE2":
#      if outKey[count][0] == int(splitLine[1]) and outKey[count][1] == int(splitLine[2]):
      count = count + 1
      index = splitLine[1:3]
      meas = splitLine[3:6]
      splitLine[0] = splitLine[0]+"_MIXTURE"
      splitLine[3] = str(k+1)
      splitLine[4] = "EDGE_SE2"
      splitLine[5] = str(1)
      splitLine[6:7] = index
      splitLine[8:10] = meas
      splitLine[11:20] = map(str, mixInv[0])
      for x in range(1, k+1):
        splitLine.append("EDGE_SE2")
        splitLine.append( str( 1e-5*( 1. / np.linalg.norm( mixInv[0] -  mixInv[x] ) ) ) )
        splitLine.append( ' '.join(index) )
        splitLine.append( ' '.join(meas) )
        splitLine.append( ' '.join( map(str, 1e-5 * mixInv[x]) ) )
      g.write(' '.join(splitLine)+ '\n')
#      else:
#        splitLine[6:12] = map(str, mixInv[0])
#        g.write(' '.join(splitLine) + '\n')
    else:
      g.write(' '.join(splitLine) + '\n')
  f.close()
  g.close()

if __name__=="__main__":
  main(sys.argv[1:])
