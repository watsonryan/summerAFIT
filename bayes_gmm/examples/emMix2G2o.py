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
      print '\n'*3
      print ' '.join(cmd)
  else :
      cmd = [args.script, '-i', args.input]
      print '\n'*3 
      print ' '.join(cmd)

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
  mu_scale = 1.
  covar_scale = 1.
  m_0 = np.zeros(D)
  k_0 = covar_scale**2/mu_scale**2
  v_0 = D + 3
  S_0 = covar_scale**2*v_0*np.eye(D)
  prior = NIW(m_0, k_0, v_0, S_0)

  # Setup IGMM
  igmm = IGMM(residuals, prior, args.alpha, assignments="rand", K=args.initComp)

  # Perform Gibbs sampling
  record = igmm.gibbs_sample(args.iterations)

  mixture = np.array([]).reshape(0,9)
  for k in xrange(igmm.components.K):
    mu, sigma = igmm.components.rand_k(k)
    mixture =  np.vstack( [mixture, np.reshape( sigma, ( 1, sigma.size ) ) ] )
  mixture = np.array(sorted(mixture,key=tuple))
  mixture = 3. * mixture
  np.savetxt( 'mixtureModel.txt', mixture, delimiter=',' )

  mixInv = np.array([]).reshape(0,9)
  for i in range(0, mixture.shape[0]):
    mixTmp = np.linalg.inv( mixture[i].reshape(3,3)  )
    mixInv =  np.vstack( [mixInv, np.reshape( mixTmp, ( 1, mixTmp.size ) ) ] )    


  ##############################################################################
  # 3) Write line for G2O mixture model 
  ##############################################################################

  f = open(args.input,'r')
  g = open(args.out,'w') 
  for lines in f:
    splitLine = lines.split()
    if splitLine[0] == "EDGE_SE2":
      index = splitLine[1:3]
      splitLine[0] = splitLine[0]+"_MIXTURE"
      splitLine[3] = str(k+1)
      splitLine[4] = "EDGE_SE2"s
      splitLine[5] = str(1)
      splitLine[6:7] = index
      splitLine[8:16] = map(str, mixInv[0]) 
      for x in range(1, k+1):
        splitLine.append("EDGE_SE2")
        splitLine.append( str( 1. / np.linalg.norm( mixture[0] - mixture[x] ) ) )
        splitLine.append( ' '.join(index) )
        splitLine.append( ' '.join( map(str, mixInv[x]) ) )
      g.write(' '.join(splitLine)+ '\n')
    else:
      g.write(' '.join(splitLine) + '\n') 

if __name__=="__main__":
  main(sys.argv[1:])
