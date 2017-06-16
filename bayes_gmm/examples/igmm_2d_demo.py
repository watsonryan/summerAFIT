#!/usr/bin/env python

"""
A basic demo of 2D generated data for illustrating the IGMM.

Author: Herman Kamper
Contact: kamperh@gmail.com
Date: 2013, 2014
"""

import logging
import matplotlib.pyplot as plt
import numpy as np
import random
import sys

sys.path.append("..")

from bayes_gmm.niw import NIW
from bayes_gmm.igmm import IGMM
from plot_utils import plot_ellipse, plot_mixture_model

logging.basicConfig(level=logging.INFO)

random.seed(1)
np.random.seed(1)


def main():

    # Data parameters
    D = 2           # dimensions
    K_true = 2      # the true number of components

    # Model parameters
    alpha = 1.
    K = 1           # initial number of components
    n_iter = 10

    # Generate data
    mu_scale = 1.
    covar_scale = 1.

    n_true = 100
    n_false = 100
    num_points = n_true+n_false

    # Generate random sample, two components
    np.random.seed(0)
    C = np.array([[0.1, 1.5], [0.3,2.3]])
    Data = np.r_[np.dot(np.random.randn(n_false, 2), C) + np.array([1,2]),
              .7 * np.random.randn(n_true, 2)]

    # Intialize prior
    m_0 = np.zeros(D)
    k_0 = covar_scale**2/mu_scale**2
    v_0 = D + 3
    S_0 = covar_scale**2*v_0*np.eye(D)
    prior = NIW(m_0, k_0, v_0, S_0)

    # Setup IGMM
#    igmm = IGMM(Data, prior, alpha, assignments="rand", K=K)
    igmm = IGMM(Data, prior, alpha, assignments="one-by-one", K=K)

    # Perform Gibbs sampling
    record = igmm.gibbs_sample(n_iter)

    # Plot results
    fig = plt.figure()
    ax = fig.add_subplot(111)
    plot_mixture_model(ax, igmm)
    for k in xrange(igmm.components.K):
        mu, sigma = igmm.components.rand_k(k)
        plot_ellipse(ax, mu, sigma*3)
    plt.xticks(())
    plt.yticks(())
    plt.show()




if __name__ == "__main__":
    main()
