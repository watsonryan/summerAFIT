#!/usr/bin/env python 

import itertools
import numpy as np
import matplotlib as mpl
from scipy import linalg
from sklearn import mixture
import matplotlib.pyplot as plt
from sklearn import cluster, datasets


color_iter = itertools.cycle(['navy', 'c', 'cornflowerblue', 'gold',
                              'darkorange'])

def plot_results(X, Y_, means, covariances, index, title):

    splot = plt.subplot(1, 2, 1 + index)
    for i, (mean, covar, color) in enumerate(zip(
            means, covariances, color_iter)):

        v, w = linalg.eigh(covar)
        v = 2. * np.sqrt(2.) * np.sqrt(v)
        u = w[0] / linalg.norm(w[0])
        # as the DP will not use every component it has access to
        # unless it needs it, we shouldn't plot the redundant
        # components.
        if not np.any(Y_ == i):
            continue
        plt.scatter(X[Y_ == i, 0], X[Y_ == i, 1], .8, color=color)

        # Plot an ellipse to show the Gaussian component
        angle = np.arctan(u[1] / u[0])
        angle = 180. * angle / np.pi  # convert to degrees
        ell = mpl.patches.Ellipse(mean, v[0], v[1], 180. + angle, color=color)
        ell.set_clip_box(splot.bbox)
        ell.set_alpha(0.5)
        splot.add_artist(ell)

    plt.xticks(())
    plt.yticks(())
    plt.title(title)


# Number of samples per component
n_true = 5000
n_false = 50
num_points = n_true+n_false

# Generate random sample, two components
np.random.seed(0)
C = np.array([[0.1, 1.5], [0.3,2.3]])
#X = np.r_[np.dot(np.random.randn(n_samples, 2), C),
#          .7 * np.random.randn(n_samples, 2) + np.array([-6, 3])]

X = np.r_[np.dot(np.random.randn(n_false, 2), C) + np.array([1,2]),
          .7 * np.random.randn(n_true, 2)]

X1 = np.r_[np.dot(np.random.randn(n_false, 2), C) + np.array([1,2])]
X2 = np.r_[.7 * np.random.randn(n_true, 2)]

plt.scatter(X1[:,0], X1[:,1], 1.5, 'navy')
plt.scatter(X2[:,0], X2[:,1], 1.5, 'c')
plt.xticks(())
plt.yticks(())
plt.title('True Distribution')


# Fit a Gaussian mixture with EM
#gmm = mixture.GaussianMixture(n_components=100, covariance_type='full').fit(X)
#plot_results(X, gmm.predict(X), gmm.means_, gmm.covariances_, 0,
#             'Gaussian Mixture')


# Fit a Dirichlet process Gaussian mixture
dpgmm = mixture.BayesianGaussianMixture( n_components=num_points, 
                                         covariance_type='full' ).fit(X)
plot_results(X, dpgmm.predict(X), dpgmm.means_, dpgmm.covariances_, 1,
             'Bayesian Gaussian Mixture with a Dirichlet process prior')


plt.show()

