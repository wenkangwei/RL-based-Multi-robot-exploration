import numpy as np
import matplotlib.pyplot as plt
import scipy.stats
from sklearn import mixture


# Test for Gaussian mixture model for reward function/map approximation
#
n_samples = 100
C = np.array([[0.8, -0.1], [0.2, 0.4]])
#
# X = np.r_[np.dot(np.random.randn(n_samples, 2), C),
#          np.random.randn(n_samples, 2) + np.array([-2, 1]),
#          np.random.randn(n_samples, 2) + np.array([1, -3])]
X = np.array([
    [np.random.randint(-10,10), np.random.randint(-10,10)]
    for i in range(50)
])
gmm = mixture.GaussianMixture(n_components=4, covariance_type='full').fit(X)

print(X.shape)
# print("X:")
# print(X)
# plt.scatter(X[:,0], X[:, 1], s = 1)
# print(gmm.covariances_)
# print(gmm.means_)
centers = np.empty(shape=(gmm.n_components, X.shape[1]))
for i in range(gmm.n_components):
    density = scipy.stats.multivariate_normal(cov=gmm.covariances_[i], mean=gmm.means_[i]).logpdf(X)
    centers[i, :] = X[np.argmax(density)]



# plt.scatter(centers[:, 0], centers[:, 1], s=20)
# plt.scatter(gmm.means_[:, 0], gmm.means_[:, 1], s=20)
x= [[1,1],[2,-1]]
for xi in X:
    x.append(xi)
    GMM = mixture.GaussianMixture(n_components=3, covariance_type='full')
    GMM.fit(x)
    plt.scatter(x[:][0], x[:][1], s=1)
    print(GMM.means_[0, 0],GMM.means_[0, 1])
    print(GMM.predict_proba(GMM.means_))
    plt.show()