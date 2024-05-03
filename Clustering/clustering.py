from sklearn.cluster import AgglomerativeClustering 
from sklearn import datasets
from scipy.cluster.hierarchy import dendrogram, linkage 
import matplotlib.pyplot as plt 
import numpy as np 

n_samples = 2000
noisy_circles = datasets.make_circles(n_samples=n_samples, factor=0.5, noise=0.2)

X = np.array(noisy_circles[0])

# randomly chosen dataset 
#X = np.array([[1, 2], [1, 4], [1, 0], [4, 2], [4, 4], [4, 0]]) 
#X = np.array([[0.8, 2], [0.5, 4], [2, 3], [1, 3], [1.5, 4.4], [5, 6], [5.4, 5], [7, 4], [6, 4], [6, 4.8], [6.5, 5.6]]) 
#X = np.array([[0.8, 2], [0.5, 4], [2, 3], [1, 3], [1.5, 4.4], [5, 6], [5.4, 5], [7, 4], [6, 4], [6, 4.8], [6.5, 5.6], [4, 4]])  # with noise

clustering = AgglomerativeClustering(n_clusters=2, linkage="single").fit(X) 

# print the class labels 
print(clustering.labels_) 

# Perform hierarchical clustering 
Z = linkage(X, 'single', 'euclidean', True) 
# Plot dendrogram 
dendrogram(Z) 

plt.title('Hierarchical Clustering Dendrogram') 
plt.xlabel('Data point') 
plt.ylabel('Distance') 
plt.show()
