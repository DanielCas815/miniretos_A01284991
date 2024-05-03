from sklearn.cluster import AgglomerativeClustering
from sklearn import datasets
import matplotlib.pyplot as plt 
import numpy as np 

# NUMERO DE CLUSTERS
numcl = 3
tip = 2

# TIPO DE LINKAGE {'ward', 'complete', 'average', 'single'}
link = 'ward'

# GENERAR PUNTOS CON TOY DATASET
n_samples = 1000

if(tip == 0):
    circ = datasets.make_circles(n_samples=n_samples, factor=0.5, noise=0.05)
    X = np.array(circ[0])
elif(tip == 1):
    blobs = datasets.make_blobs(n_samples=n_samples, random_state=20)
    X = np.array(blobs[0])
else:
    friedman = datasets.make_blobs(n_samples=n_samples, cluster_std=[1.0, 2.5, 0.5], random_state=170)
    X = np.array(friedman[0])

# DATOS DE PRUEBA
#X = np.array([[1, 2], [1, 4], [1, 0], [4, 2], [4, 4], [4, 0]])
#X = np.array([[0.8, 2], [0.5, 4], [2, 3], [1, 3], [1.5, 4.4], [5, 6], [5.4, 5], [7, 4], [6, 4], [6, 4.8], [6.5, 5.6], [4, 4], [2,20]]) 

# AGGLOMERATIVE HIERARCHICAL CLUSTERING
clustering = AgglomerativeClustering(n_clusters=numcl, linkage=link).fit(X)

# COLORES PARA EL CLUSTER PLOT
colors = np.array(['green', 'red', 'blue', 'orange', 'purple', 'yellow', 'black'])  

# PLOTEAR PUNTOS CON COLOR DE CLUSTER CORRESPONDIENTE
plt.scatter(X[:, 0], X[:, 1], c=colors[clustering.labels_])
plt.plot(X[:, 0], X[:, 1], marker='o', linestyle='', alpha=0.1) 
plt.xlabel('Característica x')
plt.ylabel('Característica y')
plt.title('Clusters de datos con método ' + link)
plt.show()