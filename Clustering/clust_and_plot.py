from sklearn.cluster import AgglomerativeClustering 
from sklearn import datasets
from scipy.cluster.hierarchy import dendrogram, linkage 
import matplotlib.pyplot as plt 
import numpy as np 

def plota(X, Z, clustering):
    # ARREGLO DE COLORES PARA PLOT DE CLUSTERS
    colores = np.array(['green', 'red', 'blue', 'orange', 'purple', 'yellow', 'black']) 
    
    fig, (ax1, ax2) = plt.subplots(1,2)
    
    # PLOT DE LOS CLUSTERS EN ESPACIO EUCLIDEO
    ax1.scatter(X[:, 0], X[:, 1], c=colores[clustering.labels_])
    ax1.plot(X[:, 0], X[:, 1], marker='o', linestyle='', alpha=0.1)
    ax1.set(xlabel='Característica x', ylabel='Característica y')
    ax1.set_title('Plot de clusters')
    
    # DENDROGRAMA DE LOS DATOS
    dendrogram(Z, leaf_font_size=7, no_labels=True) #color_threshold=0, above_threshold_color='#15A59E'
    ax2.set(xlabel='Datos', ylabel='Distancia')
    ax2.set_title('Dendrograma')
    
    plt.show()

# ESCOGE METODO DE LINKAGE: 
Link = ['single', 'ward', 'complete']
met = Link[1]

# NUMERO DE SET DE DATOS
n_samples = 20

# CREAR UN SET DE DATOS CON FUNCION DE TOY DATASETS
noisy_circles = datasets.make_circles(n_samples=n_samples, factor=0.5, noise=0.2)
X = np.array(noisy_circles[0])

# FUNCION PARA OBTENER LOS LABELS DE LOS CLUSTERS CREADOS
clustering = AgglomerativeClustering(n_clusters=2, linkage=met).fit(X)  

# AGGLOMERATIVE HIERACHICAL CLUSTERING
Z = linkage(X, met, 'euclidean', True) 

plota(X,Z,clustering)
