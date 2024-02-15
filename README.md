# GeoPolyMesh Benchmark

> Ce projet se place dans une démarche d'optimisation du programme de thèse [GeoPolyMesh](https://github.com/AlexandreMarin/GeoPolyMesh) d'[Alexandre MARIN](https://github.com/AlexandreMarin).

## 📖 Présentation du projet 📖

GeoPolyMesh est un programme permettant de calculer le maillage polyédrique anisotropique d'un volume d'intérêt, dans le cadre de l'étude des sous-sols par l'[IFPEN](https://www.ifpenergiesnouvelles.fr/).

Pour contrôler certaines propriétés des celluless, il utilise une fonction potentiel qu'il convient de minimiser et doit, pour ce faire, calculer des cellules de Voronoï de façon fréquente.

Le problème est le suivant : **Le calcul est lent.** Environ 24h au total pour des cas tests.

Il faut donc l'optimiser afin de le rendre utilisable en situation réelle.

## 💻 Documentation du fichier 💻

Le fichier est composé de deux parties :
- Les déclarations de prédicats et structures de CGAL.
- L'espace de nom Benchmark.

### Déclarations inhérentes à CGAL


### L'espace de nom Benchmark

