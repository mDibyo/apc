import numpy as np
from stl import stl
import sys

import IPython

filename = sys.argv[1]
mesh = stl.StlMesh(filename)

# concatenate all vertices
verts = np.r_[mesh.v0, mesh.v1]
verts = np.r_[verts, mesh.v2]

# sort vertices to put identical vertices next to one another
order = np.lexsort(verts.T)
verts_remapped = verts[order]
diff = np.diff(verts_remapped, axis = 0)

# get indices of unique vertices (based on nonzero indices)
ui = np.ones(len(verts_remapped), 'bool')
ui[1:] = (diff != 0).any(axis = 1)
v = verts_remapped[ui]

# compute centroid and print
num_vertices = len(v)
centroid = (1.0 / num_vertices) * np.sum(v, axis = 0)
print 'Centroid = ', centroid

