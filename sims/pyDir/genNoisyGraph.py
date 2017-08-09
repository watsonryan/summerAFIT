#!/usr/bin/env python

import argparse
import numpy as np

# take from Niko Suenderhauf's script to add faults to pose graphs
def read_graph(graph, edgeStr='EDGE_SE2', vertexStr='VERTEX_SE2'):
    # read the complete file
    f = file(graph,'r')
    lines=f.readlines()

    mode = 2

    # build a dictionary of vertices and edges
    v=[]
    e=[]

    for line in lines:
        if line.startswith(vertexStr):
            idx=line.split()[1]
            v.append(line)

        elif line.startswith(edgeStr):
            idx=(line.split()[1],line.split()[2])
            e.append(line)

    return (v,e, mode)

def write_graph(filename, vertices, edges, mode, scale, outliers=0, informationMatrix="50,0,0,50,0,100"):

    mean_0 = np.array( [0,0,0] )
    sigma_0 = np.array( [ [0.2, 0.0, 0.0], [0.0, 0.2, 0.2], [0.0, 0.0, 0.1] ] )
    sigma = sigma_0 * scale
    # first write out all pose vertices (no need to change them)
    f = file(filename, 'w')
    for n in vertices:
        f.write(n)

    edgeStr='EDGE_SE2'

    # check entries for information matrix for additional loop closure constraints (outliers)
    if not informationMatrix:
        print "Determining information matrix automatically..."
    else:
        if informationMatrix.count(",")==0:
            # if there is only a single value, convert it into full upper triangular form with that value on the diagonal
            try:
                diagEntry=float(informationMatrix)
            except:
                print "! Invalid value for information matrix. If you give only a single value, it must be a number, e.g. --information=42"
                return False
            informationMatrix = "%f,0,0,%f,0,%f" % (diagEntry,diagEntry,diagEntry)

        elif informationMatrix.count(",")!=5:
            print "! Invalid number of entries in information matrix. Full upper triangular form has to be given, e.g. --information=42,0,0,42,0,42."
            return False
    poseCount = len(vertices)

    # for every edge, create a new switch node and its associated prior and write the new switchable edges
    for oldStr in edges:
        noisyMeas = np.array( map(float, oldStr.split()[3:6]) ) + np.random.multivariate_normal(mean_0,sigma,1)
        noisy_arrstr = list(np.char.mod('%f', noisyMeas)[0])
        noisy_str = ",".join(noisy_arrstr)
        oldStr = oldStr.split()
        oldStr[3:6] = noisy_str.split(',')
        f.write(' '.join(map(str, oldStr)) + '\n')

    return True

def add_noise(graph):
    print graph

if __name__=="__main__":
    # Add command line interface
    parser = argparse.ArgumentParser(description="Simple script to generate"
                                    "noisy pose graph")
    parser.add_argument('-i', '--input', dest='input',
                       help="Define the input graph")
    parser.add_argument('-o', '--output', dest='output', default='noisy.g2o',
                       help="Define the file that you would like to write the graph to.")
    parser.add_argument('-s', '--scale', dest='scale', type=float,
                        help="Information scale factor")

    args = parser.parse_args()
    verticies, edge, mode = read_graph(args.input)
    write_graph(args.output, verticies, edge, mode, args.scale)
