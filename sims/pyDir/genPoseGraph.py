#!/usr/bin/env python

'''
This is a simple script to generate random pose-graphs with known noise covariance

for all options ::
    ./genPoseGraph.py -h
'''

__author__='Ryan Watson'
__email__='rwatso12@gmail.com'

import argparse
import numpy as np
import networkx as nx
from scipy import spatial
import matplotlib.pyplot as plt

def generate_clean_graph(nnodes,search_radius):
    positions =  np.random.rand(nnodes,2)
    pose = dict(zip(range(nnodes),positions))
    kdtree = spatial.KDTree(positions)
    pairs = kdtree.query_pairs(search_radius)
    graph = nx.Graph()
    graph.add_nodes_from(range(nnodes))
    graph.add_edges_from(list(pairs))
    max_graph = nx.algorithms.clique.make_max_clique_graph( graph )
    weak_nodes = [node for node,degree in max_graph.degree().items() if degree < 3]
    max_graph.remove_nodes_from(weak_nodes)
    for n in graph.nodes():
        graph.node[n]['pose'] = pose.values()[n]
    node_pose = nx.get_node_attributes(graph,'pose')
    return max_graph, node_pose

def generate_measurements(graph):
    #for n in graph.nodes():
    print graph.node

def print_g2o(graph,pose):
    for n in graph.nodes():
        graph.node[n]['pose'] = pose.values()[n]
    pose = nx.get_node_attributes(graph,'pose')
    print pose

def plot_graph(graph,pose):
    nx.draw(graph,pose)
    plt.show()

def generate_noisy_pose(pose,mu,var):
    pose = np.asarray(pose.values())
    nu = np.random.normal(mu, var, np.asarray(pose.shape))
    pose = pose + nu
    return dict(zip(range(np.shape(pose)[0]),pose))

if __name__=="__main__":

    # Add command line interface
    parser = argparse.ArgumentParser(description="Simple script to generate"
                                    "a random pose graph with known noise"
                                    "characteristics")
    parser.add_argument('-n', '--nnodes', dest='nnodes', default = 100,
                       help="Define the number of nodes in the graph")
    parser.add_argument('-r', '--radius', dest='radius', default = 0.15,
                       help="Define radius to find local nodes to connect")
    parser.add_argument('--noisyG2o', dest='g2oFile',
                        help="Define g2o data file name")
    parser.add_argument('--cleanGraph', dest='cleanG2o',
                        help="Define file name for clean g2o data file.")
    parser.add_argument('--plot', default=False, action="store_true",
                        help="Use flag if you would like to visualize the graph")
    args = parser.parse_args()

    clean_graph, clean_pose = generate_clean_graph(args.nnodes,args.radius)
    #noisy_pose = generate_noisy_pose(pose, [0,0], [0.1,0.25])
    print_g2o(clean_graph, clean_pose)
    #generate_measurements(clean_graph)
    if args.plot:
        plot_graph(clean_graph,clean_pose)
