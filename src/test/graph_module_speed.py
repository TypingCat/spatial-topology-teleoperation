#!/usr/bin/env python3

import time
import matplotlib.pyplot as plt

import networkx
import neo4j
# import graphlib
# import graph-theory
# import graph-tool

def test_hash(num, sim):
    T = {}
    time_elapsed = []
    start = time.time()

    for n in range(num):
        t = {k: [2*n] for k in range(2*n, 2*n+10)}
        T.update(t)

        if sim:
            T_set = set(T.keys())
            t_set = set(t.keys())
            distance = len(T_set - t_set) + len(t_set - T_set) + sum([len(set(T[i]) - set(t[i])) for i in T_set & t_set])
            print(distance)

        time_elapsed.append(time.time() - start)

    return [b - a for a, b in zip(time_elapsed[:-1], time_elapsed[1:])]

def test_networkx(num, sim):
    T = networkx.Graph()
    time_elapsed = []
    start = time.time()

    for n in range(num):
        t = networkx.Graph()
        t.add_nodes_from(list(range(2*n, 2*n+10)))
        t.add_edges_from([(2*n, i) for i in range(2*n, 10)])
        T.add_nodes_from(t.nodes)
        T.add_edges_from(t.edges)

        if sim: print([v for v in networkx.optimize_graph_edit_distance(T, t)])

        time_elapsed.append(time.time() - start)

    return [b - a for a, b in zip(time_elapsed[:-1], time_elapsed[1:])]

def test_neo4j(num, sim=False):
    T = {}
    time_elapsed = []
    start = time.time()

    graphDB_Driver  = neo4j.GraphDatabase.driver(
        "bolt://localhost:7687",
        auth=("neo4j", "test"))

    for _ in range(num):
        with graphDB_Driver.session() as graphDB_Session:
            graphDB_Session.run("""CREATE
            (alice:Person {name: 'Alice'}),
            (bob:Person {name: 'Bob'}),
            (carol:Person {name: 'Carol'}),
            (dave:Person {name: 'Dave'}),
            (eve:Person {name: 'Eve'}),
            (guitar:Instrument {name: 'Guitar'}),
            (synth:Instrument {name: 'Synthesizer'}),
            (bongos:Instrument {name: 'Bongos'}),
            (trumpet:Instrument {name: 'Trumpet'}),
            (alice)-[:LIKES]->(guitar),
            (alice)-[:LIKES]->(synth),
            (alice)-[:LIKES {strength: 0.5}]->(bongos),
            (bob)-[:LIKES]->(guitar),
            (bob)-[:LIKES]->(synth),
            (carol)-[:LIKES]->(bongos),
            (dave)-[:LIKES]->(guitar),
            (dave)-[:LIKES]->(synth),
            (dave)-[:LIKES]->(bongos);""")

        # Skip similarity calculation

        time_elapsed.append(time.time() - start)

    return [b - a for a, b in zip(time_elapsed[:-1], time_elapsed[1:])]

if __name__ == '__main__':
    num = 500
    num_ignore = 30

    # Test graph modules
    time_hash = test_hash(num, False)
    time_networkx = test_networkx(num, False)
    time_neo4j = test_neo4j(num)
    time_hash_full = test_hash(num, True)
    time_networkx_full = test_networkx(num, True)

    # Visualize results
    for i, data in enumerate([time_hash, time_networkx, time_neo4j, time_hash_full, time_networkx_full], start=1):
        title = [k for k, v in locals().items() if v == data][0]
        plt.subplot(2, 3, i)
        plt.plot(data[num_ignore:])
        plt.title(title)
        if len(data) > 0: print(f'{title}: mean {sum(data)/len(data):.5f}, max {max(data):.5f}')
        if len(data) > num_ignore: print(f'{title}[{num_ignore}:]: mean {sum(data[num_ignore:])/len(data[num_ignore:]):.5f}, max {max(data[num_ignore:]):.5f}')
    plt.show()