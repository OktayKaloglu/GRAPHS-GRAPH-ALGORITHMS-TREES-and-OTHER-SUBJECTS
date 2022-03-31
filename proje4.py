

import networkx as nx
import matplotlib.pyplot as plt

G = nx.DiGraph()
list_nodes = [0, 1, 2, 3, 4]
G.add_nodes_from(list_nodes)
G.nodes()
list_arcs = [(0, 1, 5.0), (0, 4, 2.0), (0, 2, 3.0), (4, 1, 6.0), (4, 2, 10.0), (4, 3, 10.0), (2, 3, 2.0), (2, 1, 1.0),(1, 2, 2.0), (1, 3, 6.0)]
G.add_weighted_edges_from(list_arcs)
G.edges()
G.nodes[0]['pos'] = (0, 0)
G.nodes[1]['pos'] = (0, 2)
G.nodes[2]['pos'] = (2, 0)
G.nodes[3]['pos'] = (4, 1)
G.nodes[4]['pos'] = (2, 2)
node_pos = nx.get_node_attributes(G, 'pos')
arc_weight = nx.get_edge_attributes(G, 'weight')

node_col = ['white' if not node in G else 'red' for node in G.nodes()]
edge_col = ['black' if not edge in G else 'red' for edge in G.edges()]
nx.draw_networkx(G, node_pos, node_color=node_col, node_size=450)
nx.draw_networkx_edge_labels(G, node_pos, edge_labels=arc_weight)
plt.axis('off')
plt.show()


for i in (1,2,3):

    sp = nx.dijkstra_path(G, source=4, target=i)
    print(sp)
    G.nodes[0]['pos'] = (0, 0)
    G.nodes[1]['pos'] = (0, 2)
    G.nodes[2]['pos'] = (2, 0)
    G.nodes[3]['pos'] = (4, 1)
    G.nodes[4]['pos'] = (2, 2)
    node_pos = nx.get_node_attributes(G, 'pos')
    arc_weight = nx.get_edge_attributes(G, 'weight')
    red_edges = list(zip(sp, sp[1:]))


    node_col = ['grey' if not node in sp else 'red' for node in G.nodes()]
    edge_col = ['black' if not edge in red_edges else 'red' for edge in G.edges()]
    nx.draw_networkx(G, node_pos, node_color=node_col, node_size=450)
    nx.draw_networkx_edges(G, node_pos, edge_color=edge_col)
    nx.draw_networkx_edge_labels(G, node_pos, edge_labels=arc_weight)
    plt.axis('off')
    plt.show()


G.remove_node(3)


plt.show()

G.nodes[0]['pos'] = (0, 0)
G.nodes[1]['pos'] = (0, 2)
G.nodes[2]['pos'] = (2, 0)

G.nodes[4]['pos'] = (2, 2)
node_pos = nx.get_node_attributes(G, 'pos')
arc_weight = nx.get_edge_attributes(G, 'weight')

node_col = ['white' if not node in G else 'red' for node in G.nodes()]
edge_col = ['black' if not edge in G else 'red' for edge in G.edges()]
nx.draw_networkx(G, node_pos, node_color=node_col, node_size=450)
nx.draw_networkx_edge_labels(G, node_pos, edge_labels=arc_weight)
plt.axis('off')
plt.show()


for i in (1,2):

    sp = nx.dijkstra_path(G, source=4, target=i)
    print(sp)
    G.nodes[0]['pos'] = (0, 0)
    G.nodes[1]['pos'] = (0, 2)
    G.nodes[2]['pos'] = (2, 0)

    G.nodes[4]['pos'] = (2, 2)
    node_pos = nx.get_node_attributes(G, 'pos')
    arc_weight = nx.get_edge_attributes(G, 'weight')
    red_edges = list(zip(sp, sp[1:]))
    node_col = ['grey' if not node in sp else 'red' for node in G.nodes()]
    edge_col = ['black' if not edge in red_edges else 'red' for edge in G.edges()]
    nx.draw_networkx(G, node_pos, node_color=node_col, node_size=450)
    nx.draw_networkx_edges(G, node_pos, edge_color=edge_col)
    nx.draw_networkx_edge_labels(G, node_pos, edge_labels=arc_weight)
    plt.axis('off')
    plt.show()






G.clear()