import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os

l = []
with open('input.txt', 'r') as file:
    line = file.readline()

    while line:
      l.append(line.strip())
      line = file.readline()

adj = l[3:]

start,goal = tuple(map(int,l[1:3]))

graph = {}
adj2 = []

for i in adj:

  adj2.append(tuple(i.split()))

map_graph = {}
for i in adj2:
  v,j,w = i

  if v in map_graph:
    map_graph[v].append((w,j))
    continue

  map_graph[v] = [(w,j)]

def convert_graph(graph):
  d_graph = {}
  for node, neighbor in graph.items():
    d_graph[int(node)] = [(float(weight) , int(target)) for weight,target in neighbor]
    
  return d_graph

d = convert_graph(map_graph)

coords = []
with open("coords.txt") as file:
  
  for line in file:
    coords.append(tuple((line.strip().split())))

floating_coords = []
for item in coords:
  (x,y) = item 
  x = float(x)
  y = float(y)
  floating_coords.append((x,y))

coords_dict = dict(enumerate(floating_coords,1))

def heuristic(node,goal,e):

  return e*(((coords_dict[node][0] - coords_dict[goal][0])**2 + (coords_dict[node][1] - coords_dict[goal][1])**2)**0.5)


#ALGORITHM
def a_star(start, goal, graph,epsilon=0):

    e = epsilon
    minimum_cost = {}
    minimum_cost[start] = 0
    path = {node: None for node in graph}

    h_estimate = {node: float("inf") for node in graph}
    h_estimate[start] = heuristic(start, goal, e)

    visited_nodes = []
    visited_edges = []

    unvisited = list(graph.keys())

    while unvisited:

      current_node = min(unvisited, key= lambda node : h_estimate.get(node, float('inf')))


      if current_node is None:
            break


      visited_nodes.append(current_node)
      unvisited.remove(current_node)


      if current_node == goal:
            break

      for cost, neighbor in graph[current_node]:

        estimate = minimum_cost[current_node] + cost

        if neighbor not in minimum_cost or estimate < minimum_cost[current_node]:
          path[neighbor] = current_node
          minimum_cost[neighbor] = estimate
          h_estimate[neighbor] = minimum_cost[neighbor] + heuristic(neighbor, goal, e)
          visited_edges.append((current_node, neighbor))


    path_shortest = []
    current = goal
    while current is not None:
        path_shortest.append(current)
        current = path[current]
    path_shortest = path_shortest[::-1]

    minimum_cost_res = []

    for i,k in minimum_cost.items():
      if i in path_shortest:
        minimum_cost_res.append(k)

    return minimum_cost_res, path_shortest, visited_nodes, visited_edges

mc0,sp0,vn0,ve0 = a_star(start, goal , d, epsilon = 0)
mc1,sp1,vn1,ve1 = a_star(start, goal , d, epsilon = 1)
mc2,sp2,vn2,ve2 = a_star(start, goal , d, epsilon = 2)
mc3,sp3,vn3,ve3 = a_star(start, goal , d, epsilon = 3)
mc4,sp4,vn4,ve4 = a_star(start, goal , d, epsilon = 4)
mc5,sp5,vn5,ve5 = a_star(start, goal , d, epsilon = 5)

file_name = "output.txt"

directory = os.path.dirname(__file__)

file_path = os.path.join(directory,file_name)

string_shortest_path_1 = " ".join(str(item) for item in sp0)
min_cost_string_1 = " ".join(str(item) for item in mc0)
string_shortest_path_2 = " ".join(str(item) for item in sp1)
min_cost_string_2 = " ".join(str(item) for item in mc1)
string_shortest_path_3 = " ".join(str(item) for item in sp2)
min_cost_string_3 = " ".join(str(item) for item in mc2)
string_shortest_path_4 = " ".join(str(item) for item in sp3)
min_cost_string_4 = " ".join(str(item) for item in mc3)
string_shortest_path_5 = " ".join(str(item) for item in sp4)
min_cost_string_5 = " ".join(str(item) for item in mc4)
string_shortest_path_6 = " ".join(str(item) for item in sp5)
min_cost_string_6 = " ".join(str(item) for item in mc5)

with open(file_path, "w") as file:
   file.write(string_shortest_path_1 + '\n' + min_cost_string_1 + '\n')
   file.write(string_shortest_path_2 + '\n' + min_cost_string_2 + '\n')
   file.write(string_shortest_path_3 + '\n' + min_cost_string_3 + '\n')
   file.write(string_shortest_path_4 + '\n' + min_cost_string_4 + '\n')
   file.write(string_shortest_path_5 + '\n' + min_cost_string_5 + '\n')
   file.write(string_shortest_path_6 + '\n' + min_cost_string_6 + '\n')

file.close()



