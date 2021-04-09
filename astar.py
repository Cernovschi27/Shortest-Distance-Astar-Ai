# This class represent a graph
class Graph:
    # Initialize the class
    def __init__(self, graph_dict=None, directed=True):
        self.graph_dict = graph_dict or {}
        self.directed = directed
        if not directed:
            self.make_undirected()

    # Create an undirected graph by adding symmetric edges
    def make_undirected(self):
        for a in list(self.graph_dict.keys()):
            for (b, dist) in self.graph_dict[a].items():
                self.graph_dict.setdefault(b, {})[a] = dist

    # Add a link from A and B of given distance, and also add the inverse link if the graph is undirected
    def connect(self, A, B, distance=1):
        self.graph_dict.setdefault(A, {})[B] = distance
        if not self.directed:
            self.graph_dict.setdefault(B, {})[A] = distance
    def dict(self):
        return self.graph_dict
    # Get neighbors or a neighbor
    def get(self, a, b=None):
        links = self.graph_dict.setdefault(a, {})
        if b is None:
            return links
        else:
            return links.get(b)

    # Return a list of nodes in the graph
    def nodes(self):
        s1 = set([k for k in self.graph_dict.keys()])
        s2 = set([k2 for v in self.graph_dict.values() for k2, v2 in v.items()])
        nodes = s1.union(s2)
        return list(nodes)


# This class represent a node
class Node:
    # Initialize the class
    def __init__(self, name: str, parent: str):
        self.name = name
        self.parent = parent
        self.g = 0  # Distance to start node
        self.h = 0  # Distance to goal node
        self.f = 0  # Total cost

    # Compare nodes
    def __eq__(self, other):
        return self.name == other.name

    # Sort nodes
    def __lt__(self, other):
        return self.f < other.f

    # Print node
    def __repr__(self):
        return ('({0},{1})'.format(self.name, self.f))


# A* search
def astar_search(graph, heuristics, start, end):
    # Create lists for open nodes and closed nodes
    open = []
    closed = []
    # Create a start node and an goal node
    start_node = Node(start, None)
    goal_node = Node(end, None)
    # Add the start node
    open.append(start_node)

    # Loop until the open list is empty
    while len(open) > 0:
        # Sort the open list to get the node with the lowest cost first
        open.sort()
        # Get the node with the lowest cost
        current_node = open.pop(0)
        # Add the current node to the closed list
        closed.append(current_node)

        # Check if we have reached the goal, return the path
        if current_node == goal_node:
            path = []
            while current_node != start_node:
                path.append(current_node.name + ': ' + str(current_node.g))
                current_node = current_node.parent
            path.append(start_node.name + ': ' + str(start_node.g))
            # Return reversed path
            return path[::-1]
        # Get neighbours
        neighbors = graph.get(current_node.name)
        # Loop neighbors
        for key, value in neighbors.items():
            # Create a neighbor node
            neighbor = Node(key, current_node)
            # Check if the neighbor is in the closed list
            if (neighbor in closed):
                continue
            # Calculate full path cost
            neighbor.g = current_node.g + graph.get(current_node.name, neighbor.name)
            neighbor.h = heuristics.get(neighbor.name)
            neighbor.f = neighbor.g + neighbor.h
            # Check if neighbor is in open list and if it has a lower f value
            if (add_to_open(open, neighbor) == True):
                # Everything is green, add neighbor to open list
                open.append(neighbor)
    # Return None, no path is found
    return None


# Check if a neighbor should be added to open list
def add_to_open(open, neighbor):
    for node in open:
        if (neighbor == node and neighbor.f > node.f):
            return False
    return True

def basic_greedy(start,end):
    # greedy search algorithm
    #am incercat eu ceva
    d_dict = {'Oslo': [('Helsinki', 970),
                       ('Stockholm', 570)],
              'Helsinki': [('Stockholm', 400),('Oslo',970)],
              'Stockholm': [('Copenhagen', 522),('Oslo',570),('Helsinki',400)],
              'Copenhagen': [('Warsaw', 668),('Stockholm',522),('Berlin',743)],
              'Warsaw': [('Bucharest', 946),('Copenhagen',668)],
              'Bucharest': [('Athens', 1300),('Warsaw',946),('Budapest',900)],
              'Athens':[('Bucharest',1300),('Palermo',907)],
              'Budapest': [('Bucharest', 900), ('Belgrade', 316), ('Prague', 443),('Milan',789),('Vienna',217)],
              'Belgrade': [('Sofia', 330),('Budapest',316)],
              'Sofia':[('Belgrade',330)],
              'Rome': [('Palermo', 1043), ('Milan', 681),('Barcelona',1471)],
              'Palermo': [('Athens', 907),('Rome',1043)],
              'Milan': [('Budapest', 789),('Rome',681)],
              'Vienna': [('Budapest', 217), ('Munich', 458),('Prague',312)],
              'Prague': [('Vienna', 312), ('Berlin', 354),('Budapest',443)],
              'Berlin': [('Copenhagen', 743), ('Amsterdam', 648),('Prague',354)],
              'Amsterdam':[('Berlin',648),('Glasgow',711)],
              'Munich': [('Lyon', 753),('Vienna',458)],
              'Lyon': [('Paris', 481), ('Bordeaux', 542),('Munich',753),('Barcelona',644)],
              'Bordeaux':[('Lyon',542),('Paris',579)],
              'Madrid': [('Barcelona', 628), ('Lisbon', 638)],
              'Lisbon': [('London', 2210),('Madrid',638)],
              'Barcelona': [('Lyon', 644), ('Rome', 1471),('Madrid',628)],
              'Paris': [('London', 414), ('Bordeaux', 579),('Lyon',481)],
              'London': [('Dublin', 463), ('Glasgow', 667),('Lisbon',2210),('Paris',414)],
              'Glasgow': [('Amsterdam', 711), ('Dublin', 306),('London',667)],
              'Dublin':[('London',463),('Glasgow',306)]
              }
    # dict of lists of tuples such that nodei : [ (neighbourj, distancej), .... ]
    currentCity = start
    tour = []   # list of covered nodes
    tour.append(currentCity)
    distanceTravelled = 0   # distance travelled in tour
    while currentCity!=end:
        minDistanceNeighbour = None
        minDistance = None
        for eachNeighbour, eachNeighbourdDistance in d_dict[currentCity]:
            if eachNeighbour != currentCity and eachNeighbour not in tour:
                if minDistance is not None:
                    if minDistance > eachNeighbourdDistance:
                        minDistance = eachNeighbourdDistance
                        minDistanceNeighbour = eachNeighbour
                else:
                    minDistance = eachNeighbourdDistance
                    minDistanceNeighbour = eachNeighbour
        nearestNeigbhourCity = (minDistanceNeighbour, minDistance)
        tour.append(nearestNeigbhourCity[0])
        currentCity = nearestNeigbhourCity[0]
        distanceTravelled += nearestNeigbhourCity[1]
    print(tour)
    print(distanceTravelled)
# The main entry point for this module
def main():
    # Create a graph
    graph = Graph()

    # Create graph connections (Actual distance)
    graph.connect('Oslo','Helsinki',970)
    graph.connect('Helsinki','Stockholm',400)
    graph.connect('Oslo','Stockholm',570)
    graph.connect('Stockholm','Copenhagen',522)
    graph.connect('Copenhagen','Warsaw',668)
    graph.connect('Warsaw','Bucharest',946)
    graph.connect('Bucharest','Athens',1300)
    graph.connect('Budapest','Bucharest',900)
    graph.connect('Budapest','Belgrade',316)
    graph.connect('Belgrade','Sofia',330)
    graph.connect('Rome','Palermo',1043)
    graph.connect('Palermo','Athens',907)
    graph.connect('Rome','Milan',681)
    graph.connect('Milan','Budapest',789)
    graph.connect('Vienna','Budapest',217)
    graph.connect('Vienna','Munich',458)
    graph.connect('Prague','Vienna',312)
    graph.connect('Prague','Berlin',354)
    graph.connect('Berlin','Copenhagen',743)
    graph.connect('Berlin','Amsterdam',648)
    graph.connect('Munich','Lyon',753)
    graph.connect('Lyon','Paris',481)
    graph.connect('Lyon','Bordeaux',542)
    graph.connect('Madrid','Barcelona',628)
    graph.connect('Madrid','Lisbon',638)
    graph.connect('Lisbon', 'London', 2210)
    graph.connect('Barcelona', 'Lyon', 644)
    graph.connect('Paris', 'London', 414)
    graph.connect('London', 'Dublin', 463)
    graph.connect('London', 'Glasgow', 667)
    graph.connect('Glasgow', 'Amsterdam', 711)
    graph.connect('Budapest', 'Prague', 443)
    graph.connect('Barcelona', 'Rome', 1471)
    graph.connect('Paris', 'Bordeaux', 579)
    graph.connect('Glasgow', 'Dublin', 306)
    # Make graph undirected, create symmetric connections
    graph.make_undirected()
    # Create heuristics (straight-line distance, air-travel distance)
    heuristics = {}
    heuristics['Bucharest'] = 0
    heuristics['Amsterdam'] = 2280
    heuristics['Athens'] = 1300
    heuristics['Barcelona'] = 2670
    heuristics['Belgrade'] = 630
    heuristics['Berlin'] = 1800
    heuristics['Bordeaux'] = 2100
    heuristics['Budapest'] = 900
    heuristics['Copenhagen'] = 2250
    heuristics['Dublin'] = 2530
    heuristics['Glasgow'] = 2470
    heuristics['Helsinki'] = 2820
    heuristics['Lisbon'] = 3950
    heuristics['London'] = 2590
    heuristics['Lyon'] = 1660
    heuristics['Madrid'] = 3300
    heuristics['Milan'] = 1750
    heuristics['Munich'] = 1600
    heuristics['Oslo'] = 2870
    heuristics['Palermo'] = 1280
    heuristics['Paris'] = 2970
    heuristics['Prague'] = 1490
    heuristics['Rome'] = 1140
    heuristics['Sofia'] = 390
    heuristics['Stockholm'] = 2890
    heuristics['Vienna'] = 1150
    heuristics['Warsaw'] = 946
    #basic_greedy('Bucharest','Paris')
    path = astar_search(graph, heuristics, 'Bucharest', 'Paris')
    print(path)
    print()


# Tell python to run main method
if __name__ == "__main__": main()