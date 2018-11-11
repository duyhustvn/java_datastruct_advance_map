/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.io.IOException;
import java.util.*;
import java.util.function.Consumer;
import java.util.logging.FileHandler;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;

import com.sun.org.apache.bcel.internal.generic.GETFIELD;
import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 3
	private int numVerticles;
	private int numEdges;
	HashMap<GeographicPoint, MapNode> nodes;
	List<MapEdge> edges;

	private final static Logger LOGGER = Logger.getLogger(MapGraph.class.getName());
//	FileHandler fh;
//
//    {
//        try {
//            fh = new FileHandler("logger/info.log");
//            SimpleFormatter formatter = new SimpleFormatter();
//            fh.setFormatter(formatter);
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
//    }


    /**
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
		nodes = new HashMap<GeographicPoint, MapNode>();
		edges = new ArrayList<MapEdge>();
		numEdges = 0;
		numVerticles = 0;
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return numVerticles;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
        //TODO: Implement this method in WEEK 3
	    Set<GeographicPoint> allVertices = new HashSet<GeographicPoint>();
	    Set<Map.Entry<GeographicPoint, MapNode>> set = nodes.entrySet();
	    Iterator<Map.Entry<GeographicPoint, MapNode>> itr = set.iterator();
	    while(itr.hasNext()) {
            Map.Entry<GeographicPoint, MapNode> node = itr.next();
            allVertices.add(node.getKey());
        }
		return allVertices;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return numEdges;
	}

	public MapNode getVertex(GeographicPoint location) {
		return nodes.get(location);
	}
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 3

		// check if node/vertex is exists
		MapNode foundNode = this.getVertex(location);
		List<MapEdge> neighbors = new ArrayList<MapEdge>();
		MapNode currentNode = new MapNode(location, neighbors);
		if (foundNode == null) {
//            LOGGER.info("Add node " + location);
		    this.numVerticles++;
            nodes.put(location, currentNode);
			return true;
		}
//        LOGGER.info("Node " + location + " is already exists");
		return false;
	}

    // Check if edge is added
	public boolean isEdgeAdded(MapNode startNode, MapNode endNode) {
		for (MapEdge edge: edges) {
			if (edge.isEqual(startNode, endNode)) {
				return true;
			}
		}
		return false;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 3
		if (length <= 0) {
//			LOGGER.warning("Length is: " + length);
			throw new IllegalArgumentException("Length is " + length);
		}

		MapNode startNode = this.getVertex(from);
		if (startNode == null) {
//			LOGGER.warning("Node " + from + " is not add to map");
			throw new IllegalArgumentException( "Node " + from + " is not add to map");
		}

		MapNode endNode = this.getVertex(to);
		if (startNode == null) {
//			LOGGER.warning("Node " + to + " is not add to map");
			throw new IllegalArgumentException("Node " + to + " is not add to map");
		}


 		boolean isEdgeAdded = this.isEdgeAdded(startNode, endNode);
		if (isEdgeAdded) {
//			LOGGER.info( startNode.getLocation() + " and " + endNode.getLocation() + " is alredy added to edge ");
		} else {
			MapEdge edge = new MapEdge(startNode, endNode, roadName, roadType);
			startNode.addEdge(edge);
			edges.add(edge);
//			LOGGER.info("Add " + startNode.getLocation() + " and " + endNode.getLocation() + " to edge ");
			this.numEdges++;
		}
	}


	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */

	/** Algorithm
	 * Path: HashMap<startNode, MapNode> store the path from start to goal
	 * Queue: Queue<MapNode> store the next MapNode to traversal
	 * Visited: HashSet<startNode, List<MapNode>> store node that is visited
	 *
	 * Add startNode to queue
	 * while(queue is not null):
	 * 		dequeue the first element as current node
	 * 		if (current == goalNode) return ;
	 * 		get the neighbors of current node
	 * 		for each node n in neighbors and n is not visited:
	 * 			add n to visited
	 *			add current, n to path
	 *			enqueue n to queue
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
		// Dummy variable for calling the search algorithms
        // Consumer<GeographicPoint> temp = (x) -> {};
	    // return bfs(start, goal, temp);
		HashSet<MapNode> visited = new HashSet<MapNode>();
		// HashMap<MapNode, MapNode> path = new HashMap<MapNode, MapNode>();
		HashMap<MapNode, MapNode> path = new HashMap<MapNode, MapNode>();
		Queue<MapNode> queue = new LinkedList<MapNode>();
		boolean isFound = false;

		MapNode startNode = nodes.get(start);
		MapNode goalNode = nodes.get(goal);
		queue.add(startNode);
		visited.add(startNode);
		while(queue.size() != 0) {
			MapNode currentNode = queue.remove();
			nodeSearched.accept(currentNode.getLocation());
			if (currentNode.isEqual(goalNode)) {
				isFound = true;
				break;
			}
			List<MapNode> neighbors = currentNode.getNeighbors();
			for (MapNode neighbor: neighbors) {
				if (visited.contains(neighbor)) continue;
				visited.add(neighbor);
				path.put(neighbor, currentNode);
				queue.add(neighbor);
			}
		}
		if (isFound) {
			return this.buildPath(startNode, goalNode, path);
		} else {
			return null;
		}
	}

	public List<GeographicPoint> buildPath(MapNode startNode, MapNode goalNode, HashMap<MapNode, MapNode> path) {
		double distance = 0.0;
		LinkedList <GeographicPoint> buildPath = new LinkedList<GeographicPoint>();
		buildPath.addFirst(goalNode.getLocation());
		MapNode curr = path.get(goalNode);
		distance += curr.getLocation().distance(goalNode.getLocation());
		while (!curr.isEqual(startNode)) {
			buildPath.addFirst(curr.getLocation());
			MapNode prev = curr;
			curr = path.get(curr);
			distance += curr.getLocation().distance(prev.getLocation());
		}
		buildPath.addFirst(startNode.getLocation());
		System.out.println("distance: " + distance);
		return buildPath;
	}

	public void printVisited(List<MapNode> visited) {
//		Set<MapNode> set = new HashSet<MapNode>(visited);
//		int i = 1;
//		for (MapNode s: set) {
//			System.out.println(i + " " + s.getLocation().toString());
//			i += 1;
//		}
		int i = 1;
		for (MapNode node: visited) {
			System.out.println(i + " " + node.getLocation().toString());
			i++;
		}
	}

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal)
	{
		// TODO: Implement this method in WEEK 3
		HashSet<MapNode> visited = new HashSet<MapNode>();
		// HashMap<MapNode, MapNode> path = new HashMap<MapNode, MapNode>();
		HashMap<MapNode, MapNode> path = new HashMap<MapNode, MapNode>();
		Queue<MapNode> queue = new LinkedList<MapNode>();
		boolean isFound = false;

		MapNode startNode = nodes.get(start);
		MapNode goalNode = nodes.get(goal);
		queue.add(startNode);
		visited.add(startNode);
		while(queue.size() != 0) {
			MapNode currentNode = queue.remove();
			if (currentNode.isEqual(goalNode)) {
				isFound = true;
				break;
			}
			List<MapNode> neighbors = currentNode.getNeighbors();
			for (MapNode neighbor: neighbors) {
				if (visited.contains(neighbor)) continue;
				visited.add(neighbor);
				path.put(neighbor, currentNode);
				queue.add(neighbor);
			}
		}
		if (isFound) {
			return this.buildPath(startNode, goalNode, path);
		} else {
			return null;
		}
	}


	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */

	/**
	 * Algorithm:
	 * path: HashMap<neighbor, current>
	 * visited: HashSet<start, List<MapNode>>
	 *
	 * Add startNode to queue
	 * while queue is not empty:
	 * 	dequeue the head element as current
	 * 	if current == goalNode:
	 * 		return path
	 * 	for each current's neighbor, n and n is not visited:
	 * 		get sum of distance from current to startNode through some node
	 * 		add n to visited
	 * 		add to path HashMap<neighbor, current>
	 * 		 enqueue to queue in respect of distance
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		HashMap<NodeWithDistance, NodeWithDistance> path = new HashMap<NodeWithDistance, NodeWithDistance>();
		List<MapNode> visited = new LinkedList<MapNode>();
		Queue<NodeWithDistance> queue = new PriorityQueue<NodeWithDistance>();
		boolean isFound = false;

		MapNode startNode = nodes.get(start);
		MapNode goalNode = nodes.get(goal);
		NodeWithDistance startNodeWithDistance = new NodeWithDistance(startNode);
		NodeWithDistance curr = new NodeWithDistance();

		queue.add(new NodeWithDistance(startNode));
		while(!queue.isEmpty()) {
			curr = queue.remove();
			if (visited.contains(curr.getNode())) continue;
			visited.add(curr.getNode());
			if (curr.getNode().isEqual(goalNode)) {
				isFound = true;
				break;
			}

			List<MapNode>  neighbors = curr.getNode().getNeighbors();
			for (MapNode neighbor: neighbors) {
				double distance = curr.getDistance() + curr.getNode().getLocation().distance(neighbor.getLocation());
				NodeWithDistance neighborNodeWithDistance = new NodeWithDistance(neighbor, distance);
				path.put(neighborNodeWithDistance, curr);
				queue.add(neighborNodeWithDistance);
			}
		}

		if (isFound) {
			System.out.println("Path: ");
			return this.buildPathDijkstra(startNodeWithDistance, curr, path);
		} else {
			return null;
		}
	}

	public List<GeographicPoint> buildPathDijkstra (NodeWithDistance startNode, NodeWithDistance goalNode, HashMap<NodeWithDistance, NodeWithDistance> parent) {
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		path.addFirst(goalNode.getNode().getLocation());
		NodeWithDistance curr = parent.get(goalNode);
		while (!curr.isEqual(startNode)) {
			path.addFirst(curr.getNode().getLocation());
			curr = parent.get(curr);
		}
		path.addFirst(startNode.getNode().getLocation());
		return path;
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		HashMap<NodeWithDistance, NodeWithDistance> path = new HashMap<NodeWithDistance, NodeWithDistance>();
		List<MapNode> visited = new LinkedList<MapNode>();
		Queue<NodeWithDistance> queue = new PriorityQueue<NodeWithDistance>();
		boolean isFound = false;

		MapNode startNode = nodes.get(start);
		MapNode goalNode = nodes.get(goal);
		NodeWithDistance startNodeWithDistance = new NodeWithDistance(startNode);
		NodeWithDistance curr = new NodeWithDistance();

		queue.add(new NodeWithDistance(startNode));
		while(!queue.isEmpty()) {
			curr = queue.remove();
			if (visited.contains(curr.getNode())) continue;
			nodeSearched.accept(curr.getNode().getLocation());
			visited.add(curr.getNode());
			if (curr.getNode().isEqual(goalNode)) {
				isFound = true;
				break;
			}

			List<MapNode>  neighbors = curr.getNode().getNeighbors();
			for (MapNode neighbor: neighbors) {
				double distance = curr.getDistance() + curr.getNode().getLocation().distance(neighbor.getLocation());
				NodeWithDistance neighborNodeWithDistance = new NodeWithDistance(neighbor, distance);
				path.put(neighborNodeWithDistance, curr);
				queue.add(neighborNodeWithDistance);
			}
		}

		if (isFound) {
			System.out.println("Path: ");
			return this.buildPathDijkstra(startNodeWithDistance, curr, path);
		} else {
			return null;
		}
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	public void printPath(List<GeographicPoint> list) {
		for(GeographicPoint geographicPoint: list) {
			System.out.println(geographicPoint.toString());
		}
	}
	
	public static void main(String[] args)
	{
//		System.out.print("Making a new map...");
//		MapGraph firstMap = new MapGraph();
//		System.out.print("DONE. \nLoading the map...");
//		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
//		System.out.println("DONE.");
//
//		firstMap.addVertex(new GeographicPoint(1.0, 20.0));
//        firstMap.addVertex(new GeographicPoint(1.0, 20.0));
//
//		System.out.println("Number of edge " +  firstMap.getNumEdges());

//		GeographicPoint startNode = new GeographicPoint(4.0, -1.0);
//		GeographicPoint endNode = new GeographicPoint(8.0, -1.0);
//		firstMap.addEdge(startNode, endNode, "short", "connector", startNode.distance(endNode));
//		System.out.println("Number of edge " +  firstMap.getNumEdges());
//
//		firstMap.addEdge(startNode, endNode, "short", "connector", startNode.distance(endNode));
//		System.out.println("Number of edge " +  firstMap.getNumEdges());
//
//		startNode = new GeographicPoint(4.0, -1.0);
//		endNode = new GeographicPoint(8.0, -1.0);
//		firstMap.addEdge(startNode, endNode, "short", "connector", startNode.distance(endNode));
//		System.out.println("Number of edge " +  firstMap.getNumEdges());

//		List<GeographicPoint> geographicPointList = firstMap.bfs(new GeographicPoint(7.0, 3.0), new GeographicPoint(8.0, -1.0));
//		System.out.println("Path");
//		for (GeographicPoint geographicPoint: geographicPointList) {
//			System.out.println(geographicPoint.toString());
//		}

		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */

		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);

		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);

		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		simpleTestMap.printPath(testroute);
//		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
//
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
//		 A very simple test using real data
		GeographicPoint testStart2 = new GeographicPoint(32.869423, -117.220917);
		GeographicPoint testEnd2 = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		List<GeographicPoint> testroute11 = testMap.dijkstra(testStart2,testEnd2);
		testMap.printPath(testroute11);
//		List<GeographicPoint> testroute22 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
//		GeographicPoint testStart3 = new GeographicPoint(32.8674388, -117.2190213);
//		GeographicPoint testEnd3 = new GeographicPoint(32.8697828, -117.2244506);
//		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
//		List<GeographicPoint> testroute = testMap.dijkstra(testStart3,testEnd3);
//		testroute2 = testMap.aStarSearch(testStart,testEnd);

		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
