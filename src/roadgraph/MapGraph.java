/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

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
	
	HashMap<GeographicPoint, MapNode> vertices;
	List<MapEdge> edges;
	Map<GeographicPoint,ArrayList<MapNode>> adjListsMap;
	
	private final int DEFAULT_SIZE = 10;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		vertices = new HashMap<GeographicPoint, MapNode>();
		edges= new ArrayList<MapEdge>();
		adjListsMap = new HashMap<GeographicPoint,ArrayList<MapNode>>();
	}
	
	public MapGraph(int width, int height)
	{
		vertices = new HashMap<GeographicPoint, MapNode>();
		edges= new ArrayList<MapEdge>();
		adjListsMap = new HashMap<GeographicPoint,ArrayList<MapNode>>();
		height = DEFAULT_SIZE;
		width = DEFAULT_SIZE;
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return vertices.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		Set<GeographicPoint> geographicPoints = new LinkedHashSet<GeographicPoint>();
		for (MapNode node : vertices.values() ) {
			geographicPoints.add(node.getLocation());
		}
		return geographicPoints;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return edges.size();
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
		adjListsMap.put(location, new ArrayList<MapNode>());
		return vertices.put(location, new MapNode(location)) == null;
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
		// Update adjacency matrix
		adjListsMap.get(from).add(vertices.get(to));
		edges.add(new MapEdge(from, to, roadName, roadType, length));
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		MapNode startNode = vertices.get(start);
		MapNode goalNode = vertices.get(goal);
		
		if (startNode == null || goalNode == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return new LinkedList<GeographicPoint>();
		}
		
		HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();
		boolean found = bfsSearch(startNode, goalNode, parentMap);
		
		if (!found) {
			System.out.println("No path exists");
			return null;
		}
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

		return constructPath(startNode, goalNode, parentMap);
	}
	

	private boolean bfsSearch(MapNode startNode, MapNode goalNode,
			HashMap<MapNode, MapNode> parentMap) {
		
		Queue<MapNode> toExplore = new LinkedList<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		
		toExplore.add(startNode);
		boolean found = false;
		while (!toExplore.isEmpty()) {
			MapNode curr = toExplore.remove();
			if (curr == goalNode) {
				found = true;
				break;
			}
			List<MapNode> neighbors = adjListsMap.get(curr.getLocation());
			ListIterator<MapNode> it = neighbors.listIterator(neighbors.size());
			while (it.hasPrevious()) {
				MapNode next = it.previous();
				if (!visited.contains(next)) {
					visited.add(next);
					parentMap.put(next, curr);
					toExplore.add(next);
				}
			}
		}
		
		return found;
	}

	/*private List<GeographicPoint> getNeigborsFromGeographicPoint(GeographicPoint curr) {
		List<GeographicPoint> neigbors = new LinkedList<GeographicPoint>();
		ArrayList<MapNode> neighborNodes =  adjListsMap.get(curr);
		for (MapNode mapNode : neighborNodes) {
			neigbors.add(mapNode.getLocation());
		}
		return neigbors;
	}*/

	private List<GeographicPoint> constructPath(MapNode startNode, MapNode goalNode,
			HashMap<MapNode, MapNode> parentMap) {
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode curr = goalNode;
		while (curr != startNode) {
			path.addFirst(curr.getLocation());
			curr = parentMap.get(curr);
		}
		path.addFirst(startNode.getLocation());
		return path;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
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
		// TODO: Implement this method in WEEK 3

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
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
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		//GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		GraphLoader.loadRoadMap("data/graders/mod2/map2.txt", theMap);
		System.out.println("DONE.");
		theMap.printGraph();
		
		List<GeographicPoint> path = theMap.bfs(new GeographicPoint(6, 6), new GeographicPoint(0, 0));
		theMap.setPath(path);
		
		// You can use this method for testing.  
		
		/* Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
	private void setPath(List<GeographicPoint> path) {
		if (path != null) {
			System.out.println("BFS PATH:");
			for (GeographicPoint n : path) {
				System.out.println(n);
			}
		}
		else {
			System.out.println("null");
		}

	}

	public void printGraph(){
		for (GeographicPoint v : getVertices()) {
			System.out.println("Vertex:" + v);
			System.out.println("Neighbors:");
			for (MapNode neighbor : adjListsMap.get(v)) {
				System.out.println(neighbor);
			}
		}
	}
	
	/*public void printGraph(){
		for (MapNode v : vertices.values()) {
			
		}
	}*/
	
}
