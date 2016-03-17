/**
 * A class to represent a node in the map
 */
package roadgraph;

import geography.GeographicPoint;

import java.util.HashSet;
import java.util.Set;

/**
 * @author UCSD MOOC development team
 * 
 *         Class representing a vertex (or node) in our MapGraph
 *
 */
@SuppressWarnings("rawtypes")
class MapNode implements Comparable {
	/** The list of edges out of this node */
	private HashSet<MapEdge> edges;

	/** the latitude and longitude of this node */
	private GeographicPoint location;

	// WEEK 3 SOLUTIONS

	/** the predicted distance of this node (used in Week 3 algorithms) */
	private double distance;

	/** the actual distance of this node from start (used in Week 3 algorithms) */
	private double actualDistance;

	// END WEEK 3 SOLUTIONS

	MapNode(GeographicPoint loc) {
		location = loc;
		edges = new HashSet<MapEdge>();
		distance = 0.0;
		actualDistance = 0.0;
	}

	void addEdge(MapEdge edge) {
		edges.add(edge);
	}

	/** Return the neighbors of this MapNode */
	Set<MapNode> getNeighbors() {
		Set<MapNode> neighbors = new HashSet<MapNode>();
		for (MapEdge edge : edges) {
			neighbors.add(edge.getOtherNode(this));
		}
		return neighbors;
	}

	/** get the location of a node */
	GeographicPoint getLocation() {
		return location;
	}

	/** return the edges out of this node */
	Set<MapEdge> getEdges() {
		return edges;
	}

	/**
	 * Returns whether two nodes are equal. Nodes are considered equal if their locations are the same, even if their
	 * street list is different.
	 */
	public boolean equals(Object o) {
		if (!(o instanceof MapNode) || (o == null)) {
			return false;
		}
		MapNode node = (MapNode) o;
		return node.location.equals(this.location);
	}

	/**
	 * Because we compare nodes using their location, we also may use their location for HashCode.
	 * 
	 * @return The HashCode for this node, which is the HashCode for the underlying point
	 */
	public int HashCode() {
		return location.hashCode();
	}

	/**
	 * ToString to print out a MapNode method
	 * 
	 * @return the string representation of a MapNode
	 */
	public String toString() {
		String toReturn = "[NODE at location (" + location + ")";
		toReturn += " intersects streets: ";
		for (MapEdge e : edges) {
			toReturn += e.getRoadName() + ", ";
		}
		toReturn += "]";
		return toReturn;
	}

	// For debugging, output roadNames as a String.
	public String roadNamesAsString() {
		String toReturn = "(";
		for (MapEdge e : edges) {
			toReturn += e.getRoadName() + ", ";
		}
		toReturn += ")";
		return toReturn;
	}

	// WEEK 3 SOLUTIONS

	// get node distance (predicted)
	public double getDistance() {
		return this.distance;
	}

	// set node distance (predicted)
	public void setDistance(double distance) {
		this.distance = distance;
	}

	// get node distance (actual)
	public double getActualDistance() {
		return this.actualDistance;
	}

	// set node distance (actual)
	public void setActualDistance(double actualDistance) {
		this.actualDistance = actualDistance;
	}

	// Code to implement Comparable
	public int compareTo(Object o) {
		// convert to map node, may throw exception
		MapNode m = (MapNode) o;
		return ((Double) this.getDistance()).compareTo((Double) m.getDistance());
	}

	/**
	 * Find the closest from a set of neighbors
	 * @param startNode 
	 * 
	 * @param unVisitedneighbors
	 * @return closest neighbor
	 */
	public MapNode getClosestNeighbor(Set<MapNode> neighbors, MapNode startNode) {
		double minDistance = Double.POSITIVE_INFINITY;
		MapEdge shortestEdge = null;
		Set<MapEdge> edges = new HashSet<MapEdge>();

		// Get edges of selected neighbors and if no neighbors selected, get all edges
		if (neighbors == null ) {
			edges = getEdges();
		} 
		else if (neighbors.isEmpty()){
			return startNode;
		}else {
			for (MapNode n : neighbors) {
				edges.add(new MapEdge("demo", this, n));
			}
		}

		// Find shortest edge
		for (MapEdge edge : edges) {
			if (edge.getLength() < minDistance) {
				shortestEdge = edge;
			}
		}
		return shortestEdge.getOtherNode(this);
	}
	
	/**
	 * Find the closest neighbor
	 * 
	 * @return closest node
	 */
	public MapNode getClosestNeighbor() {
		double minDistance = Double.POSITIVE_INFINITY;
		MapEdge shortestEdge = null;

		// Find shortest edge
		for (MapEdge edge : getEdges()) {
			if (edge.getLength() < minDistance) {
				shortestEdge = edge;
			}
		}
		return shortestEdge.getOtherNode(this);
	}
}