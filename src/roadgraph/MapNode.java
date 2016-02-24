package roadgraph;

import geography.GeographicPoint;

import java.util.LinkedList;
import java.util.List;

public class MapNode {
	
	GeographicPoint location;
	List<MapEdge> edges;
	private List<MapNode> neighbors;

	private char displayChar;
	
	public static final char EMPTY = '-';
	public static final char PATH = 'o';
	public static final char START = 'S';
	public static final char GOAL = 'G';
	
	/**
	 * @return the displayChar
	 */
	public char getDisplayChar() {
		return displayChar;
	}

	/**
	 * @param displayChar the displayChar to set
	 */
	public void setDisplayChar(char displayChar) {
		this.displayChar = displayChar;
	}
	
	public MapNode(int row, int col)
	{
		this.location.setLocation(row, col);
		neighbors = new LinkedList<MapNode>();
		displayChar = EMPTY;
	}

	public void addNeighbor(MapNode neighbor) 
	{
		neighbors.add(neighbor);
	}
	
	/**
	 * @return the neighbors
	 */
	public List<MapNode> getNeighbors() {
		return neighbors;
	}

	/**
	 * @return the row
	 */
	public double getRow() {
		return location.getX();
	}

	/**
	 * @return the column
	 */
	public double getColumn() {
		return location.getY();
	}
}
