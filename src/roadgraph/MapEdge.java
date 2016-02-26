package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
	
	GeographicPoint from;
	GeographicPoint to;
	String streetName;
	String streetType;
	double distance;
	
	public MapEdge(GeographicPoint from, GeographicPoint to, String streetName, String streetType, double distance) {
		this.from = from;
		this.to = to;
		this.streetName = streetName;
		this.streetType = streetType;
		this.distance = distance;
	}
	
	

	

}
