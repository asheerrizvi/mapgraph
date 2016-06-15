package roadgraph;

import geography.GeographicPoint;

/** The MapEdge class is used to represent a set of edges originating from a node.
 * @author Ice
 */
public class MapEdge {
	private GeographicPoint start;
	private GeographicPoint end;
	private String streetName;
	private String streetType;
	private double distance;
	
	/** Creates a new MapEdge associating one GeographicPoint to another GeographicPoint.
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 */
	public MapEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length)
	{
		start = from;
		end = to;
		streetName = roadName;
		streetType = roadType;
		distance = length;
	}
	
	/** Getter for the starting point of the edge.
	 * @return A GeographicPoint
	 */
	public GeographicPoint getStart() {
		return start;
	}

	/** Getter for the ending point of the edge.
	 * @return A GeographicPoint
	 */
	public GeographicPoint getEnd() {
		return end;
	}
	
	/** Getter for the name of the street making up the edge.
	 * @return A String
	 */
	public String getStreetName() {
		return streetName;
	}
	
	/** Getter for the type of the street making up the edge.
	 * @return A String
	 */
	public String getStreetType() {
		return streetType;
	}
	
	/** Getter for the length of the edge, in km.
	 * @return A double 
	 */
	public double getDistance() {
		return distance;
	}
}
