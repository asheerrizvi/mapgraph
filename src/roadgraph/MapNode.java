package roadgraph;

import java.util.ArrayList;
import java.util.List;
import geography.GeographicPoint;

/** The MapNode class is used to Map a GeographicPoint to a list of edges. 
 * These edges are in turn represented as objects of another class called MapEdge.
 * @author Ice 
 */
public class MapNode{
	private GeographicPoint location;
	private List<MapEdge> edges;
	private double startDistance;
	private double stDistance;
	
	/** Creates a new MapNode having a GeographicPoint for location 
	 * and an empty List of MapEdge(s) called edges.
	 * @param loc
	 */
	public MapNode(GeographicPoint loc)
	{
		location = loc;
		edges = new ArrayList<MapEdge>();
	}
	
	/** Adds an edge to the list of MapEdge(s)
	 * @param edge
	 */
	public void addEdge(MapEdge edge) 
	{
		edges.add(edge);
	}

	/** Used to retrieve the location of the MapNode,
	 * the location returned is a GeographicPoint.
	 * @return A GeographicPoint
	 */
	public GeographicPoint getLocation() {
		return location;
	}
	
	/** Used to get the List of all the MapEdge(s) associated with a particular node.
	 * @return A List of MapEdge(s)
	 */
	public List<MapEdge> getEdges() {
		return edges;
	}

	public double getStartDistance() {
		return startDistance;
	}

	public void setStartDistance(double startDistance) {
		this.startDistance = startDistance;
	}

	public double getStDistance() {
		return stDistance;
	}

	public void setStDistance(double stDistance) {
		this.stDistance = stDistance;
	}
	
}
