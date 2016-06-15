package roadgraph;

import geography.GeographicPoint;

public class NodePriority implements Comparable<NodePriority> {
	private GeographicPoint location;
	private double priority;

	
	public NodePriority (GeographicPoint loc, double dis) {
		location = loc;
		priority = dis;
	}

	public GeographicPoint getLocation() {
		return location;
	}

	public double getPriority() {
		return priority;
	}


	@Override
	public int compareTo(NodePriority o) {
		// TODO Auto-generated method stub
		return Double.compare(this.priority, o.priority);
	}
	

}
