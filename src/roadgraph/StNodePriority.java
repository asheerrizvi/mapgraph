package roadgraph;

import geography.GeographicPoint;

public class StNodePriority implements Comparable<StNodePriority> {
	private GeographicPoint location;
	private double priority;
	private double stpriority;

	
	public StNodePriority (GeographicPoint loc, double dis, double totaldis) {
		location = loc;
		priority = dis;
		stpriority = totaldis;
	}

	public GeographicPoint getLocation() {
		return location;
	}

	public double getPriority() {
		return priority;
	}


	@Override
	public int compareTo(StNodePriority o) {
		// TODO Auto-generated method stub
		return Double.compare(this.stpriority, o.stpriority);
	}
	

}
