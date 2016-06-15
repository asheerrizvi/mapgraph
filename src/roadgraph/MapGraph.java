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
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
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
	//TODO: Add your member variables here in WEEK 2
	private HashMap<GeographicPoint, MapNode> vertices;
	private int edges;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		vertices = new HashMap<GeographicPoint, MapNode>();
		edges = 0;
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return vertices.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		return vertices.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
		return edges;
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
		// TODO: Implement this method in WEEK 2
		if(vertices.containsKey(location) || location.equals(null))
			return false;
		else{
			MapNode ob = new MapNode(location);
			vertices.put(location, ob);
			return true;
		}
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
		//TODO: Implement this method in WEEK 2
		MapNode node = vertices.get(from);
		if(node.equals(null) || vertices.get(to).equals(null) || from.equals(null) || to.equals(null) || 
				roadName.equals(null) || roadType.equals(null) || length == 0)
		{
			throw new IllegalArgumentException("Illegal Argument!");
		}
		MapEdge edge = new MapEdge(from, to, roadName, roadType, length);
		edges++;
		node.addEdge(edge);
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
		// TODO: Implement this method in WEEK 2
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		HashMap<GeographicPoint, ArrayList<GeographicPoint>> adjListsMap = makeAdjlist();
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		
		if(start == null || goal == null) {
			System.out.println("No Path Exists---");
			return new LinkedList<GeographicPoint>();
		}
		
		boolean found = bfsSearch(start, goal, adjListsMap, parentMap, nodeSearched);
		if(!found){
			System.out.print("No Path Exists<<<");
			return new LinkedList<GeographicPoint>();
		}
		return makePath(start, goal, parentMap);
		
	}
	
	
	/** Reconstructs the path to be returned as a result of BFS
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param parentMap A HashMap of two GeographicPoint(s)
	 * @return The list of GeographicPoint(s).
	 */
	public List<GeographicPoint> makePath(GeographicPoint start, GeographicPoint goal, 
										  HashMap<GeographicPoint, GeographicPoint> parentMap)
	{
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		GeographicPoint curr = goal;
		while(curr != start){
			path.addFirst(curr);
			curr = parentMap.get(curr);
		}
		path.addFirst(start);
		return path;
	}
	
	
	/** Performs the search from start node to the goal node. 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param adjListsMap The adjacency list representing each node in the graph and it's neighbors.
	 * @param parentMap A HashMap of two GeographicPoint(s)
	 * @param nodeSearched A hook for visualization.
	 * @return  Returns true if a path is found else returns false.
	 */
	public boolean bfsSearch(GeographicPoint start, GeographicPoint goal,
							 HashMap<GeographicPoint, ArrayList<GeographicPoint>> adjListsMap, 
							 HashMap<GeographicPoint, GeographicPoint> parentMap,
							 Consumer<GeographicPoint> nodeSearched)
	{
		//
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		Queue<GeographicPoint> toExplore = new LinkedList<GeographicPoint>();
		toExplore.add(start);
		visited.add(start);
		boolean found = false;
		while(!toExplore.isEmpty()){
			GeographicPoint curr = toExplore.remove();
			if(curr.equals(goal)){
				found = true;
				break;
			}
			ArrayList<GeographicPoint> neighbors = adjListsMap.get(curr);
			for(GeographicPoint n : neighbors){
				if(!visited.contains(n)){
					visited.add(n);
					parentMap.put(n, curr);
					toExplore.add(n);
					nodeSearched.accept(n);
				}
			}
		}
		return found;
	}
	
	
	/** Makes an adjacency list by iterating through all the vertices in the graph and associating each
	 * one with its neighbors.
	 * @return A HashMap of a GeographicPoint associated with an ArrayList of GeographicPoint(s) which 
	 * is to be used as the Adjacency List.
	 */
	public HashMap<GeographicPoint, ArrayList<GeographicPoint>> makeAdjlist()
	{
		HashMap<GeographicPoint, ArrayList<GeographicPoint>> adjListsMap 
		= new HashMap<GeographicPoint, ArrayList<GeographicPoint>>();
		for(GeographicPoint point : vertices.keySet()){
			ArrayList<GeographicPoint> neighbors = new ArrayList<GeographicPoint>();
			MapNode node = vertices.get(point);
			List<MapEdge> edges = node.getEdges();
			for(MapEdge edge : edges){
				neighbors.add(edge.getEnd());
			}
			adjListsMap.put(point, neighbors);
		}
		return adjListsMap;
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
		HashMap<GeographicPoint, ArrayList<GeographicPoint>> adjListsMap = makeAdjlist();
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		
		if(start == null || goal == null) {
			System.out.println("No Path Exists---");
			return new LinkedList<GeographicPoint>();
		}
		
		boolean found = dSearch(start, goal, adjListsMap, parentMap, nodeSearched);
		/*
		for (Map.Entry<GeographicPoint, GeographicPoint> entry : parentMap.entrySet()) {
		   System.out.println("Value" + entry.getKey() + entry.getValue());
		}
		*/
		if(!found){
			System.out.print("No Path Exists<<<");
			return new LinkedList<GeographicPoint>();
		}
		return makePath(start, goal, parentMap);
	}
	
	public boolean dSearch(GeographicPoint start, GeographicPoint goal,
			 				 HashMap<GeographicPoint, ArrayList<GeographicPoint>> adjListsMap, 
			 				 HashMap<GeographicPoint, GeographicPoint> parentMap,
			 				 Consumer<GeographicPoint> nodeSearched)
	{
		PriorityQueue<NodePriority> toExplore = new PriorityQueue<NodePriority>();
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		int count = 0;
		setInfinity();
		toExplore.add(new NodePriority(start, 0));
		boolean found = false;
		while(!toExplore.isEmpty()){
			NodePriority rnode = toExplore.poll();
			count++;
			GeographicPoint curr = rnode.getLocation();
			if(!visited.contains(curr)) {
				visited.add(curr);
				if(curr.equals(goal)){
					found = true;
					break;
				}
				ArrayList<GeographicPoint> neighbors = adjListsMap.get(curr);
				for(GeographicPoint n : neighbors){
					if(!visited.contains(n)){	
						MapNode nOb = vertices.get(n);
						double dis = getNDistance(curr, n);
						if((dis + rnode.getPriority()) < nOb.getStartDistance()){
							nOb.setStartDistance(dis + rnode.getPriority());
							parentMap.put(n, curr);
							toExplore.add(new NodePriority(n, dis + rnode.getPriority()));
							nodeSearched.accept(n);
						}
					}
				}
			}
		}
		System.out.println(count);
		return found;
	}
	
	public double getNDistance(GeographicPoint curr, GeographicPoint n) {
		List<MapEdge> currOb = vertices.get(curr).getEdges();
		for(MapEdge edge : currOb){
			if((edge.getEnd()).equals(n)){
				return edge.getDistance();
			}
		}
		return 0.0;
	}
	
	public void setInfinity() {
		for (GeographicPoint p : vertices.keySet()) {
			vertices.get(p).setStartDistance(Integer.MAX_VALUE);
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
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		HashMap<GeographicPoint, ArrayList<GeographicPoint>> adjListsMap = makeAdjlist();
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		
		if(start == null || goal == null) {
			System.out.println("No Path Exists---");
			return new LinkedList<GeographicPoint>();
		}
		
		boolean found = aSearch(start, goal, adjListsMap, parentMap, nodeSearched);
		/*
		for (Map.Entry<GeographicPoint, GeographicPoint> entry : parentMap.entrySet()) {
		   System.out.println("Value" + entry.getKey() + entry.getValue());
		}
		*/
		if(!found){
			System.out.print("No Path Exists<<<");
			return new LinkedList<GeographicPoint>();
		}
		return makePath(start, goal, parentMap);
	}
	
	public boolean aSearch(GeographicPoint start, GeographicPoint goal,
			 HashMap<GeographicPoint, ArrayList<GeographicPoint>> adjListsMap, 
			 HashMap<GeographicPoint, GeographicPoint> parentMap,
			 Consumer<GeographicPoint> nodeSearched)
	{
		PriorityQueue<StNodePriority> toExplore = new PriorityQueue<StNodePriority>();
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		setAInfinity();
		int count = 0;
		toExplore.add(new StNodePriority(start, 0, 0));
		boolean found = false;
		while(!toExplore.isEmpty()){
			StNodePriority rnode = toExplore.poll();
			count++;
			GeographicPoint curr = rnode.getLocation();
			if(!visited.contains(curr)) {
				visited.add(curr);
				if(curr.equals(goal)){
					found = true;
					break;
				}
				ArrayList<GeographicPoint> neighbors = adjListsMap.get(curr);
				for(GeographicPoint n : neighbors){
					if(!visited.contains(n)){	
						MapNode nOb = vertices.get(n);
						double dis = getNDistance(curr, n);
						double stdis = getStDistance(n, goal);
						if(dis + rnode.getPriority() < nOb.getStartDistance()){
							nOb.setStartDistance(dis + rnode.getPriority());
							nOb.setStDistance(stdis);
							parentMap.put(n, curr);
							toExplore.add(new StNodePriority(n, dis + rnode.getPriority(), stdis + dis + rnode.getPriority()));
							nodeSearched.accept(n);
						}
					}
				}
			}
		}
		System.out.println(count);
		return found;
	}

	public void setAInfinity() {
		for (GeographicPoint p : vertices.keySet()) {
			vertices.get(p).setStartDistance(Integer.MAX_VALUE);
			vertices.get(p).setStDistance(Integer.MAX_VALUE);
		}	
	}
	
	public double getStDistance(GeographicPoint n, GeographicPoint goal) {
		return n.distance(goal);
	}
	
	
	public static void main(String[] args)
	{

		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);	
	}
}
