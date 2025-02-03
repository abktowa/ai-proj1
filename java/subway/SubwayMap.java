package subway;

import java.util.*;
import java.io.*;

public class SubwayMap{
	public HashMap<Integer, Station> stations;
	public HashMap<Station, ArrayList<Link>> links;
	public ArrayList<Link> linkList;
	
	private SubwayMap(){
		stations = new HashMap<>();
		links = new HashMap<>();
		linkList = new ArrayList<>();
	}
	
	/**
	Return the number of stations on the map
	@return The number of stations
	*/
	public int numStations(){
		return stations.size();
	}
	
	/**
	Return a Collection containing all Stations
	@return All Stations on the map
	*/
	public Collection<Station> stations(){
		return stations.values();
	}
	
	/**
	Return the Station object corresponding to a given ID number (or null)
	@param num An ID number
	@return The corresponding Station
	*/
	public Station getStationByID(int num){
		return stations.get(num);
	}
	
	/**
	Return the Station object corresponding to the given station name (or null)
	@param name The Station name
	@return The corresponding Station
	*/
	public Station getStationByName(String name){
		for(Station sta:this.stations()){
			if(name.equals(sta.name))
				return sta;
		}
		return null;
	}
	
	/**
	Return the number of links in the Subway Map.
	@return The number of links
	*/
	public int numLinks(){
		int total = 0;
		for(ArrayList<Link> innerList:links.values()){
			total += innerList.size();
		}
		return total / 2;
	}
	
	/**
	Returns a list containing all links in the SubwayMap.
	@return The list of all links
	*/
	public ArrayList<Link> links(){
		return linkList;
	}
	
	/**
	Returns an ArrayList containing all Links between two Stations. Empty if no such link exists. Note that if there are multiple links between the two stations, then each link will be provided.
	@param u A Station
	@param v Another station
	@return List of links between the two stations
	*/
	public ArrayList<Link> getLinksBetween(Station u, Station v){
		ArrayList<Link> result = new ArrayList<>();
		ArrayList<Link> adj = links.get(u);
		for(Link l:adj){
			if(l.opposite(u).equals(v))
				result.add(l);
		}
		return result;
	}
	
	/**
	Returns the degree for the given Station (the number of links associated with that Station).
	@param v A Station
	@return The degree of the Station
	*/
	public int degree(Station v){
		return links.get(v).size();
	}
	
	/**
	Returns an ArrayList of Links associated with the given Station
	@param v A Station
	@return All links connected to the Station given
	*/
	public ArrayList<Link> incidentLinks(Station v){
		return links.get(v);
	}
	
	/**
	Returns an ArrayList of Stations that are adjacent to the given Station (directly connected by a Link). Note that if a Station is adjacent to the given Station by more than one Link, then the returned list will contain the adjacent Station multiple times (once per Link).
	@param v A station
	@return All stations directly connected to the given Station
	*/
	public ArrayList<Station> adjacentStations(Station v){
		ArrayList<Link> incident = links.get(v);
		ArrayList<Station> result = new ArrayList<>();
		for(Link l:incident){
			result.add(l.opposite(v));
		}
		return result;
	}
	
	private void insertStation(int id, String name, double latitude, double longitude){
		Station vertex = new Station(id, name, latitude, longitude);
		stations.put(id, vertex);
		links.put(vertex, new ArrayList<Link>());
	}
	
	private void insertLink(Station u, Station v, double weight, String line){
		Link newlink = new Link(u, v, weight, line);
		links.get(u).add(newlink);
		links.get(v).add(newlink);
	}
	
	/**
	Calculate the Straight Line Distance (in km) between two Stations
	@param station1 First station
	@param station2 Second station
	@return The straight line distance between the two stations
	*/
	public static double straightLineDistance(Station station1, Station station2){
		final int EARTH_RADIUS = 6371; // kilometers
		
		double s1Latitude = Math.toRadians(station1.latitude);
		double s1Longitude = Math.toRadians(station1.longitude);
		double s2Latitude = Math.toRadians(station2.latitude);
		double s2Longitude = Math.toRadians(station2.longitude);
		
		double deltaLat = Math.abs(s1Latitude - s2Latitude);
		double deltaLong = Math.abs(s1Longitude - s2Longitude);
		
		double underRoot = (Math.sin(deltaLat/2) * Math.sin(deltaLat/2)) + (Math.cos(s1Latitude) * Math.cos(s2Latitude) * Math.sin(deltaLong/2) * Math.sin(deltaLong/2));
		
		double centralAngle = 2 * Math.asin(Math.sqrt(underRoot));
		
		double distance = EARTH_RADIUS * centralAngle;
		
		return distance;
	}
	
	/**
	Return a Subway Map representing the Boston T Map
	@return A SubwayMap object containing the Boston T data
	*/
	public static SubwayMap buildBostonMap() throws FileNotFoundException {
		File stationFile = new File("../data/boston_stations.csv");
		File linkFile = new File("../data/boston_links.csv");
		
		return buildMap(stationFile, linkFile);	
	}
	
	/**
	Return a Subway Map representing the London Underground Map
	@return A SubwayMap object containing the London Underground data
	*/
	public static SubwayMap buildLondonMap() throws FileNotFoundException {
		File stationFile = new File("../data/london_stations.csv");
		File linkFile = new File("../data/london_links.csv");
		
		return buildMap(stationFile, linkFile);	
	}	
	
	private static SubwayMap buildMap(File stationFile, File linkFile) throws FileNotFoundException {
		SubwayMap newmap = new SubwayMap();
		
		Scanner reader = new Scanner(stationFile);
		reader.nextLine(); // advance past header row
		
		while(reader.hasNextLine()){
			String line = reader.nextLine();
			String[] parts = line.split(",");
			
			int idnum = Integer.parseInt(parts[0]);
			String name = parts[1];
			double latitude = Double.parseDouble(parts[2]);
			double longitude = Double.parseDouble(parts[3]);
			
			newmap.insertStation(idnum, name, latitude, longitude);
		}
		
		reader.close();
		
		reader = new Scanner(linkFile);
		reader.nextLine(); // advance past header row
		
		while(reader.hasNextLine()){
			boolean deprecatedRow = false;

			String line = reader.nextLine();
			String[] parts = line.split(",");
			
			String name1 = parts[0];
			int id1 = Integer.parseInt(parts[1]);
			String name2 = parts[2];
			int id2 = Integer.parseInt(parts[3]);
			
			String route = parts[4];
			
			double distanceMiles = Double.parseDouble(parts[5]);
			double distanceKm = Double.parseDouble(parts[6]);
			
			if(parts.length > 7){
				if(parts[7].equals("DEP")){
					deprecatedRow = true;
				}
			}

			if(!deprecatedRow){
				Station station1 = newmap.getStationByID(id1);
				Station station2 = newmap.getStationByID(id2);
				
				newmap.insertLink(station1, station2, distanceKm, route);
			}
		}
		reader.close();
		
		return newmap;
	}
}