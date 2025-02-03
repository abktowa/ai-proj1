package subway;

public class Station{
	public int id;
	public String name;
	public double latitude;
	public double longitude;
	
	protected Station(int i, String n, double lat, double lon){
		id = i;
		name = n;
		latitude = lat;
		longitude = lon;
	}
	
	public String toString(){
		return name + "(" + id + ")";
	}
	
	/**
	Calculate the Straight Line Distance (in km) between this Station and a given Station
	@param station2 Second station
	@return The straight line distance between the two stations
	*/
	public double straightLineDistance(Station station2){
		final int EARTH_RADIUS = 6371; // kilometers
		
		double s1Latitude = Math.toRadians(this.latitude);
		double s1Longitude = Math.toRadians(this.longitude);
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
	Calculate the Straight Line Distance (in km) between two Stations
	@param station1 First station
	@param station2 Second station
	@return The straight line distance between the two stations
	*/
	public static double straightLineDistance(Station s1, Station s2){
		return s1.straightLineDistance(s2);
	}
}