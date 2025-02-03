package subway;

public class Link{
	public Station start;
	public Station end;
	public double distance;
	public String line;
	
	protected Link(Station u, Station v, double weight, String l){
		start = u;
		end = v;
		distance = weight;
		line = l;
	}
	
	public String toString(){
		return start.toString() + "<-->" + end.toString() + "(" + line + ")";
	}
	
	public Station[] endpoints(){
		Station[] result = {start, end};
		return result;
	}
	
	public Station opposite(Station v){
		if(start.equals(v))
			return end;
		else if(end.equals(v))
			return start;
		else
			throw new IllegalArgumentException("Argument is neither endpoint");
	}

	public double getDistance(){
		return distance;
	}
}