package search;

/* YOU MAY WISH TO SUBCLASS THIS CLASS FOR 
ONE OR MORE OF THE PROBLEM TYPES */

public class State{
	private String name;
	
	public State(String n){
		this.name = n;
	}
	
	public String getName(){
		return name;
	}
	
	public String toString(){
		return "State: "+name;
	}
	
	public boolean equals(State s){
		return this.name.equals(s.name);
	}
	
	public boolean equals(Object other){
		if(!(other instanceof State))
			return false;
		if(other == this)
			return true;
		
		State s = (State) other;
		return this.name.equals(s.name);
	}
	
	public int hashCode(){
		return name.hashCode();
	}
}
