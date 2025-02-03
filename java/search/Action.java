package search;

/* YOU MAY WISH TO SUBCLASS THIS CLASS FOR 
ONE OR MORE OF THE PROBLEM TYPES */

public class Action{
	private String name;
	
	public Action(String n){
		this.name = n;
	}
	
	public String toString(){
		return "Action: "+name;
	}
	
	public String getName(){
		return name;
	}
	
	public boolean equals(Action other){
		return (this.name.equals(other.name));
	}
	
	public boolean equals(Object other){
		if(!(other instanceof Action))
			return false;
		if(other == this)
			return true;
		
		Action a = (Action) other;
		return this.name.equals(a.name);
	}
	
	public int hashCode(){
		return name.hashCode();
	}
}
