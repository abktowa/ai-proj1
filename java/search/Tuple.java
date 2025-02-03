package search;

public class Tuple{
	private Action action;
	private State state;
	
	public Tuple(Action act, State st){
		this.action = act;
		this.state = st;
	}
	
	public Action getAction(){
		return this.action;
	}
	
	public State getState(){
		return this.state;
	}
	
	public String toString(){
		return "("+this.action+", "+this.state+")";
	}
}