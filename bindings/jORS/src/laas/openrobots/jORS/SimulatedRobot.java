package laas.openrobots.jORS;

import java.util.Set;

import laas.openrobots.jORS.exceptions.SimulatorConnectorException;
import laas.openrobots.jORS.helpers.Waypoint;
import laas.openrobots.jORS.states.FollowStatus;
import laas.openrobots.jORS.states.MoveStatus;

public interface SimulatedRobot {
	
	public String getName();
	
	/***** MOVEMENTS *****/
	
	public void move(float angle, float speed);
	public void goTo(Waypoint waypoint);
	public void followPath(Set<Waypoint> waypoints);
	public FollowStatus getFollowStatus();
	public MoveStatus getMoveStatus();
	
	/***** COMMUNICATION *****/
	public Set<SimulatedRobot> getNetworkEntities();
}
