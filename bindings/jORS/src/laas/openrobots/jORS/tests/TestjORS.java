package laas.openrobots.jORS.tests;

import java.util.Set;

import laas.openrobots.jORS.OrsBridge;
import laas.openrobots.jORS.SimulatedRobot;
import laas.openrobots.jORS.helpers.Waypoint;
import laas.openrobots.jORS.helpers.WaypointImpl;

public class TestjORS {

	public static final int FIELD_SIZE = 10;

	public static void main(String[] args) {
		Set<SimulatedRobot> robots = OrsBridge.getRobots();
		
		for (SimulatedRobot r : robots) {
			Waypoint w = new WaypointImpl(Math.random() * FIELD_SIZE, Math.random() * FIELD_SIZE);
			r.goTo(w);	
		}
		

	}

}
