package laas.openrobots.jORS;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import laas.openrobots.jORS.connectors.IConnector;
import laas.openrobots.jORS.exceptions.MalformedYarpMessageException;
import laas.openrobots.jORS.exceptions.SimulatorConnectorException;
import laas.openrobots.jORS.exceptions.UnvalidRobotException;
import laas.openrobots.jORS.helpers.Waypoint;
import laas.openrobots.jORS.states.FollowStatus;
import laas.openrobots.jORS.states.MoveStatus;

public class SimulatedRobotImpl implements SimulatedRobot {

	//This map holds the relation "component -> corresponding YARP port".
	//It the YARP port is set to a default value in the static initialization block
	//below, but must be updated with the simulator data when the bridge is initialized.
    private static final Map<String, String> expectedComponents;
    
    static {
        Map<String, String> aMap = new HashMap<String, String>();
        aMap.put("controller", "MOTION_CONTROLLER1");
        aMap.put("network_sensor", "NETWORK_SENSOR1");
        expectedComponents = Collections.unmodifiableMap(aMap);
    }

	
	private String name;
	private Map<String, String> equipment;
	private IConnector connector;
	
	public SimulatedRobotImpl(String name, Map<String, String> equipment, IConnector connector) throws UnvalidRobotException {
		this.name = name;
		this.equipment = equipment;
		this.connector = connector;
		
		checkValidRobot();
	}
	
	@Override
	public String getName() {
		return name;
	}

	@Override
	public void followPath(Set<Waypoint> waypoints) {
		// TODO Auto-generated method stub
		
	}
	
	public void goTo(Waypoint waypoint) {
		try {
			connector.execute(expectedComponents.get("controller"), waypoint.toYarp(), null);
		} catch (MalformedYarpMessageException e) {
			e.printStackTrace();
		} catch (SimulatorConnectorException e) {
			e.printStackTrace();
		}
		
	}

	@Override
	public FollowStatus getFollowStatus() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public MoveStatus getMoveStatus() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Set<SimulatedRobot> getNetworkEntities() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void move(float angle, float speed) {
		// TODO Auto-generated method stub
		
	}


	private void checkValidRobot() throws UnvalidRobotException {
		if (!equipment.keySet().containsAll(
				new HashSet<String>(expectedComponents.values())))
			throw new UnvalidRobotException("One of the robots (" + name + ") lacks one of the expected components for this bridge");
	}
}
