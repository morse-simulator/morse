package laas.openrobots.jORS;

import java.util.Set;

import yarp.Value;

import laas.openrobots.jORS.connectors.IConnector;
import laas.openrobots.jORS.connectors.YarpConnector;
import laas.openrobots.jORS.exceptions.MalformedYarpMessageException;
import laas.openrobots.jORS.exceptions.SimulatorConnectorException;

public class OrsBridge {
	
	private static OrsBridge instance;
	
	IConnector connector;

	private OrsBridge() {
		connector = new YarpConnector("ors");
	}
	
	private static OrsBridge getInstance() {
		if (instance == null)
			instance = new OrsBridge();
		
		return instance;
	}
	
	public static Set<SimulatedRobot> getRobots() {
		return getInstance()._getRobots();
	}
	
	private Set<SimulatedRobot> _getRobots() {
		Set<String>  robots = null;
		try {
			robots = connector.execute("admin", Value.makeValue("getListRobots"), Set.class);
		} catch (MalformedYarpMessageException e) {
			e.printStackTrace();
		} catch (SimulatorConnectorException e) {
			e.printStackTrace();
		}
		
		for (String s : robots)
			System.out.println(s);
		
		return null;
	}
}
