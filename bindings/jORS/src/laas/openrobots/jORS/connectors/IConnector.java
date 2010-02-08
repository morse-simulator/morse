/**
 * 
 */
package laas.openrobots.jORS.connectors;

import yarp.Value;

import laas.openrobots.jORS.exceptions.MalformedYarpMessageException;
import laas.openrobots.jORS.exceptions.SimulatorConnectorException;

/**
 * @author slemaign
 *
 */
public interface IConnector {
	
	public abstract void initializeConnector() throws SimulatorConnectorException;
	
	public abstract void finalizeConnector() throws SimulatorConnectorException;
	
	/**
	 * Handle incoming YARP request from the simulator.
	 * @throws MalformedYarpMessageException 
	 */
	//public abstract void run() throws MalformedYarpMessageException;
	
	/**
	 * Send an object to the simulator.
	 * @throws MalformedYarpMessageException 
	 * @throws SimulatorConnectorException 
	 */
	//TODO: Remove this infamous Value from this innocent interface
	public abstract <T> T execute(String port, Value query, Class<T> c) throws MalformedYarpMessageException, SimulatorConnectorException;

}
