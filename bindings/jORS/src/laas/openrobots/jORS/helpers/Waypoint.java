package laas.openrobots.jORS.helpers;

import laas.openrobots.jORS.connectors.YarpSerializable;

public interface Waypoint extends YarpSerializable {

	public double getX();
	public double getY();
	public double getZ();
	
	public double getVX();
	public double getVY();
	public double getVZ();
}
