package laas.openrobots.jORS.helpers;

import yarp.Value;

public class WaypointImpl implements Waypoint {

	private double x, y, z, vx, vy, vz;
	
	public WaypointImpl(double x, double y) {
		super();
		this.x = x;
		this.y = y;
		this.z = 0; //not used yet
		this.vx = 0; //not used yet
		this.vy = 0; //not used yet
		this.vz = 0; //not used yet
	}

	@Override
	public Value toYarp() {
		return Value.makeList(x + " " + y + " " + z);
	}

	@Override
	public double getVX() {
		return vx;
	}

	@Override
	public double getVY() {
		return vy;
	}

	@Override
	public double getVZ() {
		return vz;
	}

	@Override
	public double getX() {
		return x;
	}

	@Override
	public double getY() {
		return y;
	}

	@Override
	public double getZ() {
		return z;
	}

}
