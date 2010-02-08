package laas.openrobots.jORS.helpers;

import yarp.Value;
import laas.openrobots.jORS.connectors.YarpSerializable;

public class DumbYarpConverter<T> implements YarpSerializable {

	T o;
	
	@Override
	public Value toYarp() {
		return Value.makeValue(o.toString());
	}

}
