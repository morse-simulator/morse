import logging; logger = logging.getLogger("morse." + __name__)


from math import sin, cos, asin, atan2, sqrt, degrees, radians

earth_radius = 6371000.0

# from http://www.movable-type.co.uk/scripts/latlong.html
def distance_and_direction(lat_start_degs, lon_start_degs, lat_finish_degs, lon_finish_degs):
    """
    calculate the distance in meters and direction in degrees from start lat/lon to finish lat/lon.
    """
    

    if (lat_start_degs == lat_finish_degs and lon_start_degs == lon_finish_degs):
        return 0, 0

    lat_s = radians(lat_start_degs)
    lon_s = radians(lon_start_degs)
    lat_f = radians(lat_finish_degs)
    lon_f = radians(lon_finish_degs)
    lat_d = lat_f - lat_s
    lon_d = lon_f - lon_s

    R = earth_radius

    t1 = sin(lat_d/2)
    t2 = sin(lon_d/2)
    distance = R*2*asin(sqrt(t1*t1 + cos(lat_s)*cos(lat_f)*t2*t2))

    y = sin(lon_d)*cos(lat_f)
    x = cos(lat_s)*sin(lat_f) - sin(lat_s)*cos(lat_f)*cos(lon_d)
    heading = degrees(atan2(y, x))

    return distance, heading

def point_from_distance_and_heading(lat_degs, lon_degs, distance, heading_rads):
    """
    calculate a lat/lon starting from a given lat/lon and
    travelling in a given distance in meters and direction in radians.
    """

    lat_s = radians(lat_degs)
    lon_s = radians(lon_degs)
    t = heading_rads
    d = distance
    R = earth_radius

    lat_f = asin(sin(lat_s)*cos(d/R) + cos(lat_s)*sin(d/R)*cos(t))
    lon_f = lon_s + atan2(sin(t)*sin(d/R)*cos(lat_s), cos(d/R) - sin(lat_s)*sin(lat_f))

    return degrees(lat_f), degrees(lon_f)



