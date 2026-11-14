"""Magnetic declination of ProjectAirSim's default dipole sensor model.

Mirrors core_sim/include/core_sim/earth_utils.hpp, not PX4's WMM model.
Use only with the default dipole magnetometer of the verified simulator build.
"""
import math


def dipole_declination_deg(latitude, longitude):
    latitude, longitude = float(latitude), float(longitude)
    if not math.isfinite(latitude) or not -90 <= latitude <= 90:
        raise ValueError('invalid latitude')
    if not math.isfinite(longitude) or not -180 <= longitude <= 180:
        raise ValueError('invalid longitude')
    lat, lon = math.radians(latitude), math.radians(longitude)
    pole_lat, pole_lon = math.radians(80.31), math.radians(-72.62)
    cos_colat = (math.cos(lat) * math.cos(pole_lat)
                 + math.sin(lat) * math.sin(pole_lat) * math.cos(lon - pole_lon))
    colat = math.acos(max(-1., min(1., cos_colat)))
    factor = math.cos(pole_lat) / math.sin(colat)
    azimuth = (math.sin(lon - pole_lon) if cos_colat > math.sin(pole_lat) * math.sin(lat)
               else math.cos(lon - pole_lon))
    return math.degrees(math.asin(max(-1., min(1., azimuth * factor))))
