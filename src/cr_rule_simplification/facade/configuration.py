from dataclasses import dataclass


@dataclass(frozen=True, kw_only=True)
class Configuration:

    # Rule instantiation parameters
    fov_radius: float = 200

    # Traffic rule parameters
    t_c: float = 3.0
    t_slw: float = 3.0
    t_ia: float = 0.5
    t_ib: float = 1.0

    # Ego vehicle parameters
    a_lon_min: float = -9.5
    a_lon_max: float = 11.5
    a_lat_min: float = -2.0
    a_lat_max: float = 2.0
    v_lon_min: float = -13.9
    v_lon_max: float = 50.8
    v_lat_min: float = -4.0
    v_lat_max: float = 4.0
    length: float = 4.5
    width: float = 1.8
    t_react: float = 0.3

    # Reachability analysis parameters
    uncertainty_p_lon: float = 0.01
    uncertainty_p_lat: float = 0.01
    uncertainty_v_lon: float = 0.01
    uncertainty_v_lat: float = 0.01
