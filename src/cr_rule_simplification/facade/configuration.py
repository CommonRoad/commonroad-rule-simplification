from dataclasses import dataclass


@dataclass(frozen=True, kw_only=True)
class Configuration:
    """Configuration parameters for the traffic rule simplification facade."""

    # Rule instantiation parameters
    fov_radius: float = 200
    """Field of view radius for the traffic rule instantiation."""

    # Traffic rule parameters
    t_c: float = 3.0
    """Parameter $t_c$ for the safe distance rule (R_G1) in s."""
    t_slw: float = 3.0
    """Parameter $t_{slw}$ for the stop sign rule (R_IN1) in s."""
    t_ia: float = 0.5
    """Parameter $t_{ia}$ for the "not_endanger_intersection" meta-predicate in s."""
    t_ib: float = 1.0
    """Parameter $t_{ib}$ for the "not_endanger_intersection" meta-predicate in s."""

    # Ego vehicle parameters
    a_lon_min: float = -9.5
    r"""Minimum longitudinal acceleration of the ego vehicle in $\frac{m}{s^2}$."""
    a_lon_max: float = 11.5
    r"""Maximum longitudinal acceleration of the ego vehicle in $\frac{m}{s^2}$."""
    a_lat_min: float = -2.0
    r"""Minimum lateral acceleration of the ego vehicle in $\frac{m}{s^2}$."""
    a_lat_max: float = 2.0
    r"""Maximum lateral acceleration of the ego vehicle in $\frac{m}{s^2}$."""
    v_lon_min: float = -13.9
    r"""Minimum longitudinal velocity of the ego vehicle in $\frac{m}{s}$."""
    v_lon_max: float = 50.8
    r"""Maximum longitudinal velocity of the ego vehicle in $\frac{m}{s}$."""
    v_lat_min: float = -4.0
    r"""Minimum lateral velocity of the ego vehicle in $\frac{m}{s}$."""
    v_lat_max: float = 4.0
    r"""Maximum lateral velocity of the ego vehicle in $\frac{m}{s}$."""
    length: float = 4.5
    """Length of the ego vehicle in m."""
    width: float = 1.8
    """Width of the ego vehicle in m."""
    t_react: float = 0.3
    """Reaction time of the ego vehicle in s."""

    # Reachability analysis parameters
    uncertainty_p_lon: float = 0.01
    """Initial uncertainty in the longitudinal position of the ego vehicle in m."""
    uncertainty_p_lat: float = 0.01
    """Initial uncertainty in the lateral position of the ego vehicle in m."""
    uncertainty_v_lon: float = 0.01
    r"""Initial uncertainty in the longitudinal velocity of the ego vehicle in $\frac{m}{s}$."""
    uncertainty_v_lat: float = 0.01
    r"""Initial uncertainty in the lateral velocity of the ego vehicle in $\frac{m}{s}$."""
