"""
This file contains all the desired positions and other values useful for the
cdf's nodes.
"""
from math import pi

# Plants Islands Entrypoint Positions (x, y, theta)
positions = {
    "island_1_entryPt": (1, 1, pi/2),
    "island_2_entryPt": (1, 1, -pi/2),

    # positions_solar_panel 
    "solar_panel_B1": (1.83, 0.145, 4.18),
    "solar_panel_B2": (1.83, 1., 4.18),

    "solar_panel_J1": (1.83, 2.855, 1.04),
    "solar_panel_J2": (1.83, 2., 1.04),

    # positions_plant_island 
    "I1_center": (0.5, 1.5, 0),
    "I2_center": (0.7, 1., 0),
    "I3_center": (1.3, 1., 0),
    "I4_center": (1.5, 1.5, 0),
    "I5_center": (1.3, 2., 0),
    "I6_center": (0.7, 2., 0),

    # positions_entry_pot 
    "P1_front": (0.6125, 0.35, 3*pi/2),
    "P2_front": (1.3875, 0.35, 3*pi/2),
    "P3_front": (1.65, 1., 0),
    "P4_front": (1.65, 2., 0),
    "P5_front": (1.3875, 2.65, pi/2),
    "P6_front": (0.6125, 2.65, pi/2)
}