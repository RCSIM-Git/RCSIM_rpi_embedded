"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.

Narzędzia matematyczne do obsługi mapy opartej na kafelkach (slippy map).
Mathematical tools for tile-based map handling (slippy map).
"""

import math

TILE_SIZE = 256


def deg_to_num(lat_deg: float, lon_deg: float, zoom: int) -> tuple[int, int]:
    """
    Przelicza współrzędne geograficzne na numer kafelka (x, y) dla danego zoomu.
    Converts geographic coordinates to tile number (x, y) for a given zoom level.

    Args:
        lat_deg (float): Szerokość geograficzna w stopniach. / Latitude in degrees.
        lon_deg (float): Długość geograficzna w stopniach. / Longitude in degrees.
        zoom (int): Poziom przybliżenia. / Zoom level.

    Returns:
        tuple[int, int]: Numer kafelka (x, y). / Tile number (x, y).
    """
    lat_rad = math.radians(lat_deg)
    n = 2.0**zoom
    xtile = int((lon_deg + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return xtile, ytile


def get_pixel_offset(
    lat_deg: float, lon_deg: float, zoom: int
) -> tuple[int, int, int, int]:
    """
    Oblicza kafelek centralny i przesunięcie w pikselach pojazdu względem lewego górnego rogu tego kafelka.
    Calculates the center tile and the pixel offset of the vehicle relative to the top-left corner of that tile.

    Args:
        lat_deg (float): Szerokość geograficzna w stopniach. / Latitude in degrees.
        lon_deg (float): Długość geograficzna w stopniach. / Longitude in degrees.
        zoom (int): Poziom przybliżenia. / Zoom level.

    Returns:
        tuple[int, int, int, int]:
            center_tile_x, center_tile_y, pixel_x_offset, pixel_y_offset
    """
    lat_rad = math.radians(lat_deg)
    n = 2.0**zoom

    # Dokładna pozycja na mapie w jednostkach kafelków
    # Exact position on the map in tile units
    exact_x_tile = (lon_deg + 180.0) / 360.0 * n
    exact_y_tile = (1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n

    # Numer kafelka, w którym znajduje się punkt
    # The tile number where the point is located
    center_tile_x = int(exact_x_tile)
    center_tile_y = int(exact_y_tile)

    # Przesunięcie w pikselach wewnątrz tego kafelka
    # Pixel offset within this tile
    pixel_x_offset = int((exact_x_tile - center_tile_x) * TILE_SIZE)
    pixel_y_offset = int((exact_y_tile - center_tile_y) * TILE_SIZE)

    return center_tile_x, center_tile_y, pixel_x_offset, pixel_y_offset


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Oblicza dystans w metrach między dwoma punktami GPS.
    Calculates the distance in meters between two GPS points.

    Args:
        lat1 (float): Szerokość geograficzna punktu startowego. / Latitude of the starting point.
        lon1 (float): Długość geograficzna punktu startowego. / Longitude of the starting point.
        lat2 (float): Szerokość geograficzna punktu docelowego. / Latitude of the target point.
        lon2 (float): Długość geograficzna punktu docelowego. / Longitude of the target point.

    Returns:
        float: Dystans między punktami w metrach. / Distance between points in meters.
    """
    R = 6371000  # Promień Ziemi w metrach / Radius of Earth in meters
    lat1_rad, lon1_rad = math.radians(lat1), math.radians(lon1)
    lat2_rad, lon2_rad = math.radians(lat2), math.radians(lon2)
    d_lat = lat2_rad - lat1_rad
    d_lon = lon2_rad - lon1_rad
    a = (
        math.sin(d_lat / 2) ** 2
        + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(d_lon / 2) ** 2
    )
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c


def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Oblicza azymut (bearing) między dwoma punktami GPS.
    Calculates the bearing between two GPS points.

    Args:
        lat1 (float): Szerokość geograficzna punktu startowego. / Latitude of the starting point.
        lon1 (float): Długość geograficzna punktu startowego. / Longitude of the starting point.
        lat2 (float): Szerokość geograficzna punktu docelowego. / Latitude of the target point.
        lon2 (float): Długość geograficzna punktu docelowego. / Longitude of the target point.

    Returns:
        float: Azymut w stopniach [0, 360). / Bearing in degrees [0, 360).
    """
    lat1_rad, lon1_rad = math.radians(lat1), math.radians(lon1)
    lat2_rad, lon2_rad = math.radians(lat2), math.radians(lon2)
    d_lon = lon2_rad - lon1_rad
    x = math.sin(d_lon) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(
        lat2_rad
    ) * math.cos(d_lon)
    bearing = math.degrees(math.atan2(x, y))
    return (bearing + 360) % 360
