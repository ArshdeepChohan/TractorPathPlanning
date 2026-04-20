import math
from typing import Dict, List, Tuple, Optional

Point = Tuple[float, float]


SHAPE_POINT_COUNTS: Dict[str, int] = {
    "rectangle": 4,
    "square": 4,
    "trapezium": 4,
    "triangle": 3,
    "rhombus": 4,
}


def gps_distance_meters(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    lat1, lon1 = p1
    lat2, lon2 = p2
    avg_lat = math.radians((lat1 + lat2) / 2.0)
    dlat = (lat2 - lat1) * 111320.0
    dlon = (lon2 - lon1) * 111320.0 * math.cos(avg_lat)
    return math.sqrt(dlat ** 2 + dlon ** 2)


def gps_points_to_local(points_gps: List[Tuple[float, float]]) -> List[Point]:
    if not points_gps:
        return []
    lat0, lon0 = points_gps[0]
    local_points: List[Point] = []
    for lat, lon in points_gps:
        avg_lat = math.radians((lat0 + lat) / 2.0)
        x = (lon - lon0) * 111320.0 * math.cos(avg_lat)
        y = (lat - lat0) * 111320.0
        local_points.append((x, y))
    return local_points


def polygon_bbox(polygon: List[Point]) -> Tuple[float, float, float, float]:
    xs = [p[0] for p in polygon]
    ys = [p[1] for p in polygon]
    return min(xs), max(xs), min(ys), max(ys)


def polygon_area(polygon: List[Point]) -> float:
    area = 0.0
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        area += (x1 * y2) - (x2 * y1)
    return 0.5 * area


def ensure_ccw(polygon: List[Point]) -> List[Point]:
    if polygon_area(polygon) < 0:
        return list(reversed(polygon))
    return polygon[:]


def line_polygon_intersections(
    polygon: List[Point],
    axis_value: float,
    direction: str,
    eps: float = 1e-9
) -> List[float]:
    values: List[float] = []
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if direction == "lengthwise":
            dy = y2 - y1
            if abs(dy) < eps:
                continue
            if (axis_value >= min(y1, y2) - eps) and (axis_value < max(y1, y2) - eps):
                t = (axis_value - y1) / dy
                x = x1 + t * (x2 - x1)
                values.append(x)
        else:
            dx = x2 - x1
            if abs(dx) < eps:
                continue
            if (axis_value >= min(x1, x2) - eps) and (axis_value < max(x1, x2) - eps):
                t = (axis_value - x1) / dx
                y = y1 + t * (y2 - y1)
                values.append(y)
    values.sort()
    deduped: List[float] = []
    for val in values:
        if not deduped or abs(val - deduped[-1]) > 1e-6:
            deduped.append(val)
    return deduped


def row_segment_inside_polygon(
    polygon: List[Point],
    axis_value: float,
    direction: str
) -> Optional[Tuple[float, float]]:
    intersections = line_polygon_intersections(polygon, axis_value, direction)
    if len(intersections) < 2:
        return None
    return intersections[0], intersections[-1]


def inward_normal(edge_start: Point, edge_end: Point, polygon_ccw: bool = True) -> Point:
    dx = edge_end[0] - edge_start[0]
    dy = edge_end[1] - edge_start[1]
    length = max(math.hypot(dx, dy), 1e-9)
    if polygon_ccw:
        nx, ny = -dy / length, dx / length
    else:
        nx, ny = dy / length, -dx / length
    return nx, ny


def find_edge_for_point(point: Point, polygon: List[Point], tolerance: float = 1.0) -> Tuple[Point, Point]:
    px, py = point
    best_dist = float("inf")
    best_edge = (polygon[0], polygon[1])
    n = len(polygon)
    for i in range(n):
        a = polygon[i]
        b = polygon[(i + 1) % n]
        ax, ay = a
        bx, by = b
        abx, aby = bx - ax, by - ay
        ab2 = max(abx * abx + aby * aby, 1e-9)
        t = max(0.0, min(1.0, ((px - ax) * abx + (py - ay) * aby) / ab2))
        cx, cy = ax + t * abx, ay + t * aby
        dist = math.hypot(px - cx, py - cy)
        if dist < best_dist:
            best_dist = dist
            best_edge = (a, b)
    if best_dist > tolerance:
        return best_edge
    return best_edge


def smooth_turn_points(
    end_point: Point,
    next_start_point: Point,
    direction_sign: float,
    polygon: List[Point],
    point_spacing: float,
    turn_radius: float
) -> List[Point]:
    ex, ey = end_point
    sx, sy = next_start_point
    row_gap = math.hypot(sx - ex, sy - ey)
    if row_gap < 1e-6:
        return []

    edge_a1, edge_a2 = find_edge_for_point(end_point, polygon)
    edge_b1, edge_b2 = find_edge_for_point(next_start_point, polygon)
    n1 = inward_normal(edge_a1, edge_a2, polygon_ccw=(polygon_area(polygon) > 0))
    n2 = inward_normal(edge_b1, edge_b2, polygon_ccw=(polygon_area(polygon) > 0))

    blend_nx = n1[0] + n2[0]
    blend_ny = n1[1] + n2[1]
    nlen = math.hypot(blend_nx, blend_ny)
    if nlen < 1e-6:
        blend_nx, blend_ny = n1
    else:
        blend_nx, blend_ny = blend_nx / nlen, blend_ny / nlen

    r = min(turn_radius, max(row_gap * 0.5, point_spacing * 2.0))
    control_scale = max(r, row_gap * 0.45)

    c1 = (ex + direction_sign * control_scale * 0.5 + blend_nx * control_scale * 0.35,
          ey + blend_ny * control_scale * 0.55)
    c2 = (sx - direction_sign * control_scale * 0.5 + blend_nx * control_scale * 0.35,
          sy + blend_ny * control_scale * 0.55)

    samples = max(20, int(row_gap / max(point_spacing, 0.2)) * 4)
    points: List[Point] = []
    for i in range(1, samples + 1):
        t = i / samples
        one_minus = 1.0 - t
        x = (
            (one_minus ** 3) * ex
            + 3 * (one_minus ** 2) * t * c1[0]
            + 3 * one_minus * (t ** 2) * c2[0]
            + (t ** 3) * sx
        )
        y = (
            (one_minus ** 3) * ey
            + 3 * (one_minus ** 2) * t * c1[1]
            + 3 * one_minus * (t ** 2) * c2[1]
            + (t ** 3) * sy
        )
        points.append((x, y))
    return points
