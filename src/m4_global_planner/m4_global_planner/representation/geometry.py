from math import cos, hypot, sin


def footprint_corners(x, y, heading, length, width):
    c, s = cos(heading), sin(heading)

    return [
        (x + c * dx - s * dy, y + s * dx + c * dy)
        for dx, dy in (
            (-length / 2, -width / 2),
            (-length / 2, width / 2),
            (length / 2, width / 2),
            (length / 2, -width / 2),
        )
    ]


def overlaps_box(corners, heading, box):
    xmin, ymin = box['min'][:2]
    xmax, ymax = box['max'][:2]

    box_corners = [
        (xmin, ymin),
        (xmin, ymax),
        (xmax, ymax),
        (xmax, ymin),
    ]

    axes = [
        (1, 0),
        (0, 1),
        (cos(heading), sin(heading)),
        (-sin(heading), cos(heading)),
    ]

    for ax, ay in axes:
        robot = [x * ax + y * ay for x, y in corners]
        obstacle = [x * ax + y * ay for x, y in box_corners]

        if max(robot) < min(obstacle) or max(obstacle) < min(robot):
            return False

    return True


def straight_sweep_corners(start, end, length, width):
    """
    Return the exact horizontal sweep for translation along a fixed heading.

    Caller must first verify aligned motion and unchanged heading.
    """
    distance = hypot(end.x - start.x, end.y - start.y)

    return footprint_corners(
        (start.x + end.x) / 2,
        (start.y + end.y) / 2,
        start.heading,
        length + distance,
        width,
    )


def segment_intersects_region(start, end, region):
    """Check whether an x/y segment touches an axis-aligned region."""
    return segment_region_interval(start, end, region) is not None


def segment_region_interval(start, end, region):
    """Return the covered fraction interval, including boundary contact."""
    return segment_box_interval(
        (start.x, start.y), (end.x, end.y), region['min'][:2], region['max'][:2],
    )


def segment_box_interval(start, end, lower, upper):
    """Slab intersection for a 2D or 3D segment and a closed box."""
    if not len(start) == len(end) == len(lower) == len(upper) or not start:
        raise ValueError('Segment and box dimensions must agree')
    entry, leave = 0.0, 1.0

    for origin, destination, lower, upper in zip(start, end, lower, upper):
        delta = destination - origin

        if delta == 0:
            if origin < lower or origin > upper:
                return None
            continue

        first = (lower - origin) / delta
        second = (upper - origin) / delta

        entry = max(entry, min(first, second))
        leave = min(leave, max(first, second))

        if entry > leave:
            return None

    return entry, leave


def disk_overlaps_box(x, y, radius, box):
    """Check disk overlap with a box projection; touching counts as collision."""
    nearest_x = max(box['min'][0], min(x, box['max'][0]))
    nearest_y = max(box['min'][1], min(y, box['max'][1]))
    return hypot(x - nearest_x, y - nearest_y) <= radius
