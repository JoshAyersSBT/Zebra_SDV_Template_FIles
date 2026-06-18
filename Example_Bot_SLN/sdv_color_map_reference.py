# SDV Game Mat Color Map Reference
# This file is a plain reference for students and coaches.
# The challenge programs remain standalone and do not import this file.
#
# Source: SDV_GM_v3.pdf, Fig. 5 "Grand loop Ride".
#
# Grid naming:
# - Rows are A through E from top to bottom.
# - Columns are 1 through 5 from left to right.
# - Example: A1 is top-left, E5 is bottom-right.


# Vertical grid lines separate columns.
# If the robot moves east or west between two numbered columns, it crosses one
# of these colors.
COLUMN_CROSSING_COLORS = {
    (1, 2): "green",
    (2, 3): "cyan",
    (3, 4): "brown",
    (4, 5): "magenta",
}


# Horizontal grid lines separate rows.
# If the robot moves south or north between two lettered rows, it crosses one
# of these colors.
ROW_CROSSING_COLORS = {
    ("A", "B"): "yellow",
    ("B", "C"): "blue",
    ("C", "D"): "red",
    ("D", "E"): "purple",
}


# Legal Format A outer loop squares in clockwise order.
FORMAT_A_OUTER_LOOP_CLOCKWISE = (
    "A1", "A2", "A3", "A4", "A5",
    "B5", "C5", "D5", "E5",
    "E4", "E3", "E2", "E1",
    "D1", "C1", "B1",
)


FORMAT_A_CORNERS = ("A1", "A5", "E5", "E1")


def crossing_color(from_grid, to_grid):
    """Return the expected line color between two adjacent grid squares."""
    from_row = from_grid[0]
    to_row = to_grid[0]
    from_col = int(from_grid[1])
    to_col = int(to_grid[1])

    if from_row == to_row:
        low = min(from_col, to_col)
        high = max(from_col, to_col)
        return COLUMN_CROSSING_COLORS.get((low, high))

    if from_col == to_col:
        pair = tuple(sorted((from_row, to_row)))
        return ROW_CROSSING_COLORS.get(pair)

    return None


FORMAT_A_OUTER_LOOP_EXPECTED_COLORS = {
    "A1>A2": "green",
    "A2>A3": "cyan",
    "A3>A4": "brown",
    "A4>A5": "magenta",
    "A5>B5": "yellow",
    "B5>C5": "blue",
    "C5>D5": "red",
    "D5>E5": "purple",
    "E5>E4": "magenta",
    "E4>E3": "brown",
    "E3>E2": "cyan",
    "E2>E1": "green",
    "E1>D1": "purple",
    "D1>C1": "red",
    "C1>B1": "blue",
    "B1>A1": "yellow",
}
