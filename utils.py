import math

"""
Team members:
Zan Ni (zn2161)
Siqi Wan (sw6195)
"""

# MOVES dictionary adjusted to align with the (i, j) coordinate system and new direction numbering
MOVES = {
    0: (1, 0),  # Move right
    1: (1, 1),  # Move right-up
    2: (0, 1),  # Move up
    3: (-1, 1),  # Move left-up
    4: (-1, 0),  # Move left
    5: (-1, -1),  # Move left-down
    6: (0, -1),  # Move down
    7: (1, -1)  # Move right-down
}


def read_input(file_path):
    """
    Reads the input file and returns the start, goal positions, and workspace grid.

    Returns:
        start: (i, j) coordinates of the start position
        goal: (i, j) coordinates of the goal position
        workspace: 2D list representing the workspace grid
    """
    workspace = []
    with open(file_path, 'r') as file:
        lines = file.readlines()
        # Line 1: Start and goal coordinates
        start_goal = list(map(int, lines[0].strip().split()))
        start = (start_goal[0], start_goal[1])
        goal = (start_goal[2], start_goal[3])
        # Lines 2-31: Workspace grid representation
        for line in lines[1:31]:
            row = list(map(int, line.strip().split()))
            workspace.append(row)
    # Reverse the workspace so that j=0 is the bottom row
    workspace = workspace[::-1]
    return start, goal, workspace


# utils.py

def write_output(file_path, depth, node_count, path, f_values, workspace):
    """
    Writes the output to the specified file in the required format.

    Parameters:
        file_path (str): Path to the output file.
        depth (int): Depth of the goal node (number of actions).
        node_count (int): Total number of nodes generated.
        path (list): Sequence of actions from start to goal.
        f_values (list): f(n) values for nodes along the solution path.
        workspace (list of lists): Workspace grid with the solution path marked.
    """
    # Reverse the workspace to match the input coordinate system (j=0 at the bottom)
    workspace_to_write = workspace[::-1]

    with open(file_path, 'w') as file:
        # First line: Depth (integer)
        file.write(f"{depth}\n")
        # Second line: Total number of nodes generated
        file.write(f"{node_count}\n")
        # Third line: Action sequence
        path_str = ' '.join(map(str, path))
        file.write(f"{path_str}\n")
        # Fourth line: f(n) values
        f_values_str = ' '.join([f"{fn:.2f}" for fn in f_values])
        file.write(f"{f_values_str}\n")
        # Lines 5-34: Workspace grid with path marked as '4'
        for row in workspace_to_write:
            row_str = ' '.join(map(str, row))
            file.write(f"{row_str}\n")


def calculate_angle(theta1, theta2):
    """
    Calculates the minimal angle difference between two directions.

    Parameters:
        theta1: Previous direction (action number)
        theta2: Current direction (action number)

    Returns:
        delta_theta: Minimal angle difference in degrees (0-180)
    """
    angle_diff = abs(theta2 - theta1) * 45  # Each action is 45 degrees apart
    if angle_diff > 180:
        angle_diff = 360 - angle_diff
    return angle_diff


def heuristic(current, goal):
    """
    Calculates the Euclidean distance heuristic between current position and goal.

    Parameters:
        current: (i, j) current position
        goal: (i, j) goal position

    Returns:
        Euclidean distance
    """
    return math.sqrt((goal[0] - current[0]) ** 2 + (goal[1] - current[1]) ** 2)


def calculate_cost(delta_theta, action, move, k):
    """
    Calculates the total movement cost c(s, a, s') = ca + cd.

    Parameters:
        delta_theta: Change in direction in degrees
        action: Current action number
        move: (Δi, Δj) movement vector
        k: Angle cost control constant

    Returns:
        Total cost
    """
    if delta_theta == 0 and action is None:
        ca = 0  # Angle cost for the start position is set to 0
    else:
        ca = k * (delta_theta / 180)

    # Calculate distance cost
    if move in [(1, 0), (0, 1), (-1, 0), (0, -1)]:
        cd = 1
    else:
        cd = math.sqrt(2)

    return ca + cd


def get_k_value():
    """
    get k from the user input.

    return:
        k value
    """
    while True:
        try:
            k = float(input("Please enter the k value: "))
            return k
        except ValueError:
            print("Invalid input! Please enter a number.")
