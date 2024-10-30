import heapq
from utils import calculate_cost, heuristic, calculate_angle, MOVES


class State:
    """
    State class representing a cell in the workspace.
    """

    def __init__(self, position, g, h, f, parent, action, direction):
        self.position = position  # (i, j) coordinates
        self.g = g  # Actual cost from start to the current state
        self.h = h  # Heuristic value
        self.f = f  # Total cost f = g + h
        self.parent = parent  # Parent node for path tracing
        self.action = action  # Action identifier from parent to current state
        self.direction = direction  # Current movement direction (action identifier)

    def __lt__(self, other):
        return self.f < other.f


def a_star_search(start, goal, workspace, k):
    """
    A* search algorithm implementation. Returns the solution path and relevant metrics.

    Returns:
        path: Sequence of actions from start to goal
        depth: Depth of the goal node (number of actions)
        node_count: Total number of nodes generated
        f_values: f(n) values for nodes along the solution path
        updated_workspace: Workspace grid with the solution path marked
    """
    open_list = []
    closed_set = set()
    node_count = 0

    start_state = State(position=start, g=0, h=heuristic(start, goal),
                        f=heuristic(start, goal), parent=None, action=None, direction=None)
    heapq.heappush(open_list, start_state)

    while open_list:
        current_state = heapq.heappop(open_list)
        # no repeated states allowed
        if current_state.position in closed_set:
            continue
        
        # Check if the goal has been reached
        if current_state.position == goal:
            path, f_values_path = reconstruct_path(current_state)
            depth = len(path)  # Depth is the number of actions
            updated_workspace = mark_path(workspace, path, start, goal)
            return path, depth, node_count, f_values_path, updated_workspace

        closed_set.add(current_state.position)

        for action, move in MOVES.items():
            neighbor_pos = (current_state.position[0] + move[0],
                            current_state.position[1] + move[1])

            # Check if the position is valid and not already visited
            if not is_valid_position(neighbor_pos, workspace):
                continue
            if neighbor_pos in closed_set:
                continue

            # Calculate angle difference for cost computation
            if current_state.direction is None:
                delta_theta = 0
            else:
                delta_theta = calculate_angle(current_state.direction, action)

            # Calculate the cost of moving to the neighboring position
            step_cost = calculate_cost(delta_theta, action, move, k)
            tentative_g = current_state.g + step_cost
            tentative_h = heuristic(neighbor_pos, goal)
            tentative_f = tentative_g + tentative_h

            # Skip if a better or equal path already exists in the open list
            existing_state = find_state_in_open(open_list, neighbor_pos)
            if existing_state and tentative_g >= existing_state.g:
                continue

            # Create a new state and add it to the open list
            neighbor_state = State(position=neighbor_pos, g=tentative_g, h=tentative_h,
                                   f=tentative_f, parent=current_state, action=action, direction=action)
            heapq.heappush(open_list, neighbor_state)

            if existing_state == None:
                node_count += 1

    # Return None if no path was found
    return None


def reconstruct_path(goal_state):
    """
    Reconstructs the path from the goal state back to the start state, collecting
    actions and f(n) values along the way.

    Returns:
        path: List of actions from start to goal
        f_values: List of f(n) values along the path
    """
    path = []
    f_values = []
    current = goal_state
    while current.parent is not None:
        path.append(current.action)
        f_values.append(current.f)
        current = current.parent
    path.reverse()
    f_values.reverse()
    return path, f_values


def mark_path(workspace, path, start, goal):
    """
    Marks the solution path in the workspace by setting cells to '4' along the path.

    Returns:
        updated_workspace: Workspace grid with the path marked
    """
    updated_workspace = [row[:] for row in workspace]  # Deep copy
    current_pos = start
    for action in path:
        move = MOVES[action]
        current_pos = (current_pos[0] + move[0], current_pos[1] + move[1])
        if current_pos != goal:
            updated_workspace[current_pos[1]][current_pos[0]] = 4  # workspace[j][i]
    return updated_workspace


def is_valid_position(position, workspace):
    """
    Checks if the position is within the bounds and not an obstacle.
    """
    i, j = position
    if 0 <= i < len(workspace[0]) and 0 <= j < len(workspace):
        return workspace[j][i] != 1  # workspace[j][i], where j is row, i is column
    return False


def find_state_in_open(open_list, position):
    """
    Searches for a state with the specified position in the open list.
    """
    for state in open_list:
        if state.position == position:
            return state
    return None
