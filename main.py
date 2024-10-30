from utils import read_input, write_output, get_k_value
from a_star import a_star_search
import sys


def main():
    """
    main function
    """
    # 1. read input
    input_file = sys.argv[1]
    start, goal, workspace = read_input(input_file)

    # 2. get k value
    k = get_k_value()

    # 3. a* search
    result = a_star_search(start, goal, workspace, k)

    if result:
        path, depth, node_count, f_values, updated_workspace = result
        # 4. write output
        output_file = 'output.txt'
        write_output(output_file, depth, node_count, path, f_values, updated_workspace)
    else:
        print("No path found!")


if __name__ == "__main__":
    main()
