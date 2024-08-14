import math
import heapq


class Node:
    def __init__(self, x, y, cost=0, parent=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent

    def __lt__(self, other):
        return self.cost < other.cost


class PathPlanning:

    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.open_list = []
        self.closed_list = set()

    def heuristic(self, node):
        return math.sqrt((node.x - self.goal.x) ** 2 + (node.y - self.goal.y) ** 2)

    def get_neighbors(self, node):
        neighbors = []
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        for direction in directions:
            new_x, new_y = node.x + direction[0], node.y + direction[1]
            if (
                0 <= new_x < len(self.grid)
                and 0 <= new_y < len(self.grid[0])
                and self.grid[new_x][new_y] == 0
            ):
                neighbors.append(Node(new_x, new_y))

        return neighbors

    def reconstruct_path(self, current_node):
        path = []
        while current_node is not None:
            path.append((current_node.x, current_node.y))
            current_node = current_node.parent
        return path[::-1]

    def a_star(self):
        heapq.heappush(self.open_list, (0, self.start))
        self.start.cost = 0

        while self.open_list:
            _, current_node = heapq.heappop(self.open_list)

            if (current_node.x, current_node.y) in self.closed_list:
                continue

            if current_node.x == self.goal.x and current_node.y == self.goal.y:
                return self.reconstruct_path(current_node)

            self.closed_list.add((current_node.x, current_node.y))

            for neighbor in self.get_neighbors(current_node):
                if (neighbor.x, neighbor.y) in self.closed_list:
                    continue

                tentative_cost = current_node.cost + 1

                if tentative_cost < neighbor.cost or neighbor not in [
                    i[1] for i in self.open_list
                ]:
                    neighbor.cost = tentative_cost
                    priority = tentative_cost + self.heuristic(neighbor)
                    heapq.heappush(self.open_list, (priority, neighbor))
                    neighbor.parent = current_node

        return None  # No path found
