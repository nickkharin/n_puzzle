from collections import deque
import heapq


class State:
    def __init__(self):
        raise NotImplementedError("Must be implemented by subclass")

    def get_applicable_actions(self):
        raise NotImplementedError("Must be implemented by subclass")

    def get_action_result(self, action):
        raise NotImplementedError("Must be implemented by subclass")

    def __eq__(self, other):
        raise NotImplementedError("Must be implemented by subclass")

    def __hash__(self):
        raise NotImplementedError("Must be implemented by subclass")


class Search:
    def __init__(self, frontier=None, heuristic=None):
        self.frontier = frontier
        self.heuristic = heuristic
        self.nodes_generated = 0
        self.max_frontier_size = 0

    def find_solution(self, initial_state, goal_test):
        raise NotImplementedError("Must be implemented by subclass")

    def print_statistics(self):
        print(f"Total number of nodes generated: {self.nodes_generated}")


class TreeSearch(Search):
    def find_solution(self, initial_state, goal_test):
        self.frontier.clear()
        self.frontier.add(Node(None, None, initial_state))
        while not self.frontier.is_empty():
            node = self.frontier.remove()
            if goal_test.is_goal(node.state):
                return node
            for action in node.state.get_applicable_actions():
                child_state = node.state.get_action_result(action)
                child_node = Node(node, action, child_state)
                self.frontier.add(child_node)
        return None


class GraphSearch(Search):
    def find_solution(self, initial_state, goal_test):
        self.frontier.clear()
        self.frontier.add(Node(None, None, initial_state))
        explored = set()
        while not self.frontier.is_empty():
            node = self.frontier.remove()
            if goal_test.is_goal(node.state):
                return node
            explored.add(node.state)
            for action in node.state.get_applicable_actions():
                child_state = node.state.get_action_result(action)
                if child_state not in explored:
                    child_node = Node(node, action, child_state)
                    self.frontier.add(child_node)
        return None


class IterativeDeepeningTreeSearch(Search):
    def find_solution(self, initial_state, goal_test):
        depth_limit = 0
        while True:
            self.frontier = DepthFirstFrontier()
            self.frontier.add(Node(None, None, initial_state, 0))
            while not self.frontier.is_empty():
                node = self.frontier.remove()
                if goal_test.is_goal(node.state):
                    return node
                if node.depth < depth_limit:
                    for action in node.state.get_applicable_actions():
                        child_state = node.state.get_action_result(action)
                        child_node = Node(node, action, child_state, node.depth + 1)
                        self.frontier.add(child_node)
            depth_limit += 1


class Frontier:
    def add(self, node):
        raise NotImplementedError

    def clear(self):
        raise NotImplementedError

    def is_empty(self):
        raise NotImplementedError

    def remove(self):
        raise NotImplementedError


class DepthFirstFrontier(Frontier):
    def __init__(self):
        self.stack = []
        self.size = 0

    def add(self, node):
        self.stack.append(node)
        self.size += 1  # Update size on adding node

    def clear(self):
        self.stack.clear()

    def is_empty(self):
        return self.size == 0

    def remove(self):
        if not self.is_empty():
            self.size -= 1
            return self.stack.pop()
        return None


class BreadthFirstFrontier:
    def __init__(self):
        self.queue = deque()
        self.size = 0

    def remove(self):
        if not self.is_empty():
            self.size -= 1
            return self.queue.popleft()
        return None

    def clear(self):
        self.queue.clear()

    def add(self, node):
        self.queue.append(node)
        self.size += 1

    def is_empty(self):
        return self.size == 0


class NodeFunction:
    def evaluate(self, node):
        raise NotImplementedError("Must be implemented by subclass")


class BestFirstFrontier(Frontier):
    def __init__(self, node_function):
        self.heap = []
        self.size = 0
        self.node_function = node_function
        heapq.heapify(self.heap)

    def add(self, node):
        heapq.heappush(self.heap, (self.node_function.evaluate(node), node))
        self.size += 1

    def clear(self):
        self.heap.clear()

    def is_empty(self):
        return self.size == 0

    def remove(self):
        if not self.is_empty():
            self.size -= 1
            return heapq.heappop(self.heap)[1]
        return None


class AStarFunction(NodeFunction):
    def __init__(self, heuristic_function):
        self.heuristic_function = heuristic_function

    def evaluate(self, node):
        g = node.path_cost
        h = self.heuristic_function(node)
        return g + h


class UCSFunction(NodeFunction):
    def evaluate(self, node):
        return node.path_cost


class Action:
    def __init__(self):
        raise NotImplementedError("Must be implemented by subclass")

    def cost(self):
        raise NotImplementedError("Must be implemented by subclass")


class GoalTest:
    def is_goal(self, state):
        raise NotImplementedError("Must be implemented by subclass")


class Node:
    def __init__(self, parent, action, state):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = 0

    def __lt__(self, other):
        return self.path_cost < other.path_cost


class Printing:
    @classmethod
    def print_solution(cls, solution):
        if not solution:
            print('No solution found!')
        else:
            stack = deque()
            node = solution
            while node:
                stack.append(node)
                node = node.parent
            stepNo = 0
            while stack:
                node = stack.pop()
                print(stepNo, end='')
                stepNo += 1
                if not node.parent:
                    print(': start')
                else:
                    print(': ', end='')
                    cls.print_action(node.action)
                    print()
                print()
                cls.print_state(node.state)
                print()

    @staticmethod
    def print_action(action):
        pass

    @staticmethod
    def print_state(state):
        pass


class BreadthFirstTreeSearch(Search):
    def find_solution(self, initial_state, goal_test):
        self.frontier.clear()
        self.nodes_generated = 0
        self.max_frontier_size = 0

        node = Node(None, None, initial_state)
        self.frontier.add(node)
        self.nodes_generated += 1

        while not self.frontier.is_empty():
            self.max_frontier_size = max(self.max_frontier_size, self.frontier.size)

            node = self.frontier.remove()
            if goal_test.is_goal(node.state):
                return node

            for action in node.state.get_applicable_actions():
                self.nodes_generated += 1
                child_state = node.state.get_action_result(action)
                child_node = Node(node, action, child_state)
                self.frontier.add(child_node)

        return None


class DepthFirstTreeSearch(Search):
    def find_solution(self, initial_state, goal_test):
        self.frontier.clear()
        self.nodes_generated = 0
        self.max_frontier_size = 0

        frontier = [Node(None, None, initial_state)]
        self.nodes_generated += 1
        while frontier:
            node = frontier.pop(0)
            if goal_test.is_goal(node.state):
                return node
            frontier.extend(Node(node, action, node.state.get_action_result(action))
                            for action in node.state.get_applicable_actions())
        return None


class AStarGraphSearch(Search):
    def find_solution(self, initial_state, goal_test):
        self.nodes_generated = 0
        frontier = []
        heapq.heappush(frontier, (0, Node(None, None, initial_state)))
        self.nodes_generated += 1
        explored = set()
        while frontier:
            _, node = heapq.heappop(frontier)
            if goal_test.is_goal(node.state):
                return node
            explored.add(node.state)
            for action in node.state.get_applicable_actions():
                child_state = node.state.get_action_result(action)
                if child_state not in explored:
                    child_node = Node(node, action, child_state)
                    child_node.heuristic_value = self.heuristic(child_node)
                    heapq.heappush(frontier, (child_node.path_cost + child_node.heuristic_value, child_node))
        return None


class AStarTreeSearch(Search):
    def find_solution(self, initial_state, goal_test):
        self.nodes_generated = 0
        MAX_DEPTH = 1000
        depth = 0
        frontier = []
        heapq.heappush(frontier, (0, Node(None, None, initial_state)))
        self.nodes_generated += 1
        while frontier and depth < MAX_DEPTH:
            _, node = heapq.heappop(frontier)
            if goal_test.is_goal(node.state):
                return node
            for action in node.state.get_applicable_actions():
                child_state = node.state.get_action_result(action)
                child_node = Node(node, action, child_state)
                child_node.heuristic_value = self.heuristic(child_node)
                heapq.heappush(frontier, (child_node.path_cost + child_node.heuristic_value, child_node))
                depth += 1
        return None


class UniformCostTreeSearch(Search):
    def find_solution(self, initial_state, goal_test):
        self.nodes_generated = 0
        MAX_DEPTH = 1000
        depth = 0
        frontier = []
        heapq.heappush(frontier, (0, Node(None, None, initial_state)))
        self.nodes_generated += 1
        while frontier and depth < MAX_DEPTH:
            _, node = heapq.heappop(frontier)
            if goal_test.is_goal(node.state):
                return node
            for action in node.state.get_applicable_actions():
                child_state = node.state.get_action_result(action)
                child_node = Node(node, action, child_state)
                cost = node.path_cost + action.cost()
                heapq.heappush(frontier, (cost, child_node))
                self.nodes_generated += 1
                depth += 1
        return None


class UniformCostGraphSearch(Search):
    def find_solution(self, initial_state, goal_test):
        self.nodes_generated = 0
        frontier = []
        visited = set()  # Track visited states for UCS on graphs
        heapq.heappush(frontier, (0, Node(None, None, initial_state)))
        self.nodes_generated += 1
        while frontier:
            _, node = heapq.heappop(frontier)
            if goal_test.is_goal(node.state):
                return node
            if node.state in visited:
                continue
            visited.add(node.state)
            for action in node.state.get_applicable_actions():
                child_state = node.state.get_action_result(action)
                child_node = Node(node, action, child_state)
                cost = node.path_cost + action.cost()
                heapq.heappush(frontier, (cost, child_node))
                self.nodes_generated += 1
        return None


def manhattan_distance(node):
    state = node.state
    width = state.width
    total_distance = 0
    for row in range(width):
        for col in range(width):
            tile = state.get_tile(row, col)
            if tile != 0:
                goal_row = (tile - 1) // width
                goal_col = (tile - 1) % width
                distance = abs(row - goal_row) + abs(col - goal_col)
                total_distance += distance
    return total_distance


def misplaced_tiles(node):
    state = node.state
    width = state.width
    misplaced = 0
    for row in range(width):
        for col in range(width):
            tile = state.get_tile(row, col)
            goal_tile = row * width + col + 1
            if tile != 0 and tile != goal_tile:
                misplaced += 1
    return misplaced
