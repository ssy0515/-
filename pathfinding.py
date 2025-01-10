import heapq
import math
import time
import ast
from collections import defaultdict

class PathfindingGraph:
    def __init__(self):
        self.G = defaultdict(dict)
        self.start_node = 1
        self.end_node = 4
        self.obstacles = set()
        self.algorithm = "A*"
        self.heuristic_cache = {}
        self.positions = self.load_positions("C:\\Users\\佘思源\\Desktop\\应急路线\\point.txt")
        for node in range(1, 51):
            self.G[node] = {}

        self.load_edges("C:\\Users\\佘思源\\Desktop\\应急路线\\edge.txt")
        self.impact_factors = self.load_impact_factors("C:\\Users\\佘思源\\Desktop\\应急路线\\impact.txt")

    def load_positions(self, file_path):
        positions = {}
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                for line in file:
                    line = line.strip()
                    node_id_str, coords_str = line.split(":")
                    node_id = int(node_id_str.strip())
                    coords = ast.literal_eval(coords_str.strip())
                    positions[node_id] = coords
        except Exception as e:
            print(f"读取文件时出错: {e}")
        return positions

    def load_edges(self, file_path):
        try:
            with open(file_path, 'r') as file:
                for line in file:
                    line = line.replace(',', '')
                    u, v, weight = map(int, line.split())
                    self.G[u][v] = {'weight': weight}
                    self.G[v][u] = {'weight': weight}
        except Exception as e:
            print(f"读取文件时出错: {e}")

    def load_impact_factors(self, file_path):
        impact_factors = {}
        try:
            with open(file_path, 'r') as file:
                for line in file:
                    line = line.strip()
                    if line:
                        node1, node2, impact_factor = line.split(', ')
                        impact_factors[(int(node1), int(node2))] = float(impact_factor)
        except Exception as e:
            print(f"读取文件时出错: {e}")
        return impact_factors

    def get_positions(self):
        return self.positions

    def add_obstacle(self, u, v):
        if u in self.G and v in self.G[u]:
            self.obstacles.add((u, v))
            self.obstacles.add((v, u))
        else:
            print(f"边 ({u}, {v}) 不存在，无法添加为障碍物")

    def clear_obstacles(self):
        self.obstacles.clear()

    def set_algorithm(self, algorithm):
        self.algorithm = algorithm

    def run_pathfinding(self, start, ends):
        if self.algorithm == "A*":
            return self.a_star(start, ends)
        elif self.algorithm == "Dijkstra":
            return self.dijkstra(start, ends)
        elif self.algorithm == "Combined":
            return self.combined_algorithm(start, ends)

    def a_star(self, start, ends):
        start_time = time.time()
        print(f"Executing A* algorithm from {start} to {ends}")

        dist = {node: float('inf') for node in self.G}
        dist[start] = 0
        prev = {node: None for node in self.G}
        pq = [(self.heuristic(start, ends), start)]
        found_targets = set()

        while pq:
            current_f, current_node = heapq.heappop(pq)
            current_dist = dist[current_node]

            if current_node in ends:
                found_targets.add(current_node)
                if found_targets == set(ends):
                    break

            for neighbor, data in self.G[current_node].items():
                if (current_node, neighbor) in self.obstacles:
                    continue

                weight = data['weight']
                impact_factor = data.get('impact_factor', 1)

                g = dist[current_node] + weight * impact_factor
                h = self.heuristic(neighbor, ends)
                f = g + h

                if g < dist[neighbor]:
                    dist[neighbor] = g
                    prev[neighbor] = current_node
                    heapq.heappush(pq, (f, neighbor))

        paths = []
        for end in ends:
            path = []
            current = end
            while current is not None:
                path.append(current)
                current = prev[current]
            if path:
                paths.append((path[::-1], dist[end]))
            else:
                paths.append(([], float('inf')))

        end_time = time.time()
        print(f"A* algorithm execution time: {end_time - start_time:.4f} seconds")

        return paths

    def dijkstra(self, start, ends):
        start_time = time.perf_counter()
        print(f"Executing Dijkstra algorithm from {start} to {ends}")

        dist = {node: float('inf') for node in self.G}
        dist[start] = 0
        prev = {node: None for node in self.G}
        pq = [(0, start)]
        found_targets = set()

        while pq:
            current_dist, current_node = heapq.heappop(pq)

            if current_node in ends:
                found_targets.add(current_node)
                if found_targets == set(ends):
                    break

            for neighbor, data in self.G[current_node].items():
                if (current_node, neighbor) in self.obstacles:
                    continue

                weight = data['weight']
                impact_factor = data.get('impact_factor', 1)

                new_dist = current_dist + weight * impact_factor

                if new_dist < dist[neighbor]:
                    dist[neighbor] = new_dist
                    prev[neighbor] = current_node
                    heapq.heappush(pq, (new_dist, neighbor))

        paths = []
        for end in ends:
            path = []
            current = end
            while current is not None:
                path.append(current)
                current = prev[current]
            if path:
                paths.append((path[::-1], dist[end]))
            else:
                paths.append(([], float('inf')))

        end_time = time.perf_counter()
        print(f"Dijkstra algorithm execution time: {end_time - start_time:.4f} seconds")

        return paths

    def heuristic(self, node, ends):
        if node in self.heuristic_cache:
            return self.heuristic_cache[node]

        pos = self.get_positions()
        if node not in pos or not ends:
            return float('inf')

        end_node = ends[0]
        x1, y1, z1 = pos[node]
        x2, y2, z2 = pos[end_node]

        h = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
        self.heuristic_cache[node] = h
        return h

    def combined_algorithm(self, start, ends):
        start_time = time.perf_counter()
        print(f"执行结合算法，从 {start} 到 {ends}")

        dist = {node: float('inf') for node in self.G}
        dist[start] = 0
        prev = {node: None for node in self.G}
        pq = [(self.heuristic(start, ends), start)]
        found_targets = set()

        while pq:
            current_f, current_node = heapq.heappop(pq)
            current_dist = dist[current_node]

            if current_node in ends:
                found_targets.add(current_node)
                if found_targets == set(ends):
                    break

            for neighbor in self.G[current_node]:
                if (current_node, neighbor) in self.obstacles:
                    continue

                weight = self.G[current_node][neighbor]['weight']
                impact_factor = self.G[current_node].get(neighbor, {}).get('impact_factor', 1)

                new_dist = current_dist + weight * impact_factor

                h = self.heuristic(neighbor, ends)
                f = new_dist + h

                if new_dist < dist[neighbor]:
                    dist[neighbor] = new_dist
                    prev[neighbor] = current_node
                    heapq.heappush(pq, (f, neighbor))

        paths = []
        for end in ends:
            path = []
            current = end
            while current is not None:
                path.append(current)
                current = prev[current]
            if path:
                print(f"目标节点: {end}, 计算路径: {path[::-1]}")
                paths.append((path[::-1], dist[end]))
            else:
                print(f"未找到目标节点 {end} 的路径")

        print(f"计算得到的路径和权重: {paths}")

        end_time = time.perf_counter()
        print(f"结合算法执行时间: {end_time - start_time:.4f} 秒")
        return paths

    def update_graph(self, new_edges, removed_edges):
        for u, v, weight in new_edges:
            self.G[u][v] = {'weight': weight}
            self.G[v][u] = {'weight': weight}
        for u, v in removed_edges:
            if v in self.G[u]:
                del self.G[u][v]
            if u in self.G[v]:
                del self.G[v][u]

    def calculate_weight(self, u, v):
        base_weight = self.G[u][v]['weight']
        impact_factor = self.impact_factors.get((u, v), 1)
        water_flow_factor = self.get_water_flow_factor(u, v)
        return base_weight * impact_factor * water_flow_factor

    def get_water_flow_factor(self, u, v):
        # 根据实际情况计算水流影响因子
        return 1.0  # 示例值