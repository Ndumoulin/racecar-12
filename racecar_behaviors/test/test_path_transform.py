import numpy as np
import matplotlib.pyplot as plt
from heapq import heappush, heappop

# -------------------------
# Exemple de carte
# -------------------------
grid = np.array([
    [0, 0, 1, 0, 0, 0, 0, 0, 0],
    [0, 0, 1, 1, 0, 0, 0, 0, 0],
    [0, 0, 1, 1, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 1, 0],
])

start = (0, 0)
goal = (5, 8)

# -------------------------
# Brushfire: distance to nearest obstacle 
# -------------------------
def brushfire(grid):
    H, W = grid.shape
    dist = np.full((H, W), np.inf)
    pq = []
    for x in range(H):
        for y in range(W):
            if grid[x, y] == 1:
                dist[x, y] = 0
                heappush(pq, (0, x, y))
    while pq:
        d, x, y = heappop(pq)
        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            nx, ny = x+dx, y+dy
            if 0 <= nx < H and 0 <= ny < W:
                nd = d + 1
                if nd < dist[nx, ny]:
                    dist[nx, ny] = nd
                    heappush(pq, (nd, nx, ny))
    return dist

# -------------------------
# BFS distance to goal (Wavefront-like)(base 4)
# -------------------------
def bfs_distance(grid, goal):
    H, W = grid.shape
    dist = np.full((H, W), np.inf)
    q = [goal]
    if grid[goal] != 0:
        # goal is not free => no path
        return dist
    dist[goal] = 0
    while q:
        x, y = q.pop(0)
        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            nx, ny = x+dx, y+dy
            if 0 <= nx < H and 0 <= ny < W:
                if grid[nx, ny] == 0 and dist[nx, ny] == np.inf:
                    dist[nx, ny] = dist[x, y] + 1
                    q.append((nx, ny))
    return dist

# -------------------------
# A* search that uses cost = 1 + alpha * obstacle_cost
# -------------------------
def a_star(grid, start, goal, obstacle_cost, alpha):
    H, W = grid.shape
    def heuristic(a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])  # Manhattan

    open_pq = []
    heappush(open_pq, (heuristic(start, goal), 0.0, start))  # (f, g, node)
    came_from = {start: None}
    g_score = {start: 0.0}
    visited = set()

    while open_pq:
        f, g, current = heappop(open_pq)
        if current == goal:
            # reconstruct path
            path = []
            node = current
            while node is not None:
                path.append(node)
                node = came_from[node]
            return list(reversed(path))

        if current in visited:
            continue
        visited.add(current)

        x, y = current
        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            nx, ny = x+dx, y+dy
            if not (0 <= nx < H and 0 <= ny < W):
                continue
            if grid[nx, ny] == 1:
                continue  # obstacle
            tentative_g = g + (1.0 + alpha * obstacle_cost[nx, ny])
            neighbor = (nx, ny)
            if neighbor in g_score and tentative_g >= g_score[neighbor]:
                continue
            came_from[neighbor] = current
            g_score[neighbor] = tentative_g
            f_score = tentative_g + heuristic(neighbor, goal)
            heappush(open_pq, (f_score, tentative_g, neighbor))

    return None  # no path found

# -------------------------
# compute maps
# -------------------------
obstacle_dist = brushfire(grid)
# protect goal: make it maximally safe so PT doesn't avoid it artificially
if np.isfinite(obstacle_dist).any():
    obstacle_dist[goal] = np.max(obstacle_dist[np.isfinite(obstacle_dist)])

# obstacle cost (inverse distance with epsilon)
eps = 1e-3
obstacle_cost = 1.0 / (obstacle_dist + eps)
# set obstacle cost of actual obstacle cells to a very large value (can't traverse)
H, W = grid.shape
for i in range(H):
    for j in range(W):
        if grid[i, j] == 1:
            obstacle_cost[i, j] = 1e6

length_map = bfs_distance(grid, goal)

# -------------------------
# compute normalized alpha automatically
# -------------------------
def compute_alpha_auto(length_map, obstacle_cost, desired_ratio=1.0, alpha_clip=(1e-3, 100.0)):
    # consider only finite/free cells
    mask = np.isfinite(length_map) & np.isfinite(obstacle_cost) & (obstacle_cost < 1e5)
    if not np.any(mask):
        return 1.0
    lm = length_map[mask]
    oc = obstacle_cost[mask]
    # ranges
    lm_range = np.nanmax(lm) - np.nanmin(lm)
    oc_range = np.nanmax(oc) - np.nanmin(oc)
    # avoid zero division
    if oc_range <= 0:
        base = 1.0
    else:
        base = (lm_range / (oc_range + 1e-9)) * desired_ratio
    # clip to avoid extremes
    base = float(np.clip(base, alpha_clip[0], alpha_clip[1]))
    return base

alpha = compute_alpha_auto(length_map, obstacle_cost, desired_ratio=1.0)
print(f"Auto alpha = {alpha:.3f}")

# -------------------------
# run A* with auto alpha
# -------------------------
path = a_star(grid, start, goal, obstacle_cost, alpha)

# fallback: if no path found, try smaller alpha
if path is None:
    print("No path found with auto alpha; trying smaller alpha values...")
    for a_try in [alpha*0.5, alpha*0.2, 0.1, 0.01]:
        path = a_star(grid, start, goal, obstacle_cost, a_try)
        print(f" try alpha={a_try:.4f} -> path {'found' if path else 'not found'}")
        if path:
            alpha = a_try
            break

if path is None:
    print("Aucune trajectoire trouvée.")
else:
    print(f"Path length: {len(path)}, alpha used: {alpha:.4f}")

    # -------------------------
    # plot results
    # -------------------------
    plt.figure(figsize=(6,6))
    plt.imshow(grid, cmap="gray_r")
    px = [p[1] for p in path]
    py = [p[0] for p in path]
    plt.plot(px, py, '-o', linewidth=2, markersize=6, label="A* path")
    plt.scatter([start[1]], [start[0]], c="green", s=120, label="start")
    plt.scatter([goal[1]], [goal[0]], c="blue", s=120, label="goal")
    plt.title(f"A* path with auto alpha = {alpha:.3f}")
    plt.legend()
    plt.show()
