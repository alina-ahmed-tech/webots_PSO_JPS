
import itertools
import numpy as np
import pandas as pd
import random
import time

# Dummy class to mimic robot position
class DummyPos:
    def __init__(self, x, z):
        self.pos = [x, 0, z]
    def getSFVec3f(self):
        return self.pos

# Fitness function
def fitness(assignments, robots, zones):
    score = 0
    workloads = []
    for i, zone_id in enumerate(assignments):
        robot = robots[i]
        zone = zones[zone_id]
        pos = robot['translation_field'].getSFVec3f()
        distance = np.linalg.norm(np.array(pos[:3:2]) - np.array(zone['position'][::2]))
        efficiency = robot['cleaning_rate'] * (robot['battery_level'] / 100)
        workload = zone['dirtiness'] / efficiency
        workloads.append(workload)
        score += zone['dirtiness'] * efficiency - distance
    std_dev_workload = np.std(workloads)
    return -score, std_dev_workload

# PSO simulation function w convergence tracking
def run_pso_test(robots, zones, swarm_size, iterations, w, c1, c2):
    zone_ids = [z['id'] for z in zones]
    num_robots = len(robots)

    particles = [np.random.choice(zone_ids, num_robots) for _ in range(swarm_size)]
    velocities = [np.zeros(num_robots, dtype=int) for _ in range(swarm_size)]
    p_best = particles[:]
    p_best_scores = [fitness(p, robots, zones)[0] for p in particles]
    g_best = p_best[np.argmin(p_best_scores)]
    g_best_score = min(p_best_scores)

    convergence_iter = iterations

    for iteration in range(iterations):
        for i in range(swarm_size):
            new_p = p_best[i].copy()
            idx = random.randint(0, num_robots - 1)

            if random.random() < w:
                new_p[idx] = p_best[i][idx]
            elif random.random() < c1:
                new_p[idx] = random.choice(p_best[i])
            elif random.random() < c2:
                new_p[idx] = random.choice(g_best)
            else:
                new_p[idx] = random.choice(zone_ids)

            new_score, _ = fitness(new_p, robots, zones)
            if new_score < p_best_scores[i]:
                p_best[i] = new_p
                p_best_scores[i] = new_score

        current_best_score = min(p_best_scores)
        if current_best_score < g_best_score:
            g_best_score = current_best_score
            g_best = p_best[np.argmin(p_best_scores)]
            convergence_iter = iteration + 1

    _, std_dev = fitness(g_best, robots, zones)
    return -g_best_score, std_dev, convergence_iter, (swarm_size, w, c1, c2, iterations)

# Create test scenarios
robots = [
    {'battery_level': 80, 'cleaning_rate': 1.0, 'translation_field': DummyPos(-0.5, -0.5)},
    {'battery_level': 60, 'cleaning_rate': 1.2, 'translation_field': DummyPos(0.0, 0.0)},
    {'battery_level': 40, 'cleaning_rate': 1.5, 'translation_field': DummyPos(0.4, 0.4)}
]

zones = [
    {'id': 0, 'position': [-0.35, 0, -0.35], 'dirtiness': 30},
    {'id': 1, 'position': [0.35, 0, -0.35], 'dirtiness': 20},
    {'id': 2, 'position': [-0.35, 0, 0.35], 'dirtiness': 40},
    {'id': 3, 'position': [0.35, 0, 0.35], 'dirtiness': 50}
]

# Hyperparameter ranges
swarm_sizes = [10, 20, 30]
inertias = [0.4, 0.6]
c1s = [1.5, 2.0]
c2s = [1.5, 2.0]
iterations_list = [20, 30, 50]

results = []

for swarm_size, w, c1, c2, iterations in itertools.product(swarm_sizes, inertias, c1s, c2s, iterations_list):
    scores = []
    std_devs = []
    convergences = []
    for _ in range(3):  # Repeat runs
        score, std_dev, conv_iter, _ = run_pso_test(robots, zones, swarm_size, iterations, w, c1, c2)
        scores.append(score)
        std_devs.append(std_dev)
        convergences.append(conv_iter)
    avg_score = np.mean(scores)
    avg_std = np.mean(std_devs)
    avg_conv = np.mean(convergences)
    results.append({
        "swarm_size": swarm_size,
        "inertia": w,
        "c1": c1,
        "c2": c2,
        "iterations": iterations,
        "avg_fitness": round(avg_score, 2),
        "std_dev_workload": round(avg_std, 2),
        "avg_convergence_iter": int(avg_conv)
    })

df = pd.DataFrame(results)
best = df.sort_values("avg_fitness", ascending=False).iloc[0]

print("\nBest Parameters Found:")
print(f"- Swarm Size: {best['swarm_size']}")
print(f"- Inertia: {best['inertia']}")
print(f"- c1: {best['c1']}")
print(f"- c2: {best['c2']}")
print(f"- Iterations: {best['iterations']}\n")

print("Achieved:")
print(f"- Avg. fitness: {best['avg_fitness']}")
print(f"- Std deviation in workload: {best['std_dev_workload']}")
print(f"- Convergence in ~{best['avg_convergence_iter']} iterations\n")

print("Full Results Table:")
print(df.to_string(index=False))
