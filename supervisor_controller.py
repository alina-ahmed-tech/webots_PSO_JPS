from controller import Supervisor, Emitter
import random
import numpy as np
from random import uniform, choice


NUM_ROBOTS = 3
robot_names = ['robotA', 'robotB', 'robotC']

supervisor = Supervisor()
emitter = supervisor.getDevice("emitter")
receiver = supervisor.getDevice("receiver")
timestep = int(supervisor.getBasicTimeStep())
receiver.enable(timestep)


zones = [
    {'id': 0, 'position': [-0.35, 0, -0.35], 'dirtiness': random.uniform(10, 50)},
    {'id': 1, 'position': [0.35, 0, -0.35], 'dirtiness': random.uniform(10, 50)},
    {'id': 2, 'position': [-0.35, 0, 0.35], 'dirtiness': random.uniform(10, 50)},
    {'id': 3, 'position': [0.35, 0, 0.35], 'dirtiness': random.uniform(10, 50)}
]

robots = []
for name in robot_names:
    node = supervisor.getFromDef(name)
    robots.append({
        'name': name,
        'node': node,
        'translation_field': node.getField('translation'),
        'battery_level': random.uniform(50, 100),
        'cleaning_rate': random.uniform(0.5, 1.5)
    })

def get_zone_id(pos):
    x, _, z = pos
    if x < 0 and z < 0:
        return 0
    elif x >= 0 and z < 0:
        return 1
    elif x < 0 and z >= 0:
        return 2
    else:
        return 3

def fitness(assignments, robots, zones):
    score = 0
    for i, zone_id in enumerate(assignments):
        robot = robots[i]
        zone = zones[zone_id]
        pos = robot['translation_field'].getSFVec3f()
        distance = np.linalg.norm(np.array(pos[:3:2]) - np.array(zone['position'][::2]))
        efficiency = robot['cleaning_rate'] * (robot['battery_level'] / 100)
        score += zone['dirtiness'] * efficiency - distance
    return -score

def run_pso(active_robots):
    particles = [np.random.randint(0, 4, len(active_robots)) for _ in range(10)]
    p_best = particles[:]
    p_best_scores = [fitness(p, active_robots, zones) for p in particles]
    g_best = p_best[np.argmin(p_best_scores)]

    for iteration in range(30):
        for i in range(len(particles)):
            new_p = particles[i][:]
            new_p[random.randint(0, len(active_robots) - 1)] = random.randint(0, 3)
            new_score = fitness(new_p, active_robots, zones)
            if new_score < p_best_scores[i]:
                p_best[i] = new_p
                p_best_scores[i] = new_score
        g_best = p_best[np.argmin(p_best_scores)]

    print(f"************ Final PSO Assignment: {g_best} with score {-min(p_best_scores):.2f}")
    return dict(zip([r['name'] for r in active_robots], g_best))



step_counter = 0

done_robots = set()

while supervisor.step(timestep) != -1:
    step_counter += 1
    if step_counter % 100 == 0:
        print(f"\n********** Environment update at step {step_counter}")
        
        for zone in zones:
            # Simulate dirtiness increasing
            zone['dirtiness'] += uniform(-2, 5)
            zone['dirtiness'] = max(zone['dirtiness'], 0)

        for robot in robots:
            # Simulate battery drain and failures
            robot['battery_level'] -= uniform(1, 5)
            if robot['battery_level'] <= 0:
                robot['battery_level'] = 0
                print(f"********** {robot['name']} battery depleted. Excluding from PSO.")
        
        # Only include functional robots
        active_robots = [r for r in robots if r['battery_level'] > 0]

        if not active_robots:
            print("************* No active robots remaining. Stopping simulation.")
            break

        assignments = run_pso(active_robots)

        for name, zone_id in assignments.items():
            message = f"{name}:{zone_id}"
            emitter.send(message.encode('utf-8'))
            print(f"************ Sent to {name}: Zone {zone_id}")
            
            print("*************** Current zone dirtiness:")
            for z in zones:
                print(f" - Zone {z['id']}: {z['dirtiness']:.2f}")

            print("************* Robot statuses:")
            for r in robots:
                print(f" - {r['name']} | Battery: {r['battery_level']:.1f} | Rate: {r['cleaning_rate']:.2f}")

