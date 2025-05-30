from controller import Supervisor, Emitter
import random
import numpy as np
from random import uniform
import time

NUM_ROBOTS = 3
robot_names = ['robotA', 'robotB', 'robotC']

supervisor = Supervisor()
emitter = supervisor.getDevice("emitter")
receiver = supervisor.getDevice("receiver")
timestep = int(supervisor.getBasicTimeStep())
receiver.enable(timestep)

zones = [
    {'id': 0, 'position': [-0.35, 0, -0.35], 'dirtiness': uniform(10, 50), 'completed': False},
    {'id': 1, 'position': [ 0.35, 0, -0.35], 'dirtiness': uniform(10, 50), 'completed': False},
    {'id': 2, 'position': [-0.35, 0,  0.35], 'dirtiness': uniform(10, 50), 'completed': False},
    {'id': 3, 'position': [ 0.35, 0,  0.35], 'dirtiness': uniform(10, 50), 'completed': False}
]

robots = []
for name in robot_names:
    node = supervisor.getFromDef(name)
    robots.append({
        'name': name,
        'node': node,
        'translation_field': node.getField('translation'),
        'battery_level': uniform(50, 100),
        'cleaning_rate': uniform(0.5, 1.5),
        'assigned_zone': None
    })

def fitness(assignments, robots, zones):
    score = 0
    for i, zone_id in enumerate(assignments):
        robot = robots[i]
        zone = zones[zone_id]
        pos = robot['translation_field'].getSFVec3f()
        distance = np.linalg.norm(np.array(pos[:3:2]) - np.array(zone['position'][::2]))
        efficiency = robot['cleaning_rate'] * (robot['battery_level'] / 100)
        score += zone['dirtiness'] * efficiency - distance
    #return -score
    return score


def run_pso(active_robots, available_zones):
    zone_ids = [z['id'] for z in available_zones]
    num_robots = len(active_robots)
    
    # Best PSO hyperparameters found from tuning
    swarm_size = 10
    iterations = 20
    inertia = 0.4
    c1 = 1.5
    c2 = 1.5

    particles = [np.random.choice(zone_ids, num_robots) for _ in range(swarm_size)]
    p_best = particles[:]
    p_best_scores = [fitness(p, active_robots, zones) for p in particles]
    g_best = p_best[np.argmax(p_best_scores)]
    g_best_score = max(p_best_scores)

    for iteration in range(iterations):
        for i in range(swarm_size):
            new_p = p_best[i].copy()
            idx = random.randint(0, num_robots - 1)

            if random.random() < inertia:
                new_p[idx] = p_best[i][idx]
            elif random.random() < c1 / (c1 + c2):
                new_p[idx] = random.choice(p_best[i])
            else:
                new_p[idx] = random.choice(g_best)

            new_score = fitness(new_p, active_robots, zones)
            if new_score > p_best_scores[i]:  # MAXIMIZE fitness
                p_best[i] = new_p
                p_best_scores[i] = new_score
                
                # UPDATE global best if this personal best beats it ------------
                if new_score > g_best_score:
                    g_best_score = new_score
                    g_best       = new_p.copy()

        current_best = max(p_best_scores)
        print(f"[PSO Iter {iteration + 1}] Best fitness: {current_best:.2f}")

    final_assignment = dict(zip([r['name'] for r in active_robots], g_best))
    return final_assignment

step_counter = 0



while supervisor.step(timestep) != -1:
    step_counter += 1

    # Handle robot done messages
    if receiver.getQueueLength() > 0:
        msg = receiver.getData().decode('utf-8')
        parts = msg.split(":")
        if len(parts) == 3 and parts[1] == "done":
            robot_name, _, zone_id = parts
            zone_id = int(zone_id)

            zone = next((z for z in zones if z['id'] == zone_id), None)
            if zone and not zone['completed']:
                zone['completed'] = True
                zone['dirtiness'] = 0
                print(f">>> Zone {zone_id} marked completed by {robot_name}")

            for r in robots:
                if r['name'] == robot_name:
                    r['assigned_zone'] = None


    if step_counter % 100 == 0:
        print(f"\n====== STEP {step_counter} | ENVIRONMENT UPDATE ======")

        # Update dirtiness
        for zone in zones:
            if not zone['completed']:
                zone['dirtiness'] += uniform(-2, 5)
                zone['dirtiness'] = max(0, zone['dirtiness'])

        # Update robot statuses
        active_robots = []
        for r in robots:
            if r['battery_level'] > 0:
                r['battery_level'] -= uniform(1, 5)
                if r['battery_level'] <= 0:
                    r['battery_level'] = 0
                    print(f"!!! {r['name']} has run out of battery.")
                    r['assigned_zone'] = None
                else:
                    active_robots.append(r)
                    
        #Appply cleaning effect if robot is in a zone
        for r in active_robots:
            zone_id = r['assigned_zone']
            if zone_id is not None:
                zone = next((z for z in zones if z['id'] == zone_id), None)
                if zone and not zone['completed']:
                    zone['dirtiness'] -= r['cleaning_rate']
                    zone['dirtiness'] = max(0, zone['dirtiness'])
                    print(f"~~~ {r['name']} cleaned zone {zone_id} | New dirtiness: {zone['dirtiness']:.2f}")
                    if zone['dirtiness'] == 0:
                        zone['completed'] = True
                        print(f">>> Zone {zone_id} fully cleaned by {r['name']}")


        if not active_robots:
            print("!!! All robots dead. Simulation ends.")
            break

        #identify available zones
        available_zones = [z for z in zones if not z['completed']]

        if not available_zones:
            print("+++ All zones cleaned. Mission complete.")
            break

        # run PSO
        print(f"[PSO] Running with {len(active_robots)} robots and {len(available_zones)} zones...")
        assignments = run_pso(active_robots, available_zones)

        for name, zone_id in assignments.items():
            for r in robots:
                if r['name'] == name:
                    r['assigned_zone'] = zone_id

            message = f"{name}:{zone_id}"
            emitter.send(message.encode('utf-8'))
            print(f"[PSO] Assigned {name} to zone {zone_id}")

        print("\n--- Current Zone States ---")
        for z in zones:
            status = "✓ Done" if z['completed'] else f"{z['dirtiness']:.2f} dirty"
            print(f" - Zone {z['id']}: {status}")

        print("\n--- Robot Statuses ---")
        for r in robots:
            print(f" - {r['name']} | Battery: {r['battery_level']:.1f} | Rate: {r['cleaning_rate']:.2f} | Assigned: {r['assigned_zone']}")

