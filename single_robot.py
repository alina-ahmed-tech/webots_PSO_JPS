from controller import Robot, Camera, CameraRecognitionObject, InertialUnit, DistanceSensor, PositionSensor
import math
import sys
from collections import deque

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Setup receiver to get zone assignments
receiver = robot.getDevice('receiver')
receiver.enable(timestep)
emitter = robot.getDevice('emitter')


robot_name = robot.getName()
current_zone = None 

#enable distance sensors
frontDistanceSensor = robot.getDevice('front_ds')
leftDistanceSensor = robot.getDevice('left_ds')
rightDistanceSensor = robot.getDevice('right_ds')
frontDistanceSensor.enable(timestep)
leftDistanceSensor.enable(timestep)
rightDistanceSensor.enable(timestep)
#getting the position sensors
leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)
# enable camera and recognition
camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)
#enable imu
imu = robot.getDevice('inertial unit')
imu.enable(timestep)

#gps
gps = robot.getDevice("gps")
gps.enable(timestep)

def gps_to_grid(x, z):
    # Arena's top-left corner in world coordinates
    floor_width = 1.07  # meters (X direction)
    floor_height = 1.02  # meters (Z direction)

    grid_cols = 9
    grid_rows = 9
    cell_size_x = floor_width / grid_cols
    cell_size_z = floor_height / grid_rows

    arena_center_x = -0.258  # from the arena's translation
    arena_center_z = -0.192

    # Compute top-left origin (in webots coordinates)
    origin_x = arena_center_x - (floor_width / 2)
    origin_z = arena_center_z - (floor_height / 2)

    # Convert GPS to grid index
    col = int((x - origin_x) / cell_size_x)
    row = int((z - origin_z) / cell_size_z)

    return (row, col)

# Wait for GPS to return valid data
while robot.step(timestep) != -1:
    gps_pos = gps.getValues()
    if not (math.isnan(gps_pos[0]) or math.isnan(gps_pos[2])):
        break  # Valid GPS reading obtained

start_row, start_col = gps_to_grid(gps_pos[0], gps_pos[2])
start = (start_row, start_col)

# set target position to infinity
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

class Cell:
    def __init__(self, num, n, s, e, w, value, visited, neighbors):
        self.num = num
        self.n = n
        self.s = s
        self.e = e
        self.w = w
        self.value = value
        self.visited = visited
        self.neighbors = neighbors

class Pose:
    def __init__(self, x, y, n):
        self.x = x
        self.y = y 
        self.n = n

def is_walkable(grid, x, y):
    return 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] != 1

def jump(grid, x, y, dx, dy, goal):
    while is_walkable(grid, x, y):
        if (x, y) == goal:
            return x, y

        if dx != 0:
            if (is_walkable(grid, x, y - 1) and not is_walkable(grid, x - dx, y - 1)) or \
               (is_walkable(grid, x, y + 1) and not is_walkable(grid, x - dx, y + 1)):
                return x, y
        elif dy != 0:
            if (is_walkable(grid, x - 1, y) and not is_walkable(grid, x - 1, y - dy)) or \
               (is_walkable(grid, x + 1, y) and not is_walkable(grid, x + 1, y - dy)):
                return x, y

        x += dx
        y += dy

        if not (0 <= x < len(grid) and 0 <= y < len(grid[0])):
            break

    return None

def get_successors(grid, node, parent, goal):
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    successors = []
    for dx, dy in directions:
        jump_point = jump(grid, node[0] + dx, node[1] + dy, dx, dy, goal)
        if jump_point is not None:
            successors.append(jump_point)
    return successors

def reconstruct_path(came_from, node):
    path = [node]
    while node in came_from:
        node = came_from[node]
        path.append(node)
    path.reverse()
    return path

# initialize grid for robot traversal
def Setup():      #0 1 2 3 4 5 6 7 8
    known_grid = [[1,1,1,1,1,1,1,1,1], #0
                  [1,0,0,0,0,0,0,0,1], #1
                  [1,1,1,1,1,0,1,0,1], #2
                  [1,0,0,0,1,0,1,0,1], #3
                  [1,0,1,0,1,0,1,0,1], #4
                  [1,0,1,0,0,0,1,0,1], #5
                  [1,0,1,1,1,1,1,0,1], #6
                  [1,0,0,0,0,0,0,0,1], #7
                  [1,1,1,1,1,1,1,1,1]] #8                                                    

    for row in known_grid:
        print(row)
    print('>> initial grid')
    return known_grid
  
def is_valid_position(grid, pos):
    row, col = pos
    return (
        0 <= row < len(grid) and
        0 <= col < len(grid[0]) and
        grid[row][col] != 1
    )

def find_nearest_valid_goal(grid, goal):
    from queue import Queue
    visited = set()
    q = Queue()
    q.put(goal)
    while not q.empty():
        r, c = q.get()
        if (r, c) in visited:
            continue
        visited.add((r, c))
        if is_walkable(grid, r, c):
            return (r, c)
        for dr, dc in [(-1,0), (1,0), (0,-1), (0,1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < len(grid) and 0 <= nc < len(grid[0]):
                q.put((nr, nc))
    return None

def JPS(grid, start, goal):
    if grid[goal[0]][goal[1]] == 2:
        grid[goal[0]][goal[1]] = 0

    open_list = deque([start])
    came_from = {}
    visited = set()

    while open_list:
        current = open_list.popleft()
        visited.add(current)

        if current == goal:
            return reconstruct_path(came_from, current)

        successors = get_successors(grid, current, came_from.get(current), goal)
        for successor in successors:
            if successor not in visited and successor not in open_list:
                came_from[successor] = current
                open_list.append(successor)

    return []

def path_to_directions(path):
    directions = []
    for i in range(1, len(path)):
        x1, y1 = path[i-1]
        x2, y2 = path[i]

        dx = x2 - x1
        dy = y2 - y1

        step_x = 0 if dx == 0 else int(dx / abs(dx))
        step_y = 0 if dy == 0 else int(dy / abs(dy))

        x, y = x1, y1
        while (x, y) != (x2, y2):
            if step_x == -1:
                directions.append("UP")
            elif step_x == 1:
                directions.append("DOWN")
            elif step_y == -1:
                directions.append("LEFT")
            elif step_y == 1:
                directions.append("RIGHT")
            x += step_x
            y += step_y

    return directions


planner = []
cells = []
path = []
visited_cells = []

robot_pose = Pose(-1, -1, -1)
for i in range(16):
    visited_cells.append('.')


#[Visited, North, East, South, West]
grid = [[0, 1, 0, 0, 1], [0, 1, 0, 0, 0], [0, 1, 0, 0, 0],[0, 1, 1, 0, 0], #starting grid
            [0, 0, 0, 0, 1], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0],[0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0],[0, 0, 1, 0, 0],
            [0, 0, 0, 1, 1], [0, 0, 0, 1, 0], [0, 0, 0, 1, 0],[0, 0, 1, 1, 0]]

def printMaze(maze):
    print("__________________________________")
    for i in range(4):
        x = i*4
        if (maze[x][0] == 0):
            v1 = "?"
        else:
            v1 = "V"
        if (maze[x+1][0] == 0):
            v2 = "?"
        else:
            v2 = "V"
        if (maze[x+2][0] == 0):
            v3 = "?"
        else:
            v3 = "V"
        if (maze[x+3][0] == 0):
            v4 = "?"
        else:
            v4 = "V"
        print("|  "+ str(maze[x][1]) +"\t  " +str(maze[x+1][1])+"\t  " +str(maze[x+2][1])
              +"\t  " +str(maze[x+3][1])+ "    |")
        print("|" +str(maze[x][4]) + " " +v1+" " + str(maze[x][2])+"\t" +str(maze[x+1][4])+ " " +v2+" " + str(maze[x+1][2])
              +"\t" +str(maze[x+2][4])+ " " +v3+" " + str(maze[x+2][2])
              +"\t" +str(maze[x+3][4]) + " " +v4+" " + str(maze[x+3][2]) +"  |")
        print("|  "+str(maze[x][3]) +"\t  " +str(maze[x+1][3])+"\t  " +str(maze[x+2][3])
              +"\t  " +str(maze[x+3][3])+"    |")
        if(i==3):
            print("|_________________________________|\n")
        else:
            print("|                                 |")

def setPose(n):
    visited_cells[n-1] = 'X'
    robot_pose.n = n
     


def Initialize():
    global path
    known_grid = Setup()

    # Wait for GPS to be ready
    while robot.step(timestep) != -1:
        gps_pos = gps.getValues()
        if not (math.isnan(gps_pos[0]) or math.isnan(gps_pos[2])):
            break

    start = gps_to_grid(gps_pos[0], gps_pos[2])

    goal = (5, 3)

    jps_path = JPS(known_grid, start, goal)
    path = path_to_directions(jps_path)

# controls robot turning based on generated plan and cardinal direction
def RobotMotion(d, m):
    if (d == 'W' and m == 'UP' or
        d == 'N' and m == 'RIGHT' or
        d == 'E' and m == 'DOWN' or
        d == 'S' and m == 'LEFT'):
        turn(-90)
    elif (d == 'W' and m == 'DOWN' or 
        d == 'N' and m == 'LEFT' or
        d == 'E' and m == 'UP' or
        d == 'S' and m == 'RIGHT'):
        turn(90)
    elif (d == 'W' and m == 'RIGHT' or 
        d == 'N' and m == 'DOWN' or
        d == 'E' and m == 'LEFT' or
        d == 'S' and m == 'UP'):
        turn(180)  
 
def ObstacleAvoidance():
    left = getDistanceSensors()[0]
    front = getDistanceSensors()[1]
    right = getDistanceSensors()[2]
    L = 1 if left < 7 else 0
    F = 1 if front < 7 else 0
    R = 1 if right < 7 else 0
    #print('L(', L,') F(', F,') R(', R,')')
    return (L,F,R)
            
def CheckWalls(L, F, R):
    pos = getPositionSensors()[0]
    #facing west
    if getDirectionFacing() == 'W': 
        if not L and not F and R: #OOW
            if visited_cells[2] == '.':
                setPose(3)            
        if L and not F and R: #WOW
            if visited_cells[1] == '.':
                setPose(2)  
            elif visited_cells[14] == '.': 
                setPose(15)  
            elif visited_cells[13] == '.': 
                setPose(14)                       
        if L and F and R: #WWW
            if visited_cells[0] == '.': 
                setPose(1)                 
        if L and F and not R: #WWO
            if visited_cells[12] == '.':   
                setPose(13)   
    #facing north                   
    elif getDirectionFacing() == 'N': 
        if not L and not F and R: #OOW
            if visited_cells[15] == '.':  
                setPose(16)
            elif visited_cells[10] == '.':
                setPose(11)        
        if L and not F and R: #WOW
            if visited_cells[11] == '.':    
                setPose(12)
            elif visited_cells[7] == '.':  
                setPose(8)
            elif visited_cells[8] == '.':   
                setPose(9)
            elif visited_cells[6] == '.': 
                setPose(7)            
        if not L and F and R: #OWW
            if visited_cells[3] == '.':   
                setPose(4)
        if L and F and not R: #WWO
            if visited_cells[4] == '.':    
                setPose(5)
    #facing east            
    elif getDirectionFacing() == 'E': 
        if L and F and not R: #WWO
            if visited_cells[5] == '.':    
                setPose(6)
        if not L and F and R: #OOW
            if visited_cells[10] == '.': 
                setPose(11)
    #facing south           
    elif getDirectionFacing() == 'S':
        if not L and F and R: #OWW
            if visited_cells[9] == '.': 
                setPose(10)
                
    FindCoordinates(robot_pose.n)
    UpdateMaze(robot_pose.n, L, F, R, getDirectionFacing())
    
# store every other grid cell movement in plan for 4x4 robot map
def RobotController():
    every_other_step = []
    for i in range(len(path)):
        if i % 2 == 0:
            every_other_step.append(path[i])
    return every_other_step
 
def getDirectionFacing():
    degrees = getIMUDegrees()
    if(degrees<45 or degrees>315):
        return 'S'
    if(degrees >45 and degrees < 135):
        return 'E'
    if(degrees >135 and degrees < 225):
        return 'N'
    if(degrees >225 and degrees <315):
        return 'W'
    return '?'
 
def Forward(distance): #distance in inches, no time requirement, run max speed ~ 5 inches
    setSpeedsIPS(5,5)
    startDistance = getPositionSensors()
    curDistance = getPositionSensors()
    while(robot.step(timestep) != -1 and abs(curDistance[0] - startDistance[0]) < distance and abs(curDistance[1] - startDistance[1]) < distance):
        curDistance = getPositionSensors()
    setSpeedsIPS(0,0)
        
def turn(degree): #turns to face another cardinal direction.
    dir = getDirectionFacing()
    if(dir == 'S'):
        target= 0+degree
    elif(dir == 'E'):
        target = 90+degree
    elif(dir=='N'):
        target = 180+degree
    elif(dir=='W'):
        target =270 + degree  
    #ensure are targetdirection is (0-359)
    if(target>=360):
        target=target%360
    elif(target <0):
        target = target+360   
    upperBound = target+1
    lowerBound = target-1
    setSpeedsIPS(-2,2)
    if(degree<0): #if we are turning to the left
        setSpeedsIPS(2,-2)      
    if(lowerBound < 0):
        #if target is 0, we want our imu to give reading of <1 or >359 to stop as IMU rolls over
        lowerBound+=360
        while(robot.step(timestep) != -1 and not (lowerBound < getIMUDegrees() or  getIMUDegrees() < upperBound )):        
            if(abs(lowerBound - getIMUDegrees())< 30 or abs(upperBound - getIMUDegrees())< 30):
                setSpeedsIPS(-1,1)
                if(degree<0): #if we are turning to the left
                    setSpeedsIPS(1,-1)
    else:# we want to stop when our IMU reads lowerBound<imu<upperBound
        while(robot.step(timestep) != -1 and not (lowerBound< getIMUDegrees()  and getIMUDegrees() <upperBound)): 
            if(abs(lowerBound - getIMUDegrees())< 30 or abs(upperBound - getIMUDegrees())< 30):
                setSpeedsIPS(-1,1)
                if(degree<0): #if we are turning to the left
                    setSpeedsIPS(1,-1)          
    setSpeedsIPS(0,0)

def UpdateMaze(n, L, F, R, direction):
    global grid
    grid[n - 1][0] = 1 #mark cell visited
    # mark internal walls based on sensor readings
    if direction == 'N':
        grid[n - 1][4] = L       # LEFT sensor = WEST wall
        grid[n - 1][1] = F       # FRONT sensor = NORTH wall
        grid[n - 1][2] = R       # RIGHT sensor = WEST wall
    elif direction == 'E':
        grid[n - 1][1] = L       # LEFT sensor = NORTH wall
        grid[n - 1][2] = F       # FRONT sensor = EAST wall
        grid[n - 1][3] = R       # RIGHT sensor = SOUTH wall
    elif direction == 'S':
        grid[n - 1][2] = L       # LEFT sensor = EAST wall
        grid[n - 1][3] = F       # FRONT sensor = SOUTH wall
        grid[n - 1][4] = R       # RIGHT sensor = WEST wall
    elif direction == 'W':
        grid[n - 1][3] = L       # LEFT sensor = SOUTH wall
        grid[n - 1][4] = F       # FRONT sensor = WEST wall
        grid[n - 1][1] = R       # RIGHT sensor = NORTH wall
        
def FindCoordinates(n):
    # general X coordinate
    if n == 1 or n == 5 or n == 9 or n == 13:
        robot_pose.x = -15
    elif n == 2 or n == 6 or n == 10 or n == 14:
        robot_pose.x = -5
    elif n == 3 or n == 7 or n == 11 or n == 15:
        robot_pose.x = 5
    elif n == 4 or n == 8 or n == 12 or n == 16:
        robot_pose.x = 15
    # general Y coordinate
    if n == 1 or n == 2 or n == 3 or n == 4:
        robot_pose.y = 15
    elif n == 5 or n == 6 or n == 7 or n == 8:
        robot_pose.y = 5
    elif n == 9 or n == 10 or n == 11 or n == 12:
        robot_pose.y = -5
    elif n == 13 or n == 14 or n == 15 or n == 16:
        robot_pose.y = -15

def setSpeedsIPS(Vl,Vr):#function for getting setting motors in inches
    left_speed = (Vl/.8) #convert to radians/sec
    right_speed = (Vr/.8)
    if left_speed > 6.28:
        left_speed = 6.28
    if left_speed < -6.28:
        left_speed = -6.28
    if right_speed > 6.28:
        right_speed = 6.28
    if right_speed < -6.28:
        right_speed = -6.28
    leftMotor.setVelocity(left_speed) 
    rightMotor.setVelocity(right_speed)

def getIMUDegrees(): #convert radians to degrees and make range 180
    return imu.getRollPitchYaw()[2]*180/math.pi + 180

def getPositionSensors(): #function for getting getting position sensors in inches. convert from radians
    return [leftposition_sensor.getValue()*0.8, rightposition_sensor.getValue()*0.8] 

def getDistanceSensors(): #function for getting distance sensors in inches. convert from meters
    return [leftDistanceSensor.getValue()*39.3701, frontDistanceSensor.getValue()*39.3701, rightDistanceSensor.getValue()*39.3701]

# implement and print robot motions generated by planning           
def FollowPath(path):
    print(' ')
    print('Shortest Path =>',path)
    for motion in path: # ADD REAL ROBOT POSE
        readings = ObstacleAvoidance()
        CheckWalls(readings[0],readings[1],readings[2])
        direction = getDirectionFacing()
        print(' ')
        print(round(getIMUDegrees()),'°',direction,'| driving',motion,'...')
        RobotMotion(direction, motion)  
        Forward(10)
    print(' ')
    print('                                               ****REACHED TARGET****')   
    print('---------------------------------------------------------------------')
    print(' ')
    # Notify that the robot is done
    done_message = f"{robot_name}:done"
    emitter.send(done_message.encode('utf-8'))

    
    
def RobotController():
    every_other_step = []
    for i in range(len(path)):
        if i % 2 == 0:
            every_other_step.append(path[i])
    return every_other_step

# Zone GPS coordinates (center of each quadrant in arena)
ZONE_COORDS = {
    0: (-0.35, -0.35),
    1: ( 0.35, -0.35),
    2: (-0.35,  0.35),
    3: ( 0.35,  0.35)
}

def zone_to_grid(zone_id):
    x, z = ZONE_COORDS[zone_id]
    return gps_to_grid(x, z)


while robot.step(timestep) != -1:
    if receiver.getQueueLength() > 0:
        msg = receiver.getData().decode('utf-8')
        receiver.nextPacket()
        try:
            name, zone = msg.split(":")
            if name == robot.getName():
                zone = int(zone)
                if zone != current_zone:
                    current_zone = zone
                    print(f"[{robot_name}] Moving to zone {zone}")
                    
                    gps_pos = gps.getValues()
                    start = gps_to_grid(gps_pos[0], gps_pos[2])
                    goal = zone_to_grid(zone)
                    
                    known_grid = Setup()
                    goal = find_nearest_valid_goal(known_grid, goal)
                
                    if goal is None:
                        print(f"[{robot_name}] ERROR: No valid goal near requested zone")
                        continue

                    if 0 <= goal[0] < len(known_grid) and 0 <= goal[1] < len(known_grid[0]):
                        jps_path = JPS(known_grid, start, goal)
                        if not jps_path:
                            print(f"[{robot_name}] No valid path found from {start} to {goal}")
                            continue

                        directions = path_to_directions(jps_path)
                        path = directions  # update global path
                        shortened_path = RobotController()  # every other step
                        print("Shortened path =>", shortened_path)
                        FollowPath(shortened_path)
                        
                        done_message = f"{robot_name}:done"
                        emitter.send(done_message.encode('utf-8'))
                        print(f"[{robot_name}] ******** Done cleaning zone {current_zone}")



                    else:
                        print(f"[{robot_name}] ERROR: Goal index {goal} out of bounds!")
        except Exception as e:
            print(f"[{robot_name}] ERROR: {e}")
