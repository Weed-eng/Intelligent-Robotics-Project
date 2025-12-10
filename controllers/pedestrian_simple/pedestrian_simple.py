# simple pedestrian controller - walks back and forth OR in rectangular loop
# usage:
#   line mode: --start=x,y --end=x,y --speed=v
#   loop mode: --loop=x1,y1:x2,y2:x3,y3:x4,y4 --speed=v
# pedestrian starts from their initial translation in world file

from controller import Supervisor
import sys
import math

def parse_args():
    start = None
    end = None
    loop = None
    speed = 0.5

    for arg in sys.argv[1:]:
        if arg.startswith('--start='):
            coords = arg.split('=')[1].split(',')
            start = (float(coords[0]), float(coords[1]))
        elif arg.startswith('--end='):
            coords = arg.split('=')[1].split(',')
            end = (float(coords[0]), float(coords[1]))
        elif arg.startswith('--loop='):
            # parse 4 waypoints: x1,y1:x2,y2:x3,y3:x4,y4
            points = arg.split('=')[1].split(':')
            loop = []
            for p in points:
                coords = p.split(',')
                loop.append((float(coords[0]), float(coords[1])))
        elif arg.startswith('--speed='):
            speed = float(arg.split('=')[1])

    return start, end, loop, speed

def main():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())

    start, end, loop, speed = parse_args()

    # get the translation field of this robot
    self_node = robot.getSelf()
    trans_field = self_node.getField('translation')

    # get initial position from world file (don't override it)
    initial_pos = trans_field.getSFVec3f()
    pos = [initial_pos[0], initial_pos[1]]

    if loop:
        # rectangular loop mode - cycle through waypoints
        run_loop_mode(robot, timestep, trans_field, loop, speed, pos)
    else:
        # line mode - back and forth between start and end
        run_line_mode(robot, timestep, trans_field, start, end, speed, pos)

def run_line_mode(robot, timestep, trans_field, start, end, speed, initial_pos):
    """Walk back and forth between two points."""
    # use initial position from world file
    pos = list(initial_pos)

    # calculate direction vector
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dist = math.sqrt(dx*dx + dy*dy)
    if dist > 0:
        dx /= dist
        dy /= dist

    # determine initial direction based on position
    # if closer to start, go toward end; if closer to end, go toward start
    dist_to_start = math.hypot(pos[0] - start[0], pos[1] - start[1])
    dist_to_end = math.hypot(pos[0] - end[0], pos[1] - end[1])
    direction = 1 if dist_to_start < dist_to_end else -1

    while robot.step(timestep) != -1:
        step_dist = speed * timestep / 1000.0
        pos[0] += direction * dx * step_dist
        pos[1] += direction * dy * step_dist

        # check if reached target
        if direction == 1:
            to_end_x = end[0] - pos[0]
            to_end_y = end[1] - pos[1]
            dot = to_end_x * dx + to_end_y * dy
            if dot <= 0:
                pos[0] = end[0]
                pos[1] = end[1]
                direction = -1
        else:
            to_start_x = start[0] - pos[0]
            to_start_y = start[1] - pos[1]
            dot = to_start_x * (-dx) + to_start_y * (-dy)
            if dot <= 0:
                pos[0] = start[0]
                pos[1] = start[1]
                direction = 1

        current = trans_field.getSFVec3f()
        trans_field.setSFVec3f([pos[0], pos[1], current[2]])

def find_nearest_waypoint(pos, waypoints):
    """Find the waypoint index closest to current position."""
    min_dist = float('inf')
    nearest = 0
    for i, wp in enumerate(waypoints):
        d = math.hypot(pos[0] - wp[0], pos[1] - wp[1])
        if d < min_dist:
            min_dist = d
            nearest = i
    return nearest

def run_loop_mode(robot, timestep, trans_field, waypoints, speed, initial_pos):
    """Walk in a loop through waypoints (rectangular path)."""
    num_wp = len(waypoints)
    pos = list(initial_pos)

    # find nearest waypoint to determine where we are in the loop
    nearest = find_nearest_waypoint(pos, waypoints)

    # we'll head toward the next waypoint after the nearest one
    # this makes the pedestrian continue in the loop direction
    current_wp = nearest

    while robot.step(timestep) != -1:
        # get next waypoint
        next_wp = (current_wp + 1) % num_wp
        target = waypoints[next_wp]

        # calculate direction to target
        dx = target[0] - pos[0]
        dy = target[1] - pos[1]
        dist = math.sqrt(dx*dx + dy*dy)

        if dist < 0.05:
            # reached waypoint, move to next
            current_wp = next_wp
            pos[0] = target[0]
            pos[1] = target[1]
        else:
            # move toward target
            dx /= dist
            dy /= dist
            step_dist = speed * timestep / 1000.0
            pos[0] += dx * step_dist
            pos[1] += dy * step_dist

        current = trans_field.getSFVec3f()
        trans_field.setSFVec3f([pos[0], pos[1], current[2]])

if __name__ == '__main__':
    main()
