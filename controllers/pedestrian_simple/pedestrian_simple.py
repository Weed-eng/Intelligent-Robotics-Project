# simple pedestrian controller - walks back and forth between two points
# usage: controller "pedestrian_simple" with args --start=x,y --end=x,y --speed=v

from controller import Supervisor
import sys
import math

def parse_args():
    start = (0, 0)
    end = (1, 0)
    speed = 0.5

    for arg in sys.argv[1:]:
        if arg.startswith('--start='):
            coords = arg.split('=')[1].split(',')
            start = (float(coords[0]), float(coords[1]))
        elif arg.startswith('--end='):
            coords = arg.split('=')[1].split(',')
            end = (float(coords[0]), float(coords[1]))
        elif arg.startswith('--speed='):
            speed = float(arg.split('=')[1])

    return start, end, speed

def main():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())

    start, end, speed = parse_args()

    # get the translation field of this robot
    self_node = robot.getSelf()
    trans_field = self_node.getField('translation')

    # current position and direction
    pos = list(start)
    target = list(end)

    # calculate direction vector
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dist = math.sqrt(dx*dx + dy*dy)
    if dist > 0:
        dx /= dist
        dy /= dist

    direction = 1  # 1 = toward end, -1 = toward start

    while robot.step(timestep) != -1:
        # move toward target
        step_dist = speed * timestep / 1000.0
        pos[0] += direction * dx * step_dist
        pos[1] += direction * dy * step_dist

        # check if reached target
        if direction == 1:
            # moving toward end
            to_end_x = end[0] - pos[0]
            to_end_y = end[1] - pos[1]
            dot = to_end_x * dx + to_end_y * dy
            if dot <= 0:
                # passed the end point, reverse
                pos[0] = end[0]
                pos[1] = end[1]
                direction = -1
        else:
            # moving toward start
            to_start_x = start[0] - pos[0]
            to_start_y = start[1] - pos[1]
            dot = to_start_x * (-dx) + to_start_y * (-dy)
            if dot <= 0:
                # passed the start point, reverse
                pos[0] = start[0]
                pos[1] = start[1]
                direction = 1

        # get current z position (height)
        current = trans_field.getSFVec3f()
        z = current[2]

        # update position
        trans_field.setSFVec3f([pos[0], pos[1], z])

if __name__ == '__main__':
    main()
