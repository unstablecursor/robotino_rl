#!/usr/bin/env python3

from robot_sim.bresenham import bresenham
import numpy as np


MOVE_IDS = {'N': 0, 'E': 1, 'S': 2, 'W': 3}
MOVE_PROB_ALLOC = [0, -1, +1, -2]

MOVE = [(0, 1),     # Up
        (1, 0),     # Right
        (0, -1),    # Down
        (-1, 0)]    # Left


class RobotSimulator2D:
    """
    A class that defines a simple environment for a robot including its interaction with it (sensing/moving).
    """

    def __init__(self, world: np.ndarray = None, pos: list = None, size_x: int = None, size_y: int = None,
                 num_objects: int = None, num_sensors: int = 4,
                 move_variance: np.ndarray = np.array([0.9, 0.04, 0.04, 0.0, 0.02])):
        """
        Initializes the class RobotEnvironment.

        @param size_x: The size of the world in x direction.
        @param size_y: The size of the world in y direction.
        @param num_objects: The number of objects.
        @param num_sensors: The number of sensors (4 or 8), defaults to 4.
        @param move_variance: The variance of the movement. Defaults to [0.9, 0.04, 0.04, 0.0, 0.02]
                                                            -> [prob. that it moved in expected direction,
                                                                prob. that it moved left of the intended direction,
                                                                prob. that it moved right of the intended direction,
                                                                prob. that it moved backwards,
                                                                prob. that it stayed in position]
        """
        if world is None:
            self.world = np.zeros((size_x, size_y), dtype=np.int8)
            self.size_x = size_x
            self.size_y = size_y

            self.init_world(num_objects)
        else:
            self.world = world
            self.size_x = world.shape[0]
            self.size_y = world.shape[1]

        if pos is None:
            self.pos = np.array([0, 0])
            self.init_position()
        else:
            self.pos = np.array(pos)

        if not num_sensors in [4, 8]:
            self.num_sensors = 4
        self.num_sensors = num_sensors

        self.move_variance = move_variance

    def init_world(self, num_objects: int):
        """
        Initializes the world with objects.

        @param size_x: The size of the world in x direction.
        @param size_y: The size of the world in y direction.
        @param num_objects: The number of objects.
        """
        for i in range(num_objects):
            obj_x = np.random.randint(self.size_x)
            obj_y = np.random.randint(self.size_y)

            while self.world[obj_x, obj_y] == 1:
                obj_x = np.random.randint(self.size_x)
                obj_y = np.random.randint(self.size_y)

            self.world[obj_x, obj_y] = 1

    def init_position(self, position: tuple = None) -> bool:
        """
        Initializes the position to the one given by the parameter, if provided and valid (free). In case the position
        parameter is None, a random position will be generated within the free space of the grid-world.

        @param position: The initial position of the robot (x, y).
        @return: True if cell at given position is valid (free), False otherwise (occupied).
        """
        if position is None:
            pos = np.array([np.random.randint(self.size_x), np.random.randint(self.size_y)])
            while self.world[tuple(pos)] != 0 or np.all(self.pos == pos):
                pos = np.array([np.random.randint(self.size_x), np.random.randint(self.size_y)])
            self.pos = pos
        else:
            if not self.cell_valid(*position):
                return False
            else:
                self.pos = np.array(position)

        return True

    def sense(self) -> list:
        """
        Measures the ranges around the robot with the predifined numbers of sensors. The output distances are measured
        clockwise, starting from South.

        @return: A list of float values containing the measured distances.
        """
        beams = [[self.pos[0], 0],              # S
                 [0, self.pos[1]],              # W
                 [self.pos[0], self.size_y-1],  # N
                 [self.size_x-1, self.pos[1]]]  # E

        if self.num_sensors == 8:
            beams_a = min(self.size_x - self.pos[0], self.size_y - self.pos[1])
            beams_b = min(self.size_x - self.pos[0], self.pos[1])
            beams_c = min(self.pos[0], self.pos[1])
            beams_d = min(self.pos[0], self.size_y - self.pos[1])
            beams = [beams[0],                                              # S
                     [self.pos[0] - beams_c, self.pos[1] - beams_c],        # SW
                     beams[1],                                              # W
                     [self.pos[0] - beams_d, self.pos[1] + beams_d],        # NW
                     beams[2],                                              # N
                     [self.pos[0] + beams_a, self.pos[1] + beams_a],        # NE
                     beams[3],                                              # E
                     [self.pos[0] + beams_b, self.pos[1] - beams_b]]        # SE

        measured_distances = []

        for beam_x, beam_y in beams:
            hit_cells = bresenham(self.pos[0], self.pos[1], beam_x, beam_y)

            measured_distance = -1
            cell_hit = False

            for cell in hit_cells:
                measured_distance += 1
                if self.world[cell] > 0:
                    measured_distances.append(measured_distance)
                    cell_hit = True
                    break

            if not cell_hit:
                measured_distances.append(measured_distance+1)

        return measured_distances

    def move(self, move_dir: str = None) -> bool:
        """
        Moves the robot either in the defined direction or in a random, free direction, in case move_dir is none.

        @param move_dir: The direction in which the robot is supposed to move (N, S, E, W)
        @return: True if the move was successful, False otherwise.
        """

        if move_dir is None:
            # Generate random movement while generated move is not valid
            move_id = np.random.randint(len(MOVE))

            while not self.move_valid(move_id):
                move_id = np.random.randint(len(MOVE))
        
        else:
            move_id = MOVE_IDS[move_dir.upper()]

            while not self.move_valid(move_id):
                return False

        # Generate real movement based on command and move variance
        new_move_id = self.generate_prob_move(move_id)
        while not self.move_valid(new_move_id):
            new_move_id = self.generate_prob_move(move_id)
        if new_move_id == -1:
            return True

        self.pos[0] += MOVE[new_move_id][0]
        self.pos[1] += MOVE[new_move_id][1]

        return True

    def cell_in_world(self, x, y) -> bool:
        """
        Checks whether the cell with the given coordinates is located within the boundaries of the world.

        @param x: The x-coordinate of the cell.
        @param y: The y-coordinate of the cell.
        @return: True, if the cell lies within the world, False otherwise.
        """
        return 0 <= x < self.size_x and 0 <= y < self.size_y

    def cell_valid(self, x, y) -> bool:
        """
        Checks whether the cell with the given coordinates is located within the boundaries of the world and is not
        occupied (free).

        @param x: The x-coordinate of the cell.
        @param y: The y-coordinate of the cell.
        @return: True, if the cell lies within the world and is free, False otherwise.
        """
        return self.cell_in_world(x, y) and self.world[x, y] == 0

    def move_valid(self, move_id) -> bool:
        """
        Checks whether the move command with the given id would result in a cell that is located within the boundaries
        of the world and is not occupied (free).

        @param move_id: The id of the move direction.
        @return: True, if the cell lies within the world and is free, False otherwise.
        """
        return self.cell_in_world(self.pos[0] + MOVE[move_id][0], self.pos[1] + MOVE[move_id][1]) \
                    and self.world[self.pos[0] + MOVE[move_id][0], self.pos[1] + MOVE[move_id][1]] == 0

    def generate_prob_move(self, move_id) -> int:
        value = np.random.rand()
        total_prob = 0
        prob_id = -1

        for i, prob in enumerate(self.move_variance):
            total_prob += prob
            if value <= total_prob:
                prob_id = i
                break

        if prob_id == len(self.move_variance)-1:
            move_id = -1
        else:
            move_id += MOVE_PROB_ALLOC[prob_id]
            move_id += len(MOVE)
            move_id = move_id%len(MOVE)

        return move_id
