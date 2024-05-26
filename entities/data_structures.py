import math
import pygame


class SpatialHashGrid:
    def __init__(self, boundary, particle_count, particle_radius):
        self.boundary = boundary
        self.particle_count = particle_count
        self.particle_radius = particle_radius
        self.average_distance = math.sqrt(self.boundary.right * self.boundary.bottom / (self.particle_count / 2))
        self.cell_dimension = max(self.average_distance, self.particle_radius)
        self.cells = {}

    def draw_grid(self, screen, color=(255, 255, 255)):
        """
        Draw the grid on a Pygame screen.
        :param screen:
        :param color:
        :return:
        """
        # draw vertical lines
        for x in range(int(self.boundary.left), int(self.boundary.right), int(self.cell_dimension)):
            pygame.draw.line(screen, color, (x, self.boundary.top), (x, self.boundary.bottom))

        # draw horizontal lines
        for y in range(int(self.boundary.top), int(self.boundary.bottom), int(self.cell_dimension)):
            pygame.draw.line(screen, color, (self.boundary.left, y), (self.boundary.right, y))

    def calculate_cell_index(self, position):
        """
        Calculates the cell index in the grid for a give position
        :param position:
        :return:
        """
        x, y = position
        return int(x / self.cell_dimension), int(y / self.cell_dimension)

    def insert_particle(self, particle):
        """
        Insert a particle into the appropriate cell in the grid
        :param particle:
        :return:
        """
        cell_index = self.calculate_cell_index(particle.position)
        if cell_index not in self.cells:
            self.cells[cell_index] = []

        self.cells[cell_index].append(particle)

    def get_neighbouring_cell(self, cell_index):
        """
        Retrieves the indices of neighbouring cells to a given cell.
        :param cell_index:
        :return:
        """
        x, y = cell_index
        return [(x + dx, y + dy) for dx in range(-1, 2) for dy in range(-1, 2)]

    def nearby_to(self, particle):
        """
        Finds and returns particles that are nearby to a given particle
        :param particle:
        :return:
        """
        cell_index = self.calculate_cell_index(particle.position)
        nearby_particles = []

        for neighbouring_cell in self.get_neighbouring_cell(cell_index):
            if neighbouring_cell in self.cells:
                nearby_particles.extend([other_particle for other_particle in self.cells[neighbouring_cell] if other_particle != particle])

        return nearby_particles
