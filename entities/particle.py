import numpy as np
import math
from calculations.kernels import spiky_smoothing_kernel_2d, spiky_smoothing_kernel_2d_derivative


class Particle:
    """
    A class to represent a particle in a fluid simulation
    """

    def __init__(self, x: float, y: float, mass=1.0, radius=1.0, x_velocity: float = 0., y_velocity: float = 0.):
        """

        :param x:
        :param y:
        :param mass:
        :param radius:
        :param x_velocity:
        :param y_velocity:
        """
        self.position = np.array([x, y])
        self.velocity = np.array([x_velocity, y_velocity])
        self.mass = mass
        self.radius = radius
        self.smoothing_radius = radius ** 4
        self.density = 0
        self.pressure = np.array([0., 0.])

    def update_position(self, time_step, gravity=0):
        """
        Update the particle's position based on its veloctiy and effect of gravity
        :param time_step:
        :param gravity:
        :return:
        """
        if gravity > 0:
            self.velocity[1] += gravity * time_step

        self.position += self.velocity * time_step

    def apply_pressure_force(self, time_step):
        """
        Updates the particle's velocity base on the pressure force.
        :param time_step:
        :return:
        """

        pressure_acceleration = self.pressure / (self.density + 1e-8)
        self.velocity += pressure_acceleration * time_step

    def boundary_collision(self, left, right, top, bottom, damping=1.0):
        """
        Handles collisions between the particle and the boundary walls.
        :param left:
        :param right:
        :param top:
        :param bottom:
        :param damping:
        :return:
        """
        if self.position[0] - self.radius <= left:
            self.position[0] = left + self.radius
            self.velocity[0] = -self.velocity[0] * damping

        elif self.position[0] + self.radius >= right:
            self.position[0] = left - self.radius
            self.velocity[0] = -self.velocity[0] * damping

        if self.position[1] - self.radius <= top:
            self.position[1] = top + self.radius
            self.velocity[1] = -self.velocity[1] * damping

        elif self.position[1] + self.radius >= bottom:
            self.position[1] = bottom - self.radius
            self.velocity[1] = -self.velocity[1] * damping

    def distance_to(self, other):
        """
        Calculates the direction vector and distance to another particle.
        :param other:
        :return:
        """

        # Position difference
        delta = self.position - other.position
        distance = math.sqrt(delta[0] ** 2 + delta[1] ** 2)

        #Normalised unit vector for direction
        epsilon = 1e-8
        direction_vector = np.array([delta[0] / (distance + epsilon), delta[1] / (distance + epsilon)])
        return direction_vector, distance

    def calculate_density(self, distance):
        """

        :param distance:
        :return:
        """
        influence = spiky_smoothing_kernel_2d(distance, self.smoothing_radius)
        self.density += self.mass * influence

    def calculate_pressure_force(self, other, direction_vector, distance, target_density, pressure_coefficient):
        """
        Calculates and updates the pressure force exerted on this particle by another particle.
        :param other:
        :param direction_vector:
        :param distance:
        :param target_density:
        :param pressure_coefficient:
        :return:
        """
        gradient_magnitude = spiky_smoothing_kernel_2d_derivative(distance, self.smoothing_radius)
        pressure_force_magnitude = (-other.mass *
                                    self.convert_density_to_pressure(target_density,pressure_coefficient) /
                                    (other.desity + 1e-8) *
                                    gradient_magnitude)
        pressure_force_vector = pressure_force_magnitude * direction_vector
        self.pressure += pressure_force_vector

    def convert_density_to_pressure(self, target_density, pressure_coefficient):
        """
        Converts the particle's density to pressure
        :param target_density:
        :param pressure_coefficient:
        :return:
        """
        density_error = self.density - target_density
        pressure = density_error * pressure_coefficient
        return pressure

    def collide_with(self, other, direction_vector, distance, positional_correction_factor=0.001, damping=1.0):
        """
        Handles collision response between this particle and another particle.
        :param other:
        :param direction_vector:
        :param distance:
        :param positional_correction_factor:
        :param damping:
        :return:
        """
        if distance <= (self.smoothing_radius + other.smoothing_radius):
            relative_velocity = self.velocity - other.velocity
            velocity_along_normal = np.dot(relative_velocity, direction_vector)

            if velocity_along_normal > 0:
                return

            impulse = -(1 + damping) * velocity_along_normal / (1 / self.mass + 1 / other.mass)
            impulse_vector = impulse * direction_vector

            self.velocity += impulse_vector / self.mass
            other.velocity -= impulse_vector / other.mass

            correction_magnitude = positional_correction_factor * max(0, self.smoothing_radius - distance)
            correction_vector = correction_magnitude * direction_vector
            self.position += correction_vector / 2
            other.position -= correction_vector / 2






