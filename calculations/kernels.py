import math


def spiky_smoothing_kernel_2d(distance, smoothing_radius):
    """
    Calculate the 2D spiky smoothing kernel value for a given distance
    :param distance:
    :param smoothing_radius:
    :return:
    """
    if 0 <= distance <= smoothing_radius:
        normalisation_constant = 10 / (math.pi * smoothing_radius ** 5)
        spiky_kernel = (smoothing_radius - distance) ** 3
        return spiky_kernel * normalisation_constant
    else:
        return 0


def spiky_smoothing_kernel_2d_derivative(distance, smoothing_radius):
    """
    Calculate the derivative of the 2D spiky smoothing kernel with respect to distance.
    :param distance:
    :param smoothing_radius:
    :return:
    """
    if 0 <= distance <= smoothing_radius:
        normalisation_constant = -30 / (math.pi * smoothing_radius ** 5)
        spiky_kernel_derivative = (smoothing_radius - distance) ** 2
        return spiky_kernel_derivative * normalisation_constant

    else:
        return 0
