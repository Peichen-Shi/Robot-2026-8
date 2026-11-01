#!/usr/bin/env python3
"""Checks the scalar equation count for the standard piecewise minimum-r model."""


def count(r, k):
    unknowns = 2 * r * k
    waypoint_equations = 2 * k
    free_derivative_equations = 2 * (r - 1) * (k - 1)
    endpoint_equations = 2 * (r - 1)
    total = waypoint_equations + free_derivative_equations + endpoint_equations
    return unknowns, waypoint_equations, free_derivative_equations, endpoint_equations, total


def main():
    for r in range(1, 8):
        for k in range(1, 8):
            values = count(r, k)
            assert values[0] == values[-1], (r, k, values)
    assert count(4, 3) == (24, 6, 12, 6, 24)
    assert count(4, 1) == (8, 2, 0, 6, 8)
    assert count(1, 3) == (6, 6, 0, 0, 6)
    print("r=4, k=3: unknowns=24, waypoint=6, free=12, endpoint=6, total=24")
    print("Boundary cases k=1 and r=1 passed; general r,k grid passed.")


if __name__ == "__main__":
    main()
