#!/usr/bin/env python3
"""Exact KKT verification for LAB4 Deliverable 1."""

from fractions import Fraction


def solve(matrix, rhs):
    a = [list(map(Fraction, row)) + [Fraction(value)]
         for row, value in zip(matrix, rhs)]
    n = len(a)
    for column in range(n):
        pivot = next(row for row in range(column, n) if a[row][column])
        a[column], a[pivot] = a[pivot], a[column]
        scale = a[column][column]
        a[column] = [value / scale for value in a[column]]
        for row in range(n):
            if row == column:
                continue
            scale = a[row][column]
            if scale:
                a[row] = [left - scale * right
                          for left, right in zip(a[row], a[column])]
    return [a[row][-1] for row in range(n)]


def q_matrix(degree):
    return [[Fraction((i + 1) * (j + 1), i + j + 1)
             for j in range(degree)] for i in range(degree)]


def kkt_solution(degree, final_velocity=None):
    q = q_matrix(degree)
    constraints = [[Fraction(1) for _ in range(degree)]]
    target = [Fraction(1)]
    if final_velocity is not None:
        constraints.append([Fraction(i + 1) for i in range(degree)])
        target.append(Fraction(final_velocity))
    m = len(constraints)
    # Stationarity for p^T Q p uses 2Q p + A^T lambda = 0.
    matrix = []
    rhs = []
    for row in range(degree):
        matrix.append([2 * q[row][column] for column in range(degree)] +
                      [constraints[c][row] for c in range(m)])
        rhs.append(Fraction(0))
    for c in range(m):
        matrix.append(constraints[c] + [Fraction(0)] * m)
        rhs.append(target[c])
    solution = solve(matrix, rhs)
    coefficients = solution[:degree]
    cost = sum(coefficients[i] * q[i][j] * coefficients[j]
               for i in range(degree) for j in range(degree))
    residuals = [sum(row[i] * coefficients[i] for i in range(degree)) - value
                 for row, value in zip(constraints, target)]
    stationarity = [
        2 * sum(q[i][j] * coefficients[j] for j in range(degree)) +
        sum(constraints[c][i] * solution[degree + c] for c in range(m))
        for i in range(degree)
    ]
    return coefficients, cost, residuals, stationarity


def main():
    expected = {
        (1, None): ([Fraction(1)], Fraction(1)),
        (2, None): ([Fraction(1), Fraction(0)], Fraction(1)),
        (3, None): ([Fraction(1), Fraction(0), Fraction(0)], Fraction(1)),
        (2, -2): ([Fraction(4), Fraction(-3)], Fraction(4)),
        (3, -2): ([Fraction(1, 4), Fraction(9, 2), Fraction(-15, 4)],
                  Fraction(17, 8)),
    }
    print("degree, terminal_velocity, coefficients, cost, max_residual")
    for problem, reference in expected.items():
        coefficients, cost, residuals, stationarity = kkt_solution(*problem)
        assert coefficients == reference[0]
        assert cost == reference[1]
        assert not any(residuals)
        assert not any(stationarity)
        all_residuals = residuals + stationarity
        print(problem[0], problem[1], coefficients, cost,
              max(map(abs, all_residuals), default=Fraction(0)), sep=", ")
    print("All exact KKT, boundary, and integral-cost checks passed.")


if __name__ == "__main__":
    main()
