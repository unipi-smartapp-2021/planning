from math import inf
import matplotlib.pyplot as plt
import numpy as np
from typing import Callable, List

# def f(x):
#     if -2 <= x < -1:
#         return 2+x
#     if -1 <= x <= 1:
#         return 1
#     if 1 < x <= 2:
#         return 2-x

# TODO: make it so that you only pass xs and ys and it automatically computs
#       an f which is piece-wise linear
# TODO: Assert that f >= 0 (maybe? It kind of works)
def Bernstein(f, a, b, N=10):
    # phi manda il nostro intervallo [a;b] in [0;1]
    phi = lambda x: (x-a)/(b-a)
    phi_inv = lambda x: a + (b-a)*x
    print(f(a), f(b))

    g = lambda x: f(phi_inv(x))

    def bernstein_poly(x):
        x = phi(x)
        f_bernstein = 0
        nCr = lambda n, r: np.math.factorial(n) / (np.math.factorial(r) * np.math.factorial(n - r))
        for n in range(1, N+1):
            f_bernstein += g(n/N) * nCr(N, n) * (x**n) * ((1-x)**(N-n))
        return f_bernstein

    return bernstein_poly

# def Bernstein(f, a, b, min_empirical_error=1e-1, N_MAX=100):
#     for N in range(1, N_MAX):
#         B = Bernstein(f, a, b, N)
#         max_error = -inf
#         for x in np.linspace(a, b, 100):
#             max_error = max(max_error, abs(B(x)-f(x)))
#         if max_error < min_empirical_error:
#             return B
#     print(f"Warning: N_MAX reached. Couldn't find an approx with min_empirical_error = {min_empirical_error}")
#     return B


def Bernstein_spline(xs, ys) -> List[Callable[[float], float]]:
    assert len(xs) >= 2
    xs_i, ys_i = [], []
    i = 1
    Right = 1
    Left  = 0
    direction = Right if xs[0] < xs[1] else Left
    lista = [0]
    while i < len(xs):
        k = i
        is_monotone = True
        while k < len(xs) and is_monotone:
            if xs[k] > xs[k-1] and direction == Right:
                lista.append(k)
                k += 1
            elif xs[k] < xs[k-1] and direction == Left:
                lista.append(k)
                k += 1
            else:
                direction = Left if direction == Right else Right
                is_monotone = False
        i = k
        xs_i.append([xs[j] for j in lista])
        ys_i.append([ys[j] for j in lista])
        lista = [k-1 if k < len(xs) else None]
    print(xs_i)
    print(ys_i)

def piecewise_linear(xs, ys):
    def f(x):
        for i in range(len(xs)-1):
            if xs[i] <= x and x < xs[i+1]:
                return ys[i] + (ys[i+1]-ys[i])*(x-xs[i])/(xs[i+1]-xs[i])
        return ys[-1]
    return f

f = piecewise_linear([0, 1, 2, 3, 4, 10], [1, 1, 4, 9, -30, 100])
xs = np.linspace(0, 10, 100)
plt.plot(xs, [f(x) for x in xs], label='f')

f_bern = Bernstein(f, 0, 10, N=300)
print(f_bern(0))
xs = np.linspace(0, 10, 100)
plt.plot(xs, [f_bern(x) for x in xs], label='f_bern')
plt.legend()
plt.show()

Bernstein_spline([0, 1, -1, -2, 3, 4], [0, 1, 1, 4, 9, 16])


# a = -2
# b = 2
# n_points = 100
# xs = np.linspace(a, b, n_points)
# ys = [f(x) for x in xs]
# f_xs = [Bernstein(f, a, b, N=50)(x) for x in xs]
# plt.plot(xs, list(map(f, xs)), label="f(x)")
# plt.plot(xs, f_xs, label="Bernstein")
# plt.legend()
# plt.show()



# -----------------------------------------------------------------------------

# # [a, b] è l'intervallo su cui stiamo discretizzando con n_points
# a = -2
# b = 2

# # phi manda il nostro intervallo [a;b] in [0;1]
# phi = lambda x: (x-a)/(b-a)
# phi_inv = lambda x: a + (b-a)*x

# g = lambda x: f(phi_inv(x))

# n_points = 100

# xs = list(np.linspace(a, b, n_points))
# phi_xs = list(map(phi, xs))

# # N è quello che mi dà L'n-esimo polinomio di Bernstein
# N = 50

# fs = [] # Approssima xs col polinomio di Bernstein
# for x in xs:
#     x = phi(x)
#     f_bernstein = 0
#     nCr = lambda n, r: np.math.factorial(n) / (np.math.factorial(r) * np.math.factorial(n - r))
#     for n in range(1, N+1):
#         f_bernstein += g(n/N) * nCr(N, n) * (x**n) * ((1-x)**(N-n))
#     fs.append(f_bernstein)


# plt.plot(xs, list(map(f, xs)), label="f(x)")
# plt.plot(xs, fs, label="Bernstein")
# plt.legend()
# plt.show()