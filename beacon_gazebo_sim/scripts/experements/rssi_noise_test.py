import numpy as np
import matplotlib.pyplot as plt
import math


class Generator:
    def __init__(self):
        self.n1_update_p = 200 / 15
        self.n1_a_T = 30 # TT

        self.n1_b = 5
        self.n1_c = 2
        self.n1_prev_t = None
        self.n1_prev_vo = 0
        self.n1_go = False

        self.n1_b_prev_t = None
        self.n1_b_update_t = self.n1_a_T / 23
        self.n1_b_prev_v = None
        self.n1_b_next_v = None
        self.n2_a = 0.4
        self.n1_b_prev_vo = 0

    def n1(self, m: float, t: float) -> float:
        if self.n1_prev_t is None:
            self.n1_prev_t = t
            return 0
        #
        if (t - self.n1_prev_t) >= ((np.random.random()) + 1)*self.n1_update_p and not self.n1_go:
            self.n1_go = True
            self.n1_go_start = t
            self.n1_go_time = (1 + 1*(-np.random.random()))*self.n1_a_T
            self.n1_b_prev_t = t
            self.n1_b_next_v = 0
            self.n1_b_prev_v = 0
        if self.n1_go:
            if (t - self.n1_go_start) >= self.n1_go_time:
                self.n1_go = False
                self.n1_prev_t = t

            if (t - self.n1_b_prev_t) >= self.n1_b_update_t:
                self.n1_b_prev_v = self.n1_b_next_v

                # self.n1_b_next_v = np.random.laplace(0, self.n1_b, 1)[0]
                self.n1_b_next_v = np.random.normal(0, self.n1_b, 1)[0]
                if np.random.random() > 0.6:
                    self.n1_b_next_v = self.n1_b_prev_v*0.3 + self.n1_b_next_v*0.7
                self.n1_b_prev_t = t
            k = ((t - self.n1_b_prev_t)/self.n1_b_update_t)
            vo = (self.n1_b_next_v*k + self.n1_b_prev_v*(1-k))*0.5 + self.n1_b_prev_vo*0.5
            self.n1_b_prev_vo = vo
            return vo
        else:
            return 0

    def n2(self, m, t):
        return np.random.normal(0, self.n2_a, 1)[0]

    def gen(self, distance: float, t: float) -> float:
        m = -60 * math.pow(((distance - 0) / 1), (1.0 / 10))
        n1 = self.n1(m, t)
        n2 = self.n2(m, t)
        return m + np.clip(n2, -5, 5) + np.clip(n1, -7, 7)


def get_d(time):
    return 5

time_all = 200

update_rate = 50

t = 0

r_list = []
d_list = []

gen = Generator()

while t < time_all:
    d = get_d(t)
    val = gen.gen(d, t)
    r_list.append(val)
    d_list.append(d)
    t += 1/update_rate

r_vals = np.array(r_list)
d_vals = np.array(d_list)

print(f"mean: {r_vals.mean()}  std: {r_vals.std()}")

plt.hist(r_vals, bins=30)
plt.show()
plt.plot(np.linspace(0, 200, len(r_vals)), r_vals)
plt.ylim(-100, 0)
plt.show()



