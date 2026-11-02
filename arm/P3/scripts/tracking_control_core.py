import time

import numpy as np
import osqp
from scipy import sparse


class Reference:
    def __init__(self, path, evaluator):
        data = np.load(path, allow_pickle=False)
        self.starts = data['segment_starts']
        self.durations = data['segment_durations']
        self.coeff = data['coefficients']
        self.names = list(data['joint_names'])
        self.duration = float(self.starts[-1]+self.durations[-1])
        self.evaluator = evaluator

    def at(self, t):
        return self.evaluator(t, self.starts, self.durations, self.coeff)


class TrackingController:
    def __init__(self, mode, config, lower, upper, reference):
        if mode not in ('p', 'mpc'):
            raise ValueError('Unknown controller')
        self.mode, self.c, self.lower, self.upper, self.reference = mode, config, lower, upper, reference
        if mode == 'mpc':
            n, ts = config['horizon'], config['ts']
            self.s = sparse.kron(sparse.eye(6), sparse.csc_matrix(ts*np.tril(np.ones((n, n)))), format='csc')
            self.d = sparse.kron(sparse.eye(6), sparse.csc_matrix(np.eye(n)-np.eye(n, k=-1)), format='csc')
            eye = sparse.eye(6*n, format='csc')
            h = config['q_weight']*(self.s.T@self.s)+config['v_weight']*eye+config['du_weight']*(self.d.T@self.d)
            self.solver = osqp.OSQP()
            self.solver.setup(P=sparse.triu(2*h, format='csc'), q=np.zeros(6*n),
                              A=sparse.vstack([eye, self.d, self.s], format='csc'),
                              l=-np.ones(18*n), u=np.ones(18*n), verbose=False,
                              eps_abs=1e-6, eps_rel=1e-6, max_iter=10000,
                              time_limit=config['solver_time_limit_s'])

    def command(self, q, previous, elapsed):
        began = time.perf_counter()
        c = self.c
        target, velocity, _ = self.reference.at(elapsed)
        raw = velocity+c['kp']*(target-q)
        status, fallback, iterations = 'baseline_no_solver', False, 0
        if self.mode == 'mpc':
            n = c['horizon']
            targets, velocities, _ = self.reference.at(elapsed+(np.arange(n)+1)*c['ts'])
            prior = np.zeros((6, n)); prior[:, 0] = previous
            prior = prior.ravel()
            repeat_q = np.repeat(q, n)
            linear = c['q_weight']*self.s.T@(repeat_q-targets.T.ravel())
            linear -= c['v_weight']*velocities.T.ravel()
            linear -= c['du_weight']*self.d.T@prior
            delta = np.full(6*n, c['rate']*c['ts'])
            bound = np.full(6*n, c['vmax'])
            lo = np.r_[-bound, prior-delta, np.repeat(self.lower, n)-repeat_q]
            hi = np.r_[bound, prior+delta, np.repeat(self.upper, n)-repeat_q]
            try:
                self.solver.update(q=2*linear, l=lo, u=hi)
                result = self.solver.solve()
                status, iterations = result.info.status, int(result.info.iter)
                if result.info.status_val != 1 or result.x is None or not np.all(np.isfinite(result.x)):
                    raise RuntimeError('solver_not_solved:'+status)
                predicted = np.r_[result.x, self.d@result.x, self.s@result.x]
                if np.any(predicted < lo-c['constraint_tolerance']) or np.any(predicted > hi+c['constraint_tolerance']):
                    raise RuntimeError('predicted_constraint_residual_exceeded')
                raw = result.x[::n].copy()
            except Exception as exc:
                status, fallback = str(exc), True
        change = c['rate']*c['ts']
        u = np.clip(raw, np.maximum(-c['vmax'], previous-change), np.minimum(c['vmax'], previous+change))
        return u, {'compute_wall_s': time.perf_counter()-began, 'status': status,
                   'fallback': fallback, 'iterations': iterations,
                   'limiter_correction_rad_s': float(np.max(np.abs(u-raw)))}


def plant_step(q, velocity, applied, dt, tau):
    if tau == 0:
        return q+dt*applied, applied.copy()
    decay = np.exp(-dt/tau)
    q_next = q+applied*dt+(velocity-applied)*tau*(1-decay)
    v_next = applied+(velocity-applied)*decay
    return q_next, v_next
