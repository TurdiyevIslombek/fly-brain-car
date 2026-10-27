"""
flybrain.py - fast leaky integrate-and-fire simulation of the fruit fly brain.

Same neuron model and constants as Shiu et al. 2024 (Nature),
"A Drosophila computational brain model reveals sensorimotor processing".
Their code uses Brian2; this is a plain numpy/scipy version so it can run
in real time on a laptop and talk to the robot.

Wiring = FlyWire connectome v783 (synapse counts, signed by neurotransmitter).
Nothing here is trained. Weights are the fly's real synapse counts.
"""
import json
import numpy as np
import scipy.sparse as sp
from pathlib import Path

# ---- constants from Shiu et al. (model.py, default_params) ----
V_REST = -52.0     # mV resting / reset potential
V_TH = -45.0       # mV spike threshold
T_MBR = 20.0       # ms membrane time constant
TAU = 5.0          # ms synaptic time constant
T_REF = 2.2        # ms refractory period
T_DLY = 1.8        # ms synaptic delay
W_SYN = 0.275      # mV per synapse
POI_KICK = W_SYN * 250  # one Poisson input event = forced spike (68.75 mV)


class FlyBrain:
    def __init__(self, folder, dt=0.1, seed=0):
        folder = Path(folder)
        self.meta = json.loads((folder / "neurons.json").read_text())
        W = sp.load_npz(folder / "weights.npz").tocsr()     # rows = presynaptic
        self.N = W.shape[0]
        W = (W * W_SYN).astype(np.float32)
        # small circuit -> dense matrix (much faster); whole brain -> sparse
        self.W = W.toarray() if self.N <= 8000 else W.tocsr()
        self.dense = self.N <= 8000
        self.dt = dt
        self.rng = np.random.default_rng(seed)
        self.delay_steps = max(1, int(round(T_DLY / dt)))
        self.ref_steps = int(round(T_REF / dt))
        self.dv = np.exp(-dt / T_MBR)
        self.dg = np.exp(-dt / TAU)
        self.groups = {k: np.array(v, dtype=int) for k, v in self.meta["groups"].items()}
        self.reset()

    def reset(self):
        self.v = np.full(self.N, V_REST, np.float32)
        self.g = np.zeros(self.N, np.float32)
        self.ref = np.zeros(self.N, np.int32)
        self.buf = [np.zeros(0, int) for _ in range(self.delay_steps)]
        self.t = 0

    def step(self, input_rates):
        """Advance one dt. input_rates: {group_name: Hz}. Returns spike indices."""
        dt = self.dt
        # delayed synaptic input arrives
        arriving = self.buf[self.t % self.delay_steps]
        if arriving.size:
            if self.dense:
                self.g += self.W[arriving].sum(axis=0)
            else:
                self.g += np.asarray(self.W[arriving].sum(axis=0)).ravel()
        # membrane + synapse update (exact solution of the linear ODEs)
        active = self.ref <= 0
        v, g = self.v, self.g
        v_new = V_REST + (v - V_REST) * self.dv + g * (self.dg - self.dv) * (TAU / (TAU - T_MBR))
        # the line above is the exact integral of dv=(v0-v+g)/Tm, dg=-g/tau
        g_new = g * self.dg
        np.copyto(self.v, v_new, where=active, casting='unsafe')
        np.copyto(self.g, g_new, where=active, casting='unsafe')
        self.ref -= 1
        # sensory drive: Poisson events on input neurons (like optogenetics in the paper)
        for name, rate in input_rates.items():
            if rate <= 0:
                continue
            idx = self.groups[name]
            hit = idx[self.rng.random(idx.size) < rate * dt / 1000.0]
            self.v[hit] += POI_KICK
        spk = np.flatnonzero(self.v > V_TH)
        self.v[spk] = V_REST
        self.g[spk] = 0.0
        self.ref[spk] = self.ref_steps
        self.buf[self.t % self.delay_steps] = spk
        self.t += 1
        return spk

    def run(self, input_rates, ms, watch):
        """Run for `ms` milliseconds; return spike counts for each watched group."""
        counts = {k: 0 for k in watch}
        member = {k: np.isin(np.arange(self.N), self.groups[k]) for k in watch}
        for _ in range(int(ms / self.dt)):
            spk = self.step(input_rates)
            if spk.size:
                for k in watch:
                    counts[k] += int(member[k][spk].sum())
        return counts
