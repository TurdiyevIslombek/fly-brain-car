"""Wiring variants for the connectome control test.

real      : FlyWire synapse counts as used on the car (unchanged).
shuffled  : the same synapses with their targets shuffled. Every neuron keeps how many
            synapses it sends and receives and its sign (excitatory/inhibitory), and the
            weight values are the same. Only WHO connects to WHOM is random.
"""
import numpy as np
import scipy.sparse as sp
from pathlib import Path

PROJ = Path(__file__).resolve().parent.parent
W_PATH = PROJ / "brain" / "locomotion" / "weights.npz"


def load_real():
    return sp.load_npz(W_PATH).tocsr()


def shuffled(seed):
    """Degree-preserving target shuffle: permute the column (target) of every synapse.
    Rows (senders) keep their out-degree and weights; each target keeps its in-degree."""
    W = sp.load_npz(W_PATH).tocoo()
    rng = np.random.default_rng(10_000 + seed)
    col = W.col.copy()
    rng.shuffle(col)
    out = sp.coo_matrix((W.data, (W.row, col)), shape=W.shape).tocsr()
    out.sum_duplicates()
    return out


def install(driver, W):
    """Put wiring W (synapse counts, signed) into a BrainDriver's FlyBrain."""
    from flybrain import W_SYN
    b = driver.brain
    b.W = (W * W_SYN).astype(np.float32).toarray()
    b.dense = True
    b.reset()
