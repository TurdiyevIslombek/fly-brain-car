# Data in this folder

`neurons.json`, `positions.csv` and `weights.npz` are derived from the FlyWire whole-brain connectome,
version 783: FlyWire Consortium, *FlyWire Whole-brain Connectome Connectivity Data*, Zenodo,
doi:10.5281/zenodo.10676866. Licensed under CC BY 4.0 (https://creativecommons.org/licenses/by/4.0/).
Cell types from Schlegel et al. 2024.

Changes made: a subset of 2,282 neurons was selected (looming-detector eye neurons LC4 and LPLC2,
12 descending neurons, and the neurons between them); connections were kept as synapse counts and
signed by the sending neuron's predicted neurotransmitter; the data was reformatted into these three files.
No neuron, weight or connection was changed by hand.
