import numpy as np

class DataLog():
    def __init__(self):
        self.data = {}

    def allocate_data(self, key: str, iterations: int, length: int):
        self.data[key] = np.empty([iterations, length], dtype=float)

    def add_point(self, key: str, k: int, array):
        arr = np.asarray(array, dtype=float).reshape(-1)
        self.data[key][k] = arr