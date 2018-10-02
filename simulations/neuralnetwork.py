from typing import Deque
import numpy as np
from collections import deque


class PIDNN:
    def __init__(self, learning_rate, samples) -> None:
        # Network states
        # Inputs
        self.inputs = deque()  # type: Deque
        self.inputs.append(np.array([0, -0.1]))

        # Hidden layer
        self.hidden_weights = np.array([
            [1.0, +1.0],
            [1.0, +1.0],
            [1.0, +1.0]])
        self.u_hidden = deque()  # type: Deque
        self.u_hidden.append(np.zeros(3))
        self.x_hidden = deque()  # type: Deque
        self.x_hidden.append(np.zeros(3))

        # Output
        self.output_weights = np.array([0.1, 0.2, 0.3])
        self.u_output = deque()  # type: Deque
        self.u_output.append(np.zeros(1))
        self.x_output = deque()  # type: Deque
        self.x_output.append(np.zeros(1))

        # Learning Parameters
        self.learning_rate = learning_rate

        self.samples = samples
        self.numiterations = 1

        self.shouldStopPropagating = False

    def _P_activation(self, u: np.float64) -> np.float64:
        return np.clip(u, -1, 1)

    def _I_activation(self, u: np.float64, u_last: np.float64) -> np.float64:
        return np.clip(u + u_last, -1, 1)

    def _D_activation(self, u: np.float64, u_last: np.float64) -> np.float64:
        return np.clip(u - u_last, -1, 1)

    def _output_activation(self, u: np.float64):
        return u

    def reset(self):
        self.inputs.clear()
        self.u_hidden.clear()
        self.x_hidden.clear()
        self.u_output.clear()
        self.x_output.clear()
        self.inputs.append(np.array([0, -0.1]))
        self.u_hidden.append(np.zeros(3))
        self.x_hidden.append(np.zeros(3))
        self.u_output.append(np.zeros(1))
        self.x_output.append(np.zeros(1))

    def backpropagate(self):
        J = 0
        m = self.samples
        d_output_weights = np.zeros(self.output_weights.shape)
        d_hidden_weights = np.zeros(self.hidden_weights.shape)
        for k in range(1, len(self.inputs)):
            r_k = self.inputs[k][0]
            y_k = self.inputs[k][1]
            print(r_k, y_k)
            y_k_1 = self.inputs[k - 1][1]
            v_k = self.x_output[k]
            v_k_1 = self.x_output[k - 1]
            x_p = self.x_hidden[k]
            x_p_1 = self.x_hidden[k - 1]
            u_p = self.u_hidden[k]
            u_p_1 = self.u_hidden[k - 1]
            x = self.inputs[k]

            # Backpropagate output layer
            delta_j = (r_k - y_k) * (y_k - y_k_1) / (v_k - v_k_1)
            d_output_weights += delta_j * x_p

            # Backpropagate hidden layer
            delta_i = delta_j * (x_p - x_p_1)/(u_p - u_p_1)
            d_hidden_weights += \
                np.column_stack((x[0] * delta_i, x[1] * delta_i))

            J += (r_k - y_k) ** 2

        self.output_weights -= 2/m * d_output_weights * self.learning_rate
        self.hidden_weights -= 2/m * d_hidden_weights * self.learning_rate

        self.inputs.clear()
        self.u_hidden.clear()
        self.x_hidden.clear()
        self.u_output.clear()
        self.x_output.clear()

    def predict(self, theta, ref):
        # Store old states
        # Apply weighted sum to inputs to hidden layer
        new_inputs = np.array([ref, theta])
        new_u_hidden = self.hidden_weights @ new_inputs

        # Apply activation function on hidden layer
        new_x_hidden = np.array([
            self._P_activation(new_u_hidden[0]),
            self._I_activation(new_u_hidden[1], self.u_hidden[-1][1]),
            self._I_activation(new_u_hidden[2], self.x_hidden[-1][2]),
        ])

        # Update output layer
        new_u_output = self.output_weights @ new_x_hidden
        new_x_output = self._output_activation(new_u_output)

        if len(self.inputs) % self.samples == 0:
            # This will update the weights, and clear the input array
            self.backpropagate()

        self.inputs.append(new_inputs)
        self.u_hidden.append(new_u_hidden)
        self.x_hidden.append(new_x_hidden)
        self.u_output.append(new_u_output)
        self.x_output.append(new_x_output)
        self.numiterations += 1

        return new_x_output


if __name__ == "__main__":
    pidnn = PIDNN(0.01, 3)
    print(pidnn.predict(-0.1, 0))
    print(pidnn.predict(-0.11, 0))
    print(pidnn.predict(-0.1, 0))
    print(pidnn.predict(-0.05, 0))
    print(pidnn.predict(-0.0, 0))
    print(pidnn.predict(-0.0, 0))
    print(pidnn.predict(-0.0, 0))
    print(pidnn.predict(-0.0, 0))
    print(pidnn.predict(-0.0, 0))
    print(pidnn.predict(-0.0, 0))
    print(pidnn.predict(-0.0, 0))
