import numpy as np


class PIDNN:
    def __init__(self) -> None:
        # Network states
        # Inputs
        self.inputs = 0

        # Hidden layer
        self.hidden_weights = np.array([
            [1, -1],
            [1, -1],
            [1, -1]])
        self.u_hidden = np.zeros(3)
        self.x_hidden = np.zeros(3)

        # Output
        self.output_weights = np.array([0.1, 0.1, 0.1])
        self.u_output = np.zeros(1)
        self.x_output = np.zeros(1)

        # Learning Parameters
        self.learning_rate = 1

    def _P_activation(self, u: np.float64) -> np.float64:
        return np.clip(u, -1, 1)

    def _I_activation(self, u: np.float64, u_last: np.float64) -> np.float64:
        return np.clip(u + u_last, -1, 1)

    def _D_activation(self, u: np.float64, u_last: np.float64) -> np.float64:
        return np.clip(u - u_last, -1, 1)

    def _output_activation(self, u: np.float64):
        return np.clip(u, -1, 1)

    def predict(self, x: np.ndarray):
        # Store old states
        old_inputs = self.inputs
        old_u_hidden = self.u_hidden
        old_x_hidden = self.x_hidden
        old_u_output = self.u_output
        old_x_output = self.x_output

        # Apply weighted sum to inputs to hidden layer
        new_inputs = x[1]
        new_u_hidden = self.hidden_weights @ x

        # Apply activation function on hidden layer
        new_x_hidden = np.array([
            self._P_activation(new_u_hidden[0]),
            self._I_activation(new_u_hidden[1], old_x_hidden[1]),
            self._I_activation(new_u_hidden[2], old_u_hidden[2]),
        ])

        # Update output layer
        new_u_output = self.output_weights @ new_x_hidden
        new_x_output = self._output_activation(new_u_output)

        # Apply backpropagation to weights
        # Output to hidden layer
        r = x[0]
        delta_j = (r - new_inputs) * \
            (new_inputs - old_inputs)/(new_x_output - old_x_output)
        d_output_weights = self.learning_rate * delta_j * new_x_hidden
        self.output_weights = self.output_weights - d_output_weights

        # Hidden layer to input layer
        delta_i = delta_j * \
            (new_x_hidden - old_x_hidden)/(new_u_hidden - old_u_hidden)
        d_hidden_weights = self.learning_rate * \
            np.column_stack((x[0] * delta_i, x[1] * delta_i))
        self.hidden_weights = self.hidden_weights - d_hidden_weights

        # Update states
        self.inputs = new_inputs
        self.u_hidden = new_u_hidden
        self.x_hidden = new_x_hidden
        self.u_output = new_u_output
        self.x_output = new_x_output

        return new_x_output


if __name__ == "__main__":
    pidnn = PIDNN()
