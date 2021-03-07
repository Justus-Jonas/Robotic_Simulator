import numpy   as np
import math

sigmoid = lambda x: 1.0 / (1.0 + np.exp(-x))

class NeuralNetwork:
    def __init__(self, hiddenSize):
        self.InputSize = 12 + 2  # 12 sensors + current vl + rl
        self.HiddenSize = hiddenSize
        self.OutputSize = 2
        self.BiasesH = np.zeros((self.HiddenSize, 1))   # biases hidden layer
        self.BiasesO = np.zeros((self.OutputSize, 1))   # biases output layer
        self.WeightsH = np.random.randn(self.HiddenSize, self.InputSize)  # weights hidden layer
        self.WeightsO = np.random.randn(self.OutputSize, self.HiddenSize)  # weights output layer

    def feed_forward(self, input):
        activation_hidden = sigmoid(self.WeightsH @ input + self.BiasesH)
        activation_output = sigmoid(self.WeightsO @ activation_hidden + self.BiasesO)
        return activation_output

    def get_weights_biases(self):
        return self.WeightsH, self.BiasesH, self.WeightsO, self.BiasesO

    def update_weights_biases(self, weightsH, biasesH, weightsO, biasesO):
        self.WeightsO = weightsO
        self.BiasesO = biasesO
        self.WeightsH = weightsH
        self.BiasesH = biasesH

    def make_prediction(self, sensors, vr, vl, max_speed):
        # normalization
        sensors = [sensor / 100 for sensor in sensors]
        vr = (vr + max_speed)/(2 * max_speed)
        vl = (vl + max_speed)/(2 * max_speed)

        sensors.extend([vl, vr])

        input_vector = np.asarray(sensors)

        output = self.feed_forward(input_vector)

        # decoding
        vl_output = round(output[0][0] * 2 * max_speed - max_speed)
        vr_output = round(output[0][1] * 2 * max_speed - max_speed)

        return vr_output, vl_output