import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import inv
import math 

class KalmanFilter():
    """
    Implementation of a Kalman Filter.
    """
    def __init__(self, mu, sigma, A, C, R, Q):
        """
        :param mu: prior mean
        :param sigma: prior covariance
        :param A: process model
        :param C: measurement model
        :param R: process noise
        :param Q: measurement noise
        """
        # prior
        self.mu = mu
        self.sigma = sigma
        self.mu_init = mu
        self.sigma_init = sigma
        # process model
        self.A = A
        self.R = R
        # measurement model
        self.C = C
        self.Q = Q

    def reset(self):
        """
        Reset belief state to initial value.
        """
        self.mu = self.mu_init
        self.sigma = self.sigma_init

    def run(self, sensor_data):
        """
        Run the Kalman Filter using the given sensor updates.

        :param sensor_data: array of T sensor updates as a TxS array.

        :returns: A tuple of predicted means (as a TxD array) and predicted
                  covariances (as a TxDxD array) representing the KF's belief
                  state AFTER each update/predict cycle, over T timesteps.
        """

        t = np.arange(0,10,0.1)
        predicted_mean = np.zeros(shape=(t.shape[0],4))
        predicted_sigma = np.zeros(shape=(t.shape[0],4,4)) 
        predicted_mean[0] = self.mu
        predicted_sigma[0] = self.sigma 
        for i in range(1,t.shape[0]):
            z = sensor_data[i]
            self._predict()
            self._update(z)
            predicted_mean[i] = self.mu
            predicted_sigma[i] = self.sigma

        return predicted_mean, predicted_sigma


    def _predict(self):

        self.mu = np.dot(self.A, self.mu)
        self.sigma = np.dot(np.dot(self.A, self.sigma), np.transpose(self.A)) + self.R


    def _update(self, z):

        y = z - np.dot(self.C, self.mu)
        S = np.dot(np.dot(self.C, self.sigma), np.transpose(self.C)) + self.Q
        K = np.dot(np.dot(self.sigma, np.transpose(self.C)), inv(S))
        self.mu = self.mu + np.dot(K, y)
        self.sigma = np.dot((np.identity(4) - np.dot(K, self.C)), self.sigma)



def plot_prediction(t, ground_truth, measurement, predict_mean, predict_cov):
    """
    Plot ground truth vs. predicted value.

    :param t: 1-dimensional array representing timesteps, in seconds.
    :param ground_truth: Tx1 array of ground truth values
    :param measurement: Tx1 array of sensor values
    :param predict_mean: TxD array of mean vectors
    :param predict_cov: TxDxD array of covariance matrices
    """
    predict_pos_mean = predict_mean[:, 0]
    predict_pos_std = predict_cov[:, 0, 0]

    plt.figure()
    plt.plot(t, ground_truth, color='k')
    plt.plot(t, measurement, color='r')
    plt.plot(t, predict_pos_mean, color='g')
    plt.fill_between(t,predict_pos_mean-predict_pos_std,predict_pos_mean+predict_pos_std,color='g',alpha=0.5)
    plt.legend(("ground truth", "measurements", "predictions"))
    plt.xlabel("time (s)")
    plt.ylabel("position (m)")
    plt.title("Predicted Values")
    plt.show()


def plot_mse(t, ground_truth, predict_means):
    """
    Plot MSE of your KF over many trials.

    :param t: 1-dimensional array representing timesteps, in seconds.
    :param ground_truth: Tx1 array of ground truth values
    :param predict_means: NxTxD array of T mean vectors over N trials
    """
    predict_pos_means = predict_means[:, :, 0]
    errors = ground_truth.squeeze() - predict_pos_means
    mse = np.mean(errors, axis=0) ** 2

    plt.figure()
    plt.plot(t, mse)
    plt.xlabel("time (s)")
    plt.ylabel("position MSE (m^2)")
    plt.title("Prediction Mean-Squared Error")
    plt.show()


def problem2a():
    t = np.arange(0,10,0.1)
    A = np.array([[1,0.1,0,0],[0,1,0.1,0],[0,0,1,0.1],[0,0,0,1]])
    C = np.array([[1,0,0,0]])
    mu = np.array([5,1,0,0])
    sigma = np.array([[10,0,0,0], [0,10,0,0], [0,0,10,0], [0,0,0,10]])
    Q = 1.0
    R = 0.0
    kf_2a = KalmanFilter(mu,sigma,A,C,R,Q)

    sensor_data = np.zeros(shape=(t.shape[0],1))
    true_value = np.zeros(shape=(t.shape[0],1))
    for i in range(t.shape[0]):
        sensor_data[i][0] = math.sin(0.1*t[i]) + np.random.normal(0, Q)
        true_value[i][0] = math.sin(0.1*t[i])

    predict_mean, predict_cov = kf_2a.run(sensor_data) 
    # print(t.shape, true_value.shape, sensor_data.shape, predict_mean.shape, predict_cov.shape)
    plot_prediction(t, true_value, sensor_data, predict_mean, predict_cov) 

    N = 10000
    predict_means = np.zeros(shape=(N, t.shape[0],4))
    for i in range(N):
        predict_means[i] = kf_2a.run(sensor_data)[0]
        kf_2a.reset()
    plot_mse(t, true_value, predict_means)


def problem2b():
    t = np.arange(0,10,0.1)
    A = np.array([[1,0.1,0,0],[0,1,0.1,0],[0,0,1,0.1],[0,0,0,1]])
    C = np.array([[1,0,0,0]])
    mu = np.array([5,1,0,0])
    sigma = np.array([[10,0,0,0], [0,10,0,0], [0,0,10,0], [0,0,0,10]])
    Q = 1.0
    R = np.array([[0.1,0,0,0],[0,0.1,0,0],[0,0,0.1,0],[0,0,0,0.1]])
    kf_2b = KalmanFilter(mu,sigma,A,C,R,Q)

    sensor_data = np.zeros(shape=(t.shape[0],1))
    true_value = np.zeros(shape=(t.shape[0],1))
    for i in range(t.shape[0]):
        sensor_data[i][0] = math.sin(0.1*t[i]) + np.random.normal(0, Q)
        true_value[i][0] = math.sin(0.1*t[i])

    predict_mean, predict_cov = kf_2b.run(sensor_data) 
    # print(t.shape, true_value.shape, sensor_data.shape, predict_mean.shape, predict_cov.shape)
    plot_prediction(t, true_value, sensor_data, predict_mean, predict_cov) 

    N = 10000
    predict_means = np.zeros(shape=(N, t.shape[0],4))
    for i in range(N):
        predict_means[i] = kf_2b.run(sensor_data)[0]
        kf_2b.reset()
        
    plot_mse(t, true_value, predict_means)

    

if __name__ == '__main__':
    problem2a()
    problem2b()
