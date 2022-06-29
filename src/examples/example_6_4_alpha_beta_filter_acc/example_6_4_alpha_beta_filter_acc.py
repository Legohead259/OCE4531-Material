# OCE4531 Lecture Notes Material - Alpha-Beta Filter Example with Acceleration
# Version: 1.0.0
# Braidan Duffy
# Created: 06/29/2022, Last Edit: 06/29/2022

# Imports
import matplotlib.pyplot as plt
from cmath import nan

# Global definitions (assumptions)
alpha_gain  = 0.2 # Position gain
beta_gain   = 0.1 # Velocity gain
delta_t     = 5   # seconds


# ========================
# === FILTER FUNCTIONS ===
# ========================


# State Update Equations

def _estimate_position(z, x_prev):
    """
    Updates the position based on the state update equation for position

    Parameters
    ----------
        z: the current position measurement
        x_prev: the prediction of the current position from the previous state

    Returns
    -------
        the estimate of the current position according to the state update equation
    """
    return x_prev + alpha_gain * (z - x_prev)


def _estimate_velocity(z, x_prev, xdot_prev):
    """
    Updates the velocity based on the state update equation for velocity

    Parameters
    ----------
        z: the current position measurement
        x_prev: the prediction of the current position from the previous iteration
        xdot_prev: the prediction of the current velocity from the previous iteration

    Returns
    -------
        the estimate of the current velocity according to the state update equation
    """
    return xdot_prev + beta_gain * ((z - x_prev) / delta_t)


def estimate_current_state(z, x_prev, xdot_prev):
    """
    Wrapper that provides a simultaneous call and return for estimating the current state

    Parameters
    ----------
        z: the current position measurement
        x_prev: the prediction of the current position from the previous iteration
        xdot_prev: the prediction of the current velocity from the previous iteration

    Returns
    -------
        a tuple containing the estimated position (0) and the estimated velocity (1)
    """
    return (_estimate_position(z, x_prev), _estimate_velocity(z, x_prev, xdot_prev))


# State Extrapolation Equations

def _predict_position(x, xdot):
    """
    Predicts the position at the next iteration based on the state extrapolation equation

    Parameters
    ----------
        x: the estimate of the current position
        xdot: the estimate of the current velocity

    Returns
    -------
        the predicted position at the next iteration based on the state extrapolation equation
    """
    return x + xdot * delta_t


def _predict_velocity(xdot):
    """
    Predicts the velocity at the next iteration based on the state extrapolation equation

    Parameters
    ----------
        xdot: the estimate of the current velocity

    Returns
    -------
        the current velocity estimate (assumes no change in velocity)
    """
    return xdot


def predict_next_state(x, xdot):
    """
    Wrapper that provides a simultaneous call and return for predicting the next state

    Parameters
    ----------
        x: the estimate of the current position
        xdot: the estimate of the current velocity

    Returns
    -------
        a tuple containing the predicted position (0) and the predicted velocity (1)
    """
    return (_predict_position(x, xdot), _predict_velocity(xdot))


# ========================
# === MAIN APPLICATION ===
# ========================


# Measure 10 different samples
samples = [60, 77, 107, 154, 252, 407] # z_1 -> z_6
times = [0, 5, 10, 15, 20, 25, 30]
# print(times) # DEBUG

# State estimates
x_estimates      = [30] * (len(samples)+1) # Initial guess of 30 m for x_0
xdot_estimates   = [5]   * (len(samples)+1) # Initial guess of 5 m/s for xdot_0
x_predictions    = [nan] * len(samples)     # Empty list of predictions for x
xdot_predictions = [nan] * len(samples)     # Empty list of predictions for xdot

# Calculate the true values based on known model
#   A vessel is transiting at a constant velocity of 5 m/s for 15 seconds,
#   then accelerates at a constant 2 m/s/s for 15 seconds

# Displacement
x_true = []
for t in range(0,15, delta_t): # No acceleration phase
    x_true.append(30+5*t)

for t in range(0,15+delta_t, delta_t): # Acceleration phase
        x_true.append(105 + 5*t + t*t)
# print(x_true) # DEBUG

# Velocity
xdot_true = [5]*3
for t in range(0,15+delta_t, delta_t):
    xdot_true.append(5 + 2*t)
# print(xdot_true) # DEBUG

# Run the measurements through the filter
i = 0 # Index tracker
for z in samples:
    # Predict the next state
    next_state = predict_next_state(x_estimates[i], xdot_estimates[i])
    x_predictions[i] = next_state[0]
    xdot_predictions[i] = next_state[1]

    # Estimate the current state
    current_state = estimate_current_state(z, x_predictions[i], xdot_predictions[i])
    x_estimates[i+1] = current_state[0]
    xdot_estimates[i+1] = current_state[1]

    # Increment current state index
    i += 1

# Plot the data
fig, axs = plt.subplots(nrows=3, ncols=1)
fig.suptitle("Alpha-Beta Filter Results with Acceleration")

# Displacement plot
samples.insert(0, nan) # Shift the sample data over to start at appropriate index (n=1)
axs[0].set_ylabel("Displacement [m]")
l0, = axs[0].plot(times, x_true, c='k', linestyle='dashed')
l1, = axs[0].plot(times, samples, c='r')
l2, = axs[0].plot(times, x_estimates, c='b')

# Velocity plot
axs[1].set_ylabel("Velocity [m/s]")
axs[1].plot(times, xdot_true, c='k', linestyle='dashed')
axs[1].plot(times, xdot_estimates, color='b')

# Acceleration plot
axs[2].set_ylabel("Acceleration [m/s/s]")
axs[2].axhline(y=0, xmin=0, xmax=0.5, c='k', linestyle='dashed')
axs[2].axhline(y=2, xmin=0.5, xmax=1, c='k', linestyle='dashed')
axs[2].set_ylim([-0.1,2.1])
axs[2].set_xlim([0,30])

plt.legend([l0, l1, l2], ["True Value", "Measurements", "Estimates"])
plt.show()

# # DEBUG Statements
# print(samples)
# print(x_estimates)
# print(xdot_estimates)
# print(x_predictions)
# print(xdot_predictions)