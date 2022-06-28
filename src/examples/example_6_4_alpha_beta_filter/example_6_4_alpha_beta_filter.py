# OCE4531 Lecture Notes Material - Alpha-Beta Filter Example
# Version: 1.0.0
# Braidan Duffy
# Created: 06/28/2022, Last Edit: 06/28/2022

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
samples = [322, 341, 358, 374, 399, 420, 436, 458, 483, 501] # z_1 -> z_10


# State estimates
x_estimates      = [300] * (len(samples)+1) # Initial guess of 300 m for x_0
xdot_estimates   = [4]   * (len(samples)+1) # Initial guess of 4 m/s for xdot_0
x_predictions    = [nan] * len(samples)     # Empty list of predictions for x
xdot_predictions = [nan] * len(samples)     # Empty list of predictions for xdot

# Calculate the true values based on initial guesses
x_true = [x_estimates[0]] * (len(samples)+1)
for i in range(0,len(samples)):
    x_true[i+1] = x_true[i] + xdot_estimates[0]*delta_t

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
fig, axs = plt.subplots(nrows=2, ncols=1)
fig.suptitle("Example 6.4: Alpha-Beta Filter Results")

# Displacement plot
samples.insert(0, nan) # Shift the sample data over to start at appropriate index (n=1)
axs[0].set_ylabel("Displacement [m]")
l0, = axs[0].plot(x_true, c='k', linestyle='dashed')
l1, = axs[0].plot(samples, c='r')
l2, = axs[0].plot(x_estimates, c='b')

# Velocity plot
axs[1].set_ylabel("Velocity [m/s]")
axs[1].axhline(y=4, c='k', linestyle='dashed')
axs[1].plot(xdot_estimates, color='b')

plt.legend([l0, l1, l2], ["True Value", "Measurements", "Estimates"])
plt.show()

# DEBUG Statements
print(samples)
print(x_estimates)
print(xdot_estimates)
print(x_predictions)
print(xdot_predictions)