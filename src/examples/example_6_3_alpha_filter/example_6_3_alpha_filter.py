# OCE4531 Lecture Notes Material - Alpha Filter Example
# Version: 1.0.0
# Braidan Duffy
# Created: 06/28/2022, Last Edit: 06/28/2022

# Imports
import matplotlib.pyplot as plt
from cmath import nan

# Global definitions (assumptions)


# ========================
# === FILTER FUNCTIONS ===
# ========================


# State Update Equations

def estimate_weight(z, x_prev, N):
    """
    Updates the weight based on the state update equation

    Parameters
    ----------
        z: the current weight measurement
        x_prev: the prediction of the current weight from the previous state
        N: the total number of samples so far

    Returns
    -------
        the estimate of the current weight according to the state update equation
    """
    return x_prev + 1/N * (z - x_prev)


# State Extrapolation Equations

def predict_weight(x):
    """
    Predicts the weight at the next iteration based on the state extrapolation equation

    Parameters
    ----------
        x: the estimate of the current weight

    Returns
    -------
        the current weight estimate (assumes no change in weight)
    """
    return x


# ========================
# === MAIN APPLICATION ===
# ========================


# Measure 10 different samples
samples = [10.3, 9.9, 9.8, 10.3, 10.4, 10.1, 10.3, 10.0, 9.8, 9.5] # z_1 -> z_10


# State estimates
x_estimates      = [10] * (len(samples)+1)   # Initial guess of 0 kg for x_0
x_predictions    = [nan] * len(samples)     # Empty list of predictions for x

# Run the measurements through the filter
i = 0 # Index tracker
for z in samples:
    # Predict the next state
    x_predictions[i] = predict_weight(x_estimates[i])

    # Estimate the current state
    x_estimates[i+1] = estimate_weight(z, x_predictions[i], i+1)

    # Increment current state index
    i += 1

# Plot the data
samples.insert(0, nan) # Shift the sample data over to start at appropriate index (n=1)
plt.axhline(y=10, c='k', linestyle='dashed')
plt.plot(samples, c='r')
plt.plot(x_estimates, c='b')

plt.title("Alpha Filter Results")
plt.ylabel("Displacement [m]")
plt.legend(["True Value", "Measurements", "Estimates"])
plt.show()

# DEBUG Statements
print(samples)
print(x_estimates)
print(x_predictions)
