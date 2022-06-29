# OCE4531 Lecture Notes Material - Simple ID Kalman Filter Example
# Version: 1.0.0
# Braidan Duffy
# Created: 06/29/2022, Last Edit: 06/29/2022
#
# In this example, we will be estimating the depth of an ROV using a 1D Kalman filter.
# We will be assuming the ROV depth remains constant and its location in space does not matter or change.
# For example's sake, we know for an absolute fact that the ROV is at a depth of 50-meters.
# We will also assume that our depth sensor has a standard deviation of 5-meters, therefore we will have a measurement variance or uncertainty of 25-meters
 

from cProfile import label
from cmath import nan
from KalmanFilters import KalmanFilter1D
import matplotlib.pyplot as plt

# Global Values (assumptions)
x_true = 50 # m
r = 25 # m2

# ========================
# === MAIN APPLICATION ===
# ========================


# Measure 10 different samples
samples = [48.5, 47.1, 55.0, 55.2, 49.9, 40.6, 46.7, 50.1, 51.3, 50] # z_1 -> z_10

# List instantiation
x_estimates      = [60] # Initial guess of 60 m for x_0
p_estimates      = [36]   # Empty list of estimate uncertainties
x_predictions    = []   # Empty list of predictions for x
p_predictions    = []   # Empty list of prediction uncertainties

# Kalman filter instantiation
kf = KalmanFilter1D(60, 225, 0.01)

# Run the measurements through the filter
for z in samples:
    # Estimate the current state
    current_state = kf.estimate_current_state(z, r)
    x_estimates.append(current_state[0])
    p_estimates.append(current_state[1])

    # Predict the next state
    next_state = kf.predict_next_state()
    x_predictions.append(next_state[0])
    p_predictions.append(next_state[1])

# Plot the data
samples.insert(0, nan) # Shift the sample data over to start at appropriate index (n=1)
l0 = plt.axhline(y=50, c='k', linestyle='dashed', label="True Value")
l1 = plt.plot(samples, c='r', label="Measurements")
# l2 = plt.plot(x_estimates, c='b')
l2 = plt.errorbar([0,1,2,3,4,5,6,7,8,9,10], x_estimates, yerr=p_estimates)


plt.title("1D Kalman Filter Results")
plt.xlabel("Iterations (n)")
plt.ylabel("Depth [m]")
plt.legend(["True Value", "Measurements", "Estimates"])
plt.show()

# DEBUG Statements
print(samples)
print(x_estimates)
print(p_estimates)
print(x_predictions)
print(p_predictions)
