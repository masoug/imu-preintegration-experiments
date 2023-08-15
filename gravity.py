import math

import gtsam
from gtsam.symbol_shorthand import P, V, B
import numpy as np

# setup preintegrator & factor
params = gtsam.PreintegrationParams.MakeSharedU(9.81)
pim = gtsam.PreintegratedImuMeasurements(params)

# simulate motion for 1 second
bias = gtsam.imuBias.ConstantBias(np.array([0.0, 0.0, 0.0]),
                                  np.array([0.0, 0.0, 0.0]))
pim.integrateMeasurement(np.array([0.0, 0.0, 9.81]),
                         np.array([0.0, 0.0, 0.0]),
                         1.0)
factor = gtsam.ImuFactor(P(0), V(0), P(1), V(1), B(0), pim)

# initial condition level
init_state_level = gtsam.NavState()
print(f'init state (level):\n{init_state_level}\n')
predicted_state_level = pim.predict(init_state_level, bias)
print(f'predicted state (level):\n{predicted_state_level}\n')
error = factor.evaluateError(init_state_level.pose(), init_state_level.velocity(),
                             predicted_state_level.pose(), predicted_state_level.velocity(),
                             bias)
print(f'factor error: {error}')

print()

# initial condition is pitched up
init_state_pitched = gtsam.NavState(gtsam.Rot3.Ry(-math.pi/2.0),
                            np.array([0.0, 0.0, 0.0]),
                            np.array([0.0, 0.0, 0.0]))
print(f'init state (pitched):\n{init_state_pitched}\n')
predicted_state_pitched = pim.predict(init_state_pitched, bias)
print(f'predicted state (pitched):\n{predicted_state_pitched}\n')
error = factor.evaluateError(init_state_pitched.pose(), init_state_pitched.velocity(),
                             predicted_state_pitched.pose(), predicted_state_pitched.velocity(),
                             bias)
print(f'factor error: {error}')