import math

import gtsam
import numpy as np

# setup preintegrator
params = gtsam.PreintegrationParams.MakeSharedU(9.81)
pim = gtsam.PreintegratedImuMeasurements(params)

# simulate motion for 1 second
bias = gtsam.imuBias.ConstantBias(np.array([0.0, 0.0, 0.0]),
                                  np.array([0.0, 0.0, 0.0]))
pim.integrateMeasurement(np.array([0.0, 0.0, 9.81]),
                         np.array([0.0, 0.0, 0.0]),
                         1.0)
init_state = gtsam.NavState()
print(f'init state:\n{init_state}\n')
predicted_state = pim.predict(init_state, bias)
print(f'predicted state:\n{predicted_state}\n')

# initial condition is pitched up
init_state = gtsam.NavState(gtsam.Rot3.Ry(-math.pi/2.0),
                            np.array([0.0, 0.0, 0.0]),
                            np.array([0.0, 0.0, 0.0]))
print(f'init state:\n{init_state}\n')
predicted_state = pim.predict(init_state, bias)
print(f'predicted state:\n{predicted_state}\n')
