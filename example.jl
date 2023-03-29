using OrbitalEstimationPlayground

function estimator(position, acceleration, angular_velocity, magnetometer, t)
    return position
end

OrbitalEstimationPlayground.measure_error(estimator, OrbitalEstimationPlayground.default_noise)