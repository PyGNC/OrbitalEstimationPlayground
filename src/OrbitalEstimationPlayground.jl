module OrbitalEstimationPlayground

using GNCTestServer
using Statistics
using Distributions
using LinearAlgebra

function measure_error(user_estimator, error_model)
    errors = []
    function control_law(state, params, t)
        estimate = user_estimator(error_model(state, params, t)..., t)
        err = state.position - estimate
        push!(errors, err)
        return zero(GNCTestServer.Control)
    end
    GNCTestServer.simulate(control_law, max_iterations=14400) # two hours
    return mean(errors), std(errors)
end

std_acceleration_noise = MvNormal(zeros(3), I(3))
std_GPS_noise = MvNormal(zeros(3), I(3) * 10000.0)
std_gyro_noise = MvNormal(zeros(3), I(3) * 0.1)
std_magnetometer_noise = MvNormal(zeros(3), I(3) * 0.1)
function default_noise(state, params, t)
    a = GNCTestServer.dynamics(state, params, zero(GNCTestServer.Control), t).velocity
    a += rand(std_acceleration_noise)

    r = state.position
    r += rand(std_GPS_noise)

    ω = state.angular_velocity
    ω += rand(std_gyro_noise)

    b = params.b
    b += rand(std_magnetometer_noise)

    return r, a, ω, b
end
# should output acceleration, position, gyro, magnetometer


end # module OrbitalEstimationPlayground
