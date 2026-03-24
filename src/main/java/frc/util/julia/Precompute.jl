import DifferentialEquations as DE
import Optimization as OPT
# import CurveFit as CF
using OptimizationNLopt
using LinearAlgebra
using Polynomials
using Plots

const L = 0.1501 # Characteristic length (diameter)
const A = pi * (L / 2)^2
const rho = 1.22 # Air density
const v = 1.516e-5 # Kinematic viscosity
const g = 9.80665 # Gravitational acceleration
const m = 0.215002783 # Projectile mass
const air = 0.5 * rho * A
const I = 2/5 * m * (L/2)^2 # Moment of inertia
const p = ( # Function parameters (apparently passing them like this is more conducive to doing parameter estimation)
    S0 = 0.2 * air, # Magnus/lift coefficient
    k = 0.02 # "Torque parameter"
)

const tspan = (0.0, 4.5)
const goalHeight = 1.8288
const goal_dydx = -tand(51)
const untilx = 6.54157776883 + 0.1
const v00 = 6.3
const vincrement = 0.025
# const vincrement = 0.1
const optTime1 = 6
const optTime = 1
const slip = 0.9

function C_d(Re)
    24/Re +
    (2.6*(Re/5.0)) / (1 + (Re/5.0)^1.52) +
    (0.411*(Re/(2.63 * 10^5))^-7.94) / (1 + (Re/(2.63 * 10^5))^-8.0) +
    (0.25*(Re/10^6)) / (1 + (Re/10^6))
end

function ballmotion!(du, u, p, t)
    # https://open.library.ubc.ca/media/stream/pdf/51869/1.0107239/1

    V = u[4:6]
    omega = u[7:9]

    speed = norm(V)
    Re = (speed * L) / v
    C = air * C_d(Re)
    S = (L/2) * norm(omega) / speed
    magnus = cross(omega, V)

    du[1:3] = V
    du[4] = (-C * V[1] * speed + p.S0 * magnus[1]) / m
    du[5] = -g - (C * V[2] * speed + p.S0 * magnus[2]) / m
    du[6] = (-C * V[3] * speed + p.S0 * magnus[3]) / m
    du[7:9] = normalize(omega) * ((-air * speed^2 * S * p.k) / I)
    nothing
end

stopCond(u, t, integrator) = u[5] < 0 ? u[2] - goalHeight : 1.0
affect!(integrator) = DE.terminate!(integrator)
cb = DE.ContinuousCallback(stopCond, affect!)

function intermediate(v0, radians)
    V0y, V0z = sincos(radians) .* v0
    u0 = [0.0; 0.0; 0.0; 0.0; V0y; V0z; (v0/(pi * L)) * slip; 0.0; 0.0]
    prob = DE.ODEProblem(ballmotion!, u0, tspan, p)
    DE.solve(prob, callback = cb)
end

function saveResult!(x, v0, angle, error, plotdata)
    push!(plotdata, (x = x, v0 = v0, angle = angle, error = error))
    @show x, v0, angle, error
    nothing
end

function compute!(v0, plotdata)
    function objective(x, p)
        odesol = intermediate(v0, first(x))
        dydx = odesol.u[end][5] / odesol.u[end][6]
        derivative = abs(dydx - goal_dydx)
        displacement = abs(odesol.u[end][2] - goalHeight)
        derivative + displacement
    end
    optf = OPT.OptimizationFunction(objective)
    prob = OPT.OptimizationProblem(optf, [0.0], lb = [0.0], ub = [pi/2])
    optsol = OPT.solve(prob, NLopt.GN_DIRECT_L(), maxtime = v0 == v00 ? optTime1 : optTime)
    angle = optsol.u[end]
    finalsol = intermediate(v0, angle)
    x = finalsol.u[end][3]
    if x > untilx
        return
    end
    saveResult!(x, v0, angle, optsol.objective, plotdata)
    compute!(trunc(v0 + vincrement, digits=3), plotdata)
    nothing
end

function run()
    plotdata = []
    compute!(v00, plotdata)
    d = getfield.(plotdata, :x)
    v0 = getfield.(plotdata, :v0)
    angle = getfield.(plotdata, :angle)
    # error = filter(y -> y != 0.0, getfield.(plotdata, :error))
    error = getfield.(plotdata, :error)

    #=
    v0prob = CF.CurveFitProblem(d, v0)
    v0fit = CF.solve(v0prob, CF.PolynomialFitAlgorithm(degree = 9))
    angleprob = CF.CurveFitProblem(d, angle)
    anglefit = CF.solve(angleprob, CF.PolynomialFitAlgorithm(degree = 9))
    println("v0: ", CF.coef(v0fit))
    println("angle: ", CF.coef(anglefit))
    =#

    function objective1(x, p)
        pfit = map(coef -> round(coef, digits=trunc(Int, x[1])), fit(d, v0, trunc(Int, x[2])))
        sum(abs.(map(x -> pfit(x), d) .- v0))
    end
    f1 = OPT.OptimizationFunction(objective1)
    prob1 = OPT.OptimizationProblem(f1, [1, 1], lb = [1, 1], ub = [8, 20])
    optsol1 = OPT.solve(prob1, NLopt.GN_DIRECT_L(), maxtime = 3)
    @show v0fit = map(coef -> round(coef, digits=trunc(Int, optsol1[1])), fit(d, v0, trunc(Int, optsol1[2])))

    function objective2(x, p)
        pfit = map(coef -> round(coef, digits=trunc(Int, x[1])), fit(d, angle, trunc(Int, x[2])))
        sum(abs.(map(x -> pfit(x), d) .- angle))
    end
    f2 = OPT.OptimizationFunction(objective2)
    prob2 = OPT.OptimizationProblem(f2, [1, 1], lb = [1, 1], ub = [8, 20])
    optsol2 = OPT.solve(prob2, NLopt.GN_DIRECT_L(), maxtime = 3)
    @show anglefit = map(coef -> round(coef, digits=trunc(Int, optsol2[1])), fit(d, angle, trunc(Int, optsol2[2])))

    default(ms = 2)

    pv0 = scatter(d, v0, ylabel = "v0")
    plot!(d, x -> v0fit(x))
    # pv0r = scatter(d, CF.fitted(v0fit) .- v0)
    pv0r = scatter(d, map(x -> v0fit(x), d) .- v0)

    pangle = scatter(d, angle, ylabel = "rad")
    plot!(d, x -> anglefit(x))
    # pangler = scatter(d, CF.fitted(anglefit) .- angle)
    pangler = scatter(d, map(x -> anglefit(x), d) .- angle)

    pobj = scatter(d, error, ylabel = "obj")

    plot(pv0, pv0r, pangle, pangler, pobj, layout = (5,1), legend = false, size = (600, 650))
end

run()