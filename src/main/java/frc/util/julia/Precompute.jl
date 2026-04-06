import DifferentialEquations as DE
import SimpleNonlinearSolve as NLS
using LinearAlgebra
using Plots
using SymbolicRegression
using DynamicQuantities: m, s, rad

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

const tspan = (0.0, 4.5) # if not terminated
const uspan = (0.0, pi/2)
const goal_dydx = -tand(51)
const untild = 6.54157776883
const untily = 1.8288
const v00 = 6.3
const increment = 0.05
# const increment = 0.1
const slip = 1
const abstol = 5e-6

function C_d(Re)
    # Calculates the drag coefficient from Reynolds number
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
    S = L/2 * norm(omega) / speed
    magnus = cross(omega, V)

    du[1:3] = V
    du[4] = (-C * V[1] * speed + p.S0 * magnus[1]) / m
    du[5] = -g - (C * V[2] * speed + p.S0 * magnus[2]) / m
    du[6] = (-C * V[3] * speed + p.S0 * magnus[3]) / m
    du[7:9] = normalize(omega) * ((-air * speed^2 * S * p.k) / I)
    nothing
end

function ode(v0, radians, goalHeight)
    V0y, V0z = sincos(radians) .* v0
    u0 = [0.0; 0.0; 0.0; 0.0; V0y; V0z; (v0/(pi * L)) * slip; 0.0; 0.0]
    prob = DE.ODEProblem(ballmotion!, u0, tspan, p)
    stopCond(u, t, integrator) = u[5] < 0 ? u[2] - goalHeight : 1.0
    affect!(integrator) = DE.terminate!(integrator)
    cb = DE.ContinuousCallback(stopCond, affect!)
    DE.solve(prob, callback=cb)
end

function objective(angle, p)
    odesol = ode(p.v0, angle, p.goalHeight)
    dydx = odesol.u[end][5] / odesol.u[end][6]
    derivative = dydx - goal_dydx
    # displacement = odesol.u[end][2] - p.goalHeight
    derivative
end

function point(v0, goalHeight)
    p = (v0=v0, goalHeight=goalHeight)
    prob = NLS.IntervalNonlinearProblem(objective, uspan, p)
    sol = NLS.solve(prob)
    angle = sol.u
    finalsol = ode(v0, angle, goalHeight)
    d = finalsol.u[end][3]
    (d=d, angle=angle, obj=sol.resid)
end

function regression(data)
    ys = QuantityArray(data.ys, m)
    ds = QuantityArray(data.ds, m)
    v0s = QuantityArray(data.v0s, m/s)
    angles = QuantityArray(data.angles, rad)
    
end

function run()
    ys = []
    ds = []
    v0s = []
    angles = []
    objs = []
    for goalHeight in 0.0:increment:untily
        v0 = v00
        # while (print("height=", goalHeight, ", v0=", v0, "; "); (_point = @show point(v0, goalHeight)).d < untild)
        while (_point = point(v0, goalHeight)).d < untild
            push!(ys, goalHeight)
            push!(ds, _point.d)
            push!(v0s, v0)
            push!(angles, _point.angle)
            push!(objs, _point.obj)
            v0 += increment
        end
    end

    #=
    roundedpolyfit(vars, ydata) = map(coef -> round(coef, digits=trunc(Int, vars[1])), fit(d, ydata, trunc(Int, vars[2])))

    function regressionobj(x, p)
        pfit = roundedpolyfit(x, p)
        sum(abs.(map(_x -> pfit(_x), d) .- p))
    end
    optf = OPT.OptimizationFunction(regressionobj)

    v0prob = OPT.OptimizationProblem(optf, [1, 1], lb=[1, 1], ub=[8, 20], p=v0)
    v0sol = OPT.solve(v0prob, NLopt.GN_DIRECT_L(), maxtime = 3)
    @show v0fit = roundedpolyfit(v0sol, v0)

    angleprob = OPT.OptimizationProblem(optf, [1, 1], lb=[1, 1], ub=[8, 20], p=angle)
    anglesol = OPT.solve(angleprob, NLopt.GN_DIRECT_L(), maxtime=3)
    @show anglefit = roundedpolyfit(anglesol, angle)
    =#

    plotlyjs()
    default(ms=2)

    pv0 = scatter(ds, v0s, ys, ylabel="v0")
    # plot!(d, x -> v0fit(x))
    # pv0r = scatter(d, CF.fitted(v0fit) .- v0)
    # pv0r = scatter(d, map(x -> v0fit(x), d) .- v0)

    pangle = scatter(ds, angles, ys, ylabel="rad")
    # plot!(d, x -> anglefit(x))
    # pangler = scatter(d, CF.fitted(anglefit) .- angle)
    # pangler = scatter(d, map(x -> anglefit(x), d) .- angle)

    pobj = scatter(ds, objs, ys, ylabel="obj")

    plot(pv0, pangle, pobj, layout=(1,3), legend=false, size=(1200, 650))
end

run()