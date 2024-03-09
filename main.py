import math
import random

### INSTANCE CONFIGURATION
# Example 1 (wide time windows):   30, 5, 30, 2-5, 100, 80-90, 2-4
# Example 2 (narrow time windows): 30, 5, 30, 2-5, 100, 10-15, 2-4
numCustomers = 14
maxNumVehicles = 5

vehicleCapacity = 30
demandRange = (2, 5)

timeHorizon = 100
timeWindowWidthRange = (10, 15)
serviceTimeRange = (2, 4)



random.seed(0)

depot = 0
customers = [*range(1, numCustomers + 1)]
locations = [depot] + customers
connections = [(i, j) for i in locations for j in locations if i != j]
vehicles = [*range(1, maxNumVehicles + 1)]

# create random depot and customer locations in the Euclidian plane (1000x1000)
points = [(random.uniform(-100, 100), random.uniform(-100, 100)) for i in locations]
# points = [(0, 0), (-6.6, -2.9), (-4.4, 4.5), (4.0, -10.6), (25.5, -48.2), (20.0, -51.1), (33.4, -54.9), (49.5, -46.5), (27.5, -57.2), (77.8, -8.1), (75.4, -16.7), (77.7, -16.6), (93.9, -5.9), (94.6, -14.7), (71.6, -9.2)]
# dictionary of Euclidean distance for each connection (interpreted as travel costs)
costs = {
    (i, j): math.ceil(
        math.sqrt(sum((points[i][k] - points[j][k]) ** 2 for k in range(2)))
    )
    for (i, j) in connections
}
maximalCosts = math.ceil(999 * math.sqrt(2))
# dictionary of travel times for each connection (related to the costs, scaled to time horizon)
travelTimes = {
    (i, j): math.ceil((costs[i, j] / maximalCosts) * timeHorizon * 0.2)
    for (i, j) in connections
}

# create random demands, service times, and time window widths in the given range
demands = {i: random.randint(demandRange[0], demandRange[1]) for i in customers}
demands[0] = 0  # depot has no demand
serviceTimes = {
    i: random.randint(serviceTimeRange[0], serviceTimeRange[1]) for i in customers
}
serviceTimes[0] = 0  # depot has no service time
timeWindowWidths = {
    i: random.randint(timeWindowWidthRange[0], timeWindowWidthRange[1])
    for i in customers
}
# vehicles are allowed to leave the depot any time within the time horizon
timeWindowWidths[0] = timeHorizon

# create time windows randomly based on the previously generated information
# such that the service at a customer can be finished within the time horizon
timeWindows = {}
timeWindows[0] = (0, 0 + timeWindowWidths[0])
for i in customers:
    start = random.randint(0, timeHorizon - serviceTimes[i] - timeWindowWidths[i] - travelTimes[i,0])
    timeWindows[i] = (start, start + timeWindowWidths[i])


import gurobipy as gp
from gurobipy import GRB


def numVehiclesNeededForCustomers(customers):
    sumDemand = 0
    for i in customers:
        sumDemand += demands[i]
    return math.ceil(sumDemand / vehicleCapacity)


# create model for Capacitated Vehicle Routing Problem instance
model = gp.Model("VRPTW")

# binary variables x(i,j): is 1 if some vehicle is going from node i to node j, 0 otherwise
x = model.addVars(connections, vtype=GRB.BINARY, name="x")

# objective function: minimize sum of connection costs
model.setObjective(x.prod(costs), GRB.MINIMIZE)

# all customers have exactly one incoming and one outgoing connection
model.addConstrs((x.sum("*", j) == 1 for j in customers), name="incoming")
model.addConstrs((x.sum(i, "*") == 1 for i in customers), name="outgoing")

# vehicle limits
model.addConstr(x.sum(0, "*") <= maxNumVehicles, name="maxNumVehicles")
model.addConstr(
    x.sum(0, "*") >= numVehiclesNeededForCustomers(customers),
    name="minNumVehicles",
)

### MODEL CONFIGURATION
loadModelType = 1  # 1: big-M, 2: flow
timeModelType = 1  # 1: big-M, 2: flow
###


def addLoadConstraintsByBigM():

    y = model.addVars(locations, lb=0, ub=vehicleCapacity, name="y")
    y[0].UB = 0  # empty load at depot

    model.addConstrs(
        (
            y[i] + demands[j] <= y[j] + vehicleCapacity * (1 - x[i, j])
            for i in locations
            for j in customers
            if i != j
        ),
        name="loadBigM1",
    )
    model.addConstrs(
        (
            y[i] + demands[j]
            >= y[j] - (vehicleCapacity - demands[i] - demands[j]) * (1 - x[i, j])
            for i in locations
            for j in customers
            if i != j
        ),
        name="loadBigM2",
    )


def addLoadConstraintsByFlows():

    z = model.addVars(connections, lb=0, ub=vehicleCapacity, name="z")

    for i in customers:
        z[0, i].UB = 0

    model.addConstrs(
        (z.sum("*", j) + demands[j] == z.sum(j, "*") for j in customers),
        name="flowConservation",
    )
    model.addConstrs(
        (
            z[i, j] >= demands[i] * x[i, j]
            for i in customers
            for j in locations
            if i != j
        ),
        name="loadLowerBound",
    )
    model.addConstrs(
        (
            z[i, j] <= (vehicleCapacity - demands[j]) * x[i, j]
            for i in customers
            for j in locations
            if i != j
        ),
        name="loadUpperBound",
    )


def addTimeConstraintsByBigM():

    y = model.addVars(locations, name="y")
    for i in locations:
        y[i].LB = timeWindows[i][0]
        y[i].UB = timeWindows[i][1]

    model.addConstrs(
        (
            y[i] + serviceTimes[i] + travelTimes[i, j]
            <= y[j]
            + (
                timeWindows[i][1]
                + serviceTimes[i]
                + travelTimes[i, j]
                - timeWindows[j][0]
            )
            * (1 - x[i, j])
            for i in locations
            for j in customers
            if i != j
        ),
        name="timeBigM",
    )


def addTimeConstraintsByFlows():

    z = model.addVars(connections, lb=0, name="z")

    for (i, j) in connections:
        z[i, j].UB = timeWindows[i][1]

    model.addConstrs(
        (
            gp.quicksum(
                z[i, j] + (serviceTimes[i] + travelTimes[i, j]) * x[i, j]
                for i in locations
                if (i, j) in connections
            )
            <= z.sum(j, "*")
            for j in customers
        ),
        name="flowConservation",
    )
    model.addConstrs(
        (
            z[i, j] >= timeWindows[i][0] * x[i, j]
            for i in customers
            for j in locations
            if i != j
        ),
        name="timeWindowStart",
    )
    model.addConstrs(
        (
            z[i, j] <= timeWindows[i][1] * x[i, j]
            for i in customers
            for j in locations
            if i != j
        ),
        name="timeWindowEnd",
    )


if loadModelType == 1:
    addLoadConstraintsByBigM()
elif loadModelType == 2:
    addLoadConstraintsByFlows()

if timeModelType == 1:
    addTimeConstraintsByBigM()
elif timeModelType == 2:
    addTimeConstraintsByFlows()


model.params.Threads = 4
model.optimize()

if model.SolCount >= 1:

    usedConnections = [(i, j) for (i, j) in x.keys() if x[i, j].X > 0.5]

    # create a dict for the next customer based on the current one
    # (note that the depot in general has multiple outgoing connections)
    nextCustomer = {}
    for (i, j) in usedConnections:
        if i == 0:
            if 0 not in nextCustomer.keys():
                nextCustomer[0] = []
            nextCustomer[0].append(j)
        else:
            nextCustomer[i] = j

    print(f"Solution contains {len(nextCustomer[0])} routes:")
    routeNumber = 0
    visitedCustomers = [False] * (numCustomers + 1)
    for firstCustomer in nextCustomer[0]:
        print(f"Route {routeNumber}: 0 -> ", end="")
        vehicleLoad = 0
        time = travelTimes[0, firstCustomer]
        violatedTimeWindows = False
        currentCustomer = firstCustomer
        while currentCustomer != 0:
            print(f"{currentCustomer} (L:{vehicleLoad}, T:{time}) -> ", end="")
            visitedCustomers[currentCustomer] = True
            vehicleLoad += demands[currentCustomer]
            time = max(time, timeWindows[currentCustomer][0])
            if time > timeWindows[currentCustomer][1]:
                violatedTimeWindows = True
            time += (
                serviceTimes[currentCustomer]
                + travelTimes[currentCustomer, nextCustomer[currentCustomer]]
            )
            currentCustomer = nextCustomer[currentCustomer]
        print(f"0 (L:{vehicleLoad}/{vehicleCapacity}, T:{time})")
        if vehicleLoad > vehicleCapacity:
            print("Vehicle capacity is exceeded!")
        if violatedTimeWindows:
            print("Time windows are violated!")
        routeNumber += 1

    print("Unvisited customers: ", end="")
    for c in customers:
        if visitedCustomers[c] == False:
            print(f"{c}, ", end="")

if model.Status == GRB.INFEASIBLE:
    model.computeIIS()
    model.write("iis.ilp")

# free resources
model.dispose()
gp.disposeDefaultEnv()