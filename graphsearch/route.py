import random

import numpy as np

PRIORITY_LEVELS = ["Urgent", "High", "Normal", "Low"]


class Route:
    def __init__(self, origin=None, destination=None, priority_level="Normal", value=0.0, distance=np.inf):
        self.origin = origin
        self.destination = destination
        self.priority_level = priority_level
        self.value = value
        self.distance = distance
        self.path = None

    def serialize(self):
        return {
            "origin": self.origin.id,
            "destination": self.destination.id,
            "priority_level": self.priority_level,
            "value": self.value,
        }


def route_prioritization(env, prioritization, threshold_v, threshold_d, reproducible=True, print_output=True):
    '''
    
    Route prioritization
    
    '''
    if reproducible:
        random.seed(0)
        np.random.seed(0)

    if print_output:
        print("\nOriginal routes:")
        for r in env.routes:
            print(f"{r.origin.id}\t{r.destination.id}\t{r.priority_level}\t{r.value:8.3f}\t{r.distance:8.3f}")

    if not prioritization:
        return

    results = []
    # sort by priority level
    for pl in PRIORITY_LEVELS:
        subset = [r for r in env.routes if r.priority_level == pl]
        # sort by value
        subset.sort(key=lambda r: r.value, reverse=True)
        i = 0
        while i < len(subset):
            j = i + 1
            while j < len(subset) and abs(subset[i].value - subset[j].value) < threshold_v:
                j += 1
            if j > i + 1:
                # for the subset indistinguishable by value
                subsubset = subset[i:j]

                # sort by distance
                subsubset.sort(key=lambda r: r.distance)
                m = 0
                while m < len(subsubset):
                    n = m + 1
                    while n < len(subsubset) and abs(subsubset[m].distance - subsubset[n].distance) < threshold_d:
                        n += 1
                    if n > m + 1:
                        # for the subset indistinguishable by distance
                        subsubsubset = subsubset[m:n]
                        random.shuffle(subsubsubset)
                        subsubset[m:n] = subsubsubset

                        m = n
                    else:
                        m += 1
                subset[i:j] = subsubset
                i = j
            else:
                i += 1
        results += subset

    if print_output:
        print("\nPrioritized routes:")
        for r in results:
            print(f"{r.origin.id}\t{r.destination.id}\t{r.priority_level}\t{r.value:8.3f}\t{r.distance:8.3f}")

    env.routes = results


def generate_routes(normal_airports, reproducible=True):
    '''
    
    Generate OD pairs
    
    '''
    if reproducible:
        random.seed(0)
        np.random.seed(0)

    # randomly generate path OD pairs
    routes = []
    for i in range(len(normal_airports)):
        for j in range(i + 1, len(normal_airports)):
            routes.append(
                Route(
                    normal_airports[i],
                    normal_airports[j],
                    PRIORITY_LEVELS[np.random.randint(0, len(PRIORITY_LEVELS))],
                    np.random.rand() * 11000 - 1000
                )
            )

    routes = random.sample(routes, int(0.6*len(routes)))
    return routes
