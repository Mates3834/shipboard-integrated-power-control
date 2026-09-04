def supervise_loads(total_demand, available_power):
    """
    Generic priority-based shedding.
    Lowest-priority loads are reduced first.
    """
    priorities = [
        ("critical", 0.18),
        ("propulsion", 0.50),
        ("navigation", 0.08),
        ("auxiliary", 0.14),
        ("hotel", 0.10),
    ]

    requested = {name: total_demand * frac for name, frac in priorities}
    served = {}
    remaining = max(0.0, float(available_power))

    for name, _ in priorities:
        take = min(requested[name], remaining)
        served[name] = take
        remaining -= take

    return served, sum(served.values())
