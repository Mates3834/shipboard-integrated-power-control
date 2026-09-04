from src.simulation import run
from src.evaluation import metrics

for scenario in ("cruise","acceleration","heavy_sea","berthing","load_shed_event"):
    print(f"\nScenario: {scenario}")
    for method in ("pid","lqr","mpc"):
        result = run(method=method, scenario=scenario)
        print(method.upper(), metrics(result))
