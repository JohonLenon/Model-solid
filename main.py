import os
import sys
from abaqus import session
from abaqusConstants import * # type: ignore
import importlib
from pathlib import Path

sys.path.append(os.getcwd())
sys.stdout = sys.__stdout__

print("\n")

def reload_modules(exclude=("main",)):
    """
    Recarrega automaticamente todos os módulos .py da pasta atual,
    exceto os listados em 'exclude'.
    """
    try:
        folder_path = Path(__file__).parent
    except NameError:
        folder_path = Path(os.getcwd())
    modules_to_reload = [
        f.stem for f in folder_path.glob("*.py")
        if f.stem not in exclude
    ]
    for module_name in modules_to_reload:
        if module_name in sys.modules:
            print(f"Recarregando módulo: {module_name}")
            importlib.reload(sys.modules[module_name])
        else:
            print(f"Importando módulo: {module_name}")
            importlib.import_module(module_name)
reload_modules()

from structure import Model, layers

# b = Model(
#     name = "Modelo-Static",
#     cases = [
#         # ("ep",  {"pressure": 1E6, "timePeriod": 1}),
#         ("bs",  {"curvature": 0.1, "timePeriod": 1}),
#         # ("ips",  {"pressure": 30E6, "timePeriod": 1}),
#         # ("is",  {"pressure": 1E6}),
#         # ("tt",  {"deformation": 0.006, "timePeriod": 1}),
#     ]
# )
# b.build(number_pitches=2, model_length=1, layers=layers)

# b = Model(
#     name = "Modelo-Settling",
#     cases = [
#         ("ss",  {"timePeriod": 0.01}),
#     ]
# )

# job_name = b.build(number_pitches=1, model_length=0.5, layers=layers)

# b = Model(
#     name = "Frequencia",
#     cases = [
#         ("f", {})
#     ]
# )
# b.build(number_pitches=1, model_length=0.5, layers=layers)

c = Model(
    name = "Modelo-Bending",
    cases = [
        # ("eps",  {"pressure": 2E6, "timePeriod": 0.2}),
        ("ep",  {"pressure": 1E6, "timePeriod": 0.05}),
        ("ip",  {"pressure": 30E6, "timePeriod": 0.2}),
        ("b",  {"curvature": 1, "timePeriod": 0.2}),
        # ("is",  {"pressure": 1E6}),
    ],
    # inp_job=job_name
)
c.build(number_pitches=1, model_length=0.5, layers=layers)

# session.viewports['Viewport: 1'].assemblyDisplay.geometryOptions.setValues(datumAxes=OFF, datumPlanes=OFF)