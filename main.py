import os
import sys
from abaqus import session
from abaqusConstants import * # type: ignore
import importlib
from pathlib import Path

sys.path.append(os.getcwd())
sys.stdout = sys.__stdout__

print("\n")

def reload_modules(exclude=("main", "create_analysis", "create_contact")):
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

f = Model(
    name="Modelo_Frequencia",
    cases=[
        ("f", {}),
    ]
)
f.build(number_pitches=2, model_length=0, layers=layers)

b = Model(
    name = "Modelo_flexao",
    cases = [
        ("is",  {"pressure": 1E6}),
        ("b",  {"curvature": 1, "timePeriod": 1}),
    ]
)
b.build(number_pitches=2, model_length=0, layers=layers)

session.viewports['Viewport: 1'].assemblyDisplay.geometryOptions.setValues(datumAxes=OFF, datumPlanes=OFF)