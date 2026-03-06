import os
import sys
from abaqus import mdb, session, highlight
from abaqusConstants import * # type: ignore
import importlib
from pathlib import Path

os.chdir(r'C:\Users\Johon\Desktop\Arquivos Importantes\Pessoal\Dissertação\Algoritmos')
sys.path.append(os.getcwd())
sys.stdout = sys.__stdout__

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

from set_dados_gerais import names, layers
from set_camadas import tension_armour, pitch
from create_contact import create_contact, create_general

nome_modelo = names["Model"]
mdb.Model(name='Model-1')
if  nome_modelo in mdb.models.keys():
    del mdb.models[nome_modelo]
mdb.Model(name=nome_modelo)
model = mdb.models[nome_modelo]
del mdb.models['Model-1']
s = model.ConstrainedSketch(name='rect', sheetSize=200.0)

for name, layer in layers.items():
    if layer["type"] == tension_armour:
        layer["pitch"] = pitch(layer['mid_diameter']/2, layer['lay_angle'])
        layer["length"] = layer['pitch']
passo = 2*max((abs(layer['pitch']) for layer in layers.values() if layer["type"] == tension_armour))
for name, layer in layers.items():
    if layer["type"] == tension_armour:
        layer["length"] = passo
# layers[min((name for name, layer in layers.items() if layer["type"] == tension_armour), key=lambda n: layers[n]["pitch"])]['length']=passo
for name, layer in layers.items():
    if layer["type"] != tension_armour:
        layer["pitch"]=passo

for name, layer in layers.items():
    type = layer["type"]
    kwargs = {k: v for k, v in layer.items() if k != "type"}
    obj = type(**kwargs)
    obj.create_geometry(model)

# create_contact(model)
create_general(model, passo)
session.viewports['Viewport: 1'].assemblyDisplay.geometryOptions.setValues(
datumAxes=OFF, datumPlanes=OFF)