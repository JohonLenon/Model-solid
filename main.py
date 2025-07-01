import os
import sys
from abaqus import mdb 

os.chdir(r'C:\Users\Johon\Desktop\Arquivos Importantes\Pessoal\Dissertação\Algoritmos')
sys.path.append(os.getcwd())

nome_modelo = 'ModeloRiser'
mdb.Model(name='Model-1')
if nome_modelo in mdb.models.keys():
    del mdb.models[nome_modelo]
mdb.Model(name=nome_modelo)
model = mdb.models[nome_modelo]
del mdb.models['Model-1']
s = model.ConstrainedSketch(name='rect', sheetSize=200.0)

from create_material import create_material
from create_tension_armour_beam import create_tension_armour_beam
from create_cilindros import create_cilindros
from create_contact import create_contact
from create_analysis import create_analysis

create_material(model)
create_tension_armour_beam(model)
create_cilindros(model, s)
create_contact(model)
create_analysis(model)