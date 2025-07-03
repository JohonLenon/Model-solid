import os
import sys
from abaqus import mdb
from numpy import pi, tan, radians

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

if 'cilindros' in sys.modules:
    del sys.modules['cilindros']
    del sys.modules['tension_armour']
    del sys.modules['create_contact']
    del sys.modules['create_analysis']

from set_camadas import cilindros, tension_armour
from create_contact import create_contact
from create_analysis import create_analysis

pitch = 2*pi*97/2/tan(radians(35))
length = 500

camadas = dict()
camadas["InterlockedCarcass"] = cilindros(name = "InterlockedCarcass",
                                          pitch = length,
                                          internal_diameter = 63.2,
                                          thickness=3.5,
                                          name_material= "METALLIC",
                                          young_modulus=207000,
                                          poisson=0.3,
                                          density=7.85*10**(-9)).create_geometry(model, s)
camadas["PressureReinforcement"] = cilindros(name = "PressureReinforcement",
                                             pitch = length,
                                             internal_diameter = 80.1,
                                             thickness=6.2,
                                             name_material= "METALLIC",
                                             young_modulus=207000,
                                             poisson=0.3,
                                             density=7.85*10**(-9)).create_geometry(model, s)
camadas['PressureSheath'] = cilindros(name = "PressureSheath",
                                      pitch = length,
                                      internal_diameter = 70.2,
                                      thickness = 4.95,
                                      name_material = "HDPE",
                                      young_modulus = 571,
                                      poisson = 0.45,
                                      density = 0.97*10**(-9),
                                      plastic_table = ((20.74,0),
                                                    (23.1578947368421,0.0077348066298342),
                                                    (25.5921052631578,0.0172375690607735),
                                                    (26.9736842105263,0.0314917127071823),
                                                    (27.4999999999999,0.0499447513812155))
                                      ).create_geometry(model, s)
camadas['InnerAntiFrictionTape'] = cilindros(name = "InnerAntiFrictionTape",
                                      pitch = length,
                                      internal_diameter = 92.5,
                                      thickness = 1.5,
                                      name_material = "HDPE",
                                      young_modulus = 571,
                                      poisson = 0.45,
                                      density = 0.97*10**(-9),
                                      plastic_table = ((20.74,0),
                                                    (23.1578947368421,0.0077348066298342),
                                                    (25.5921052631578,0.0172375690607735),
                                                    (26.9736842105263,0.0314917127071823),
                                                    (27.4999999999999,0.0499447513812155))
                                      ).create_geometry(model, s)
camadas['InnerArmour'] = tension_armour(name = "InnerArmour",
                                        pitch = pitch,
                                        lay_angle = 35,
                                        number_strips = 40,
                                        internal_diameter = 97,
                                        height = 3,
                                        width = 6,
                                        points = 1000,
                                        name_material= "METALLIC",
                                        young_modulus=207000,
                                        poisson=0.3,
                                        density=7.85*10**(-9),
                                        length = length
                                        ).create_geometry(model)
camadas['OuterArmour'] = tension_armour(name = "OuterArmour",
                                        pitch = pitch,
                                        lay_angle = -35,
                                        number_strips = 44,
                                        internal_diameter = 106,
                                        height = 3,
                                        width = 6,
                                        points = 1000,
                                        name_material= "METALLIC",
                                        young_modulus=207000,
                                        poisson=0.3,
                                        density=7.85*10**(-9),
                                        length = length
                                        ).create_geometry(model)
camadas['OuterAntiFrictionTape'] = cilindros(name = "OuterAntiFrictionTape",
                                      pitch = length,
                                      internal_diameter = 101.5,
                                      thickness = 1.5,
                                      name_material = "HDPE",
                                      young_modulus = 571,
                                      poisson = 0.45,
                                      density = 0.97*10**(-9),
                                      plastic_table = ((20.74,0),
                                                    (23.1578947368421,0.0077348066298342),
                                                    (25.5921052631578,0.0172375690607735),
                                                    (26.9736842105263,0.0314917127071823),
                                                    (27.4999999999999,0.0499447513812155))
                                      ).create_geometry(model, s)
camadas['FabricTape'] = cilindros(name = "FabricTape",
                                      pitch = length,
                                      internal_diameter = 110.5,
                                      thickness = 0.5,
                                      name_material = "HDPE",
                                      young_modulus = 571,
                                      poisson = 0.45,
                                      density = 0.97*10**(-9),
                                      plastic_table = ((20.74,0),
                                                    (23.1578947368421,0.0077348066298342),
                                                    (25.5921052631578,0.0172375690607735),
                                                    (26.9736842105263,0.0314917127071823),
                                                    (27.4999999999999,0.0499447513812155))
                                      ).create_geometry(model, s)


create_contact(model)
create_analysis(model)