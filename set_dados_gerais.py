"""Nessa função são definidos todos os dados gerais do modelo."""

from abaqus import mdb
from abaqusConstants import ON, OFF, DEFORMABLE_BODY, S4
from numpy import pi, tan, radians

width, height = 3, 6
numVoltas = 2
n_pontos = 100
nome_step = 'Step_Explict'
nome_buildup = 'Step_BuildUp'
nome_modelo = 'ModeloRiser'

fios = {
        "InnerArmour": [35, OFF, 40, 97],
        "OuterArmour": [-35, ON, 44, 104.5]
        }

cilindros ={
    "InterlockedCarcass": [63.2, DEFORMABLE_BODY, S4, 'METALLIC', 3.5],
    "PressureSheath": [70.2, DEFORMABLE_BODY, S4, 'HDPE', 4.95],
    "PressureReinforcement": [80.1, DEFORMABLE_BODY, S4, 'METALLIC', 6.2],
    "InnerAntiFrictionTape": [92.5, DEFORMABLE_BODY, S4, 'HDPE', 1.5],
    "OuterAntiFrictionTape": [101.5, DEFORMABLE_BODY, S4, 'HDPE', 1.5],
    "FabricTape": [110.5, DEFORMABLE_BODY, S4, 'HDPE', 0.5],
}

camadas =[
    "InterlockedCarcass",
    "PressureSheath",
    "PressureReinforcement",
    "InnerAntiFrictionTape",
    "InnerArmour",
    "OuterAntiFrictionTape",
    "OuterArmour",
    "FabricTape"]

ordem_camadas = {camada: idx for idx, camada in enumerate(camadas)}
def obter_chave_ordenacao(s):
    parte = s.replace("Surf-", "")
    if "-" in parte:
        parte = "-".join(parte.split("-")[:-1])
    return ordem_camadas.get(parte, float('inf'))

# Cálculo do passo de hélice
# =====================================================================================================================
def pitch(R, angle):
    p = 2*pi*R*tan(radians(angle))
    return p
passo = pitch(fios["InnerArmour"][3]/2, fios["InnerArmour"][0])

# Construção de um plano de sketch
# =====================================================================================================================
# nome_modelo = 'ModeloRiser'
# mdb.Model(name='Model-1')
# if nome_modelo in mdb.models.keys():
#     del mdb.models[nome_modelo]
# mdb.Model(name=nome_modelo)
# model = mdb.models[nome_modelo]
# del mdb.models['Model-1']

# s = model.ConstrainedSketch(name='rect', sheetSize=200.0)
# s.setPrimaryObject(option=STANDALONE)