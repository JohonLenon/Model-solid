"""Nessa função são definidos todos os dados gerais do modelo."""

from abaqusConstants import ON, OFF
from set_camadas import cilindros_ortotropicos, cilindros_solidos, tension_armour

names = {
    "Model": "ModeloRiser",
    "Step1": "Step_Settling",
    # "Step2": "Step_Pressure",
    "Step2": "Step_Explicit",
}

layers = {
    "InterlockedCarcass": {"type": cilindros_ortotropicos, "name": "InterlockedCarcass", "n": 1, "pitch": None, "mid_diameter": 66.7, "thickness": 3.5, "name_material": "AISI304", "young_modulus": 193000,"poisson": 0.3,"density": 7.85*10**(-9), "Ix": 20, "Iy": 556, "A": 19.6, "J": 6.5, "lay_angle": 87.5, "plastic_table": None},

    "PressureSheath": {"type": cilindros_solidos, "name": "PressureSheath", "pitch": None, "mid_diameter": 75.15, "thickness":  4.95, "name_material": "Grilamid", "young_modulus": 284,"poisson": 0.4,"density": 0.97*10**(-9),"plastic_table": None},

    "PressureReinforcement": {"type": cilindros_ortotropicos, "name": "PressureReinforcement", "n": 2, "pitch": None, "mid_diameter": 86.3, "thickness":  6.2, "name_material": "FI15", "young_modulus": 207000,"poisson": 0.3,"density": 7.85*10**(-9), "Ix": 771, "Iy": 100, "A": 51.5, "J": 204.6, "lay_angle": 85.5, "plastic_table": None},

    "InnerAntiFrictionTape": {"type": cilindros_solidos, "name": "InnerAntiFrictionTape", "pitch": None, "mid_diameter": 94, "thickness": 1.5, "name_material": "Rilsan", "young_modulus": 301,"poisson": 0.4,"density": 0.97*10**(-9),"plastic_table": None},

    "InnerArmour": {"type": tension_armour, "name": "InnerArmour", "pitch": None, "lay_angle": 35, "number_strips": 40, "mid_diameter": 98.5, "height":6, "width": 3, "points": 1000, "name_material": "FI41", "young_modulus": 211000, "poisson": 0.3, "density": 7.85*10**(-9), "revolve_direction": OFF, "plastic_table": None, "length": 0},

    "OuterAntiFrictionTape": {"type": cilindros_solidos, "name": "OuterAntiFrictionTape", "pitch": None, "mid_diameter": 103, "thickness": 1.5, "name_material": "Rilsan", "young_modulus": 301,"poisson": 0.4,"density": 0.97*10**(-9),"plastic_table": None},

    "OuterArmour": {"type": tension_armour, "name": "OuterArmour", "pitch": None, "lay_angle": 35, "number_strips": 44, "mid_diameter": 107.5, "height":6, "width": 3, "points": 1000, "name_material": "FI41", "young_modulus": 211000, "poisson": 0.3, "density": 7.85*10**(-9), "revolve_direction": ON, "plastic_table": None, "length": 0},

    "FabricTape": {"type": cilindros_solidos, "name": "FabricTape", "pitch": None, "mid_diameter": 111, "thickness": 0.5, "name_material": "HDPE", "young_modulus": 600,"poisson": 0.45,"density": 0.97*10**(-9),"plastic_table": None}
}

layer_order = {layer: idx for idx, layer in enumerate(layers)}
def obter_chave_ordenacao(s):
    parte = s.replace("Surf-", "")
    if "-" in parte:
        parte = "-".join(parte.split("-")[:-1])
    return layer_order.get(parte, float('inf'))