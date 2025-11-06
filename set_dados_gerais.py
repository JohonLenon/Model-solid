"""Nessa função são definidos todos os dados gerais do modelo."""

from abaqusConstants import ON, OFF
from numpy import pi, tan, radians
from set_camadas import cilindros, cilindros_ortotropicos, tension_armour

names = {
    "Model": "ModeloRiser",
    "Step": "Step_Explicit",
}

layers = {
    "InterlockedCarcass": {"type": cilindros_ortotropicos, "name": "InterlockedCarcass", "pitch": None, "internal_diameter": 66.7, "thickness": 3.5, "name_material": "AISI304", "young_modulus": 193000,"poisson": 0.3,"density": 7.85*10**(-9), "Ix": 556, "Iy": 20, "A": 19.6, "J": 6.5, "plastic_table": None},

    "PressureSheath": {"type": cilindros, "name": "PressureSheath", "pitch": None, "internal_diameter": 75.15, "thickness":  4.95, "name_material": "Grilamid", "young_modulus": 284,"poisson": 0.4,"density": 0.97*10**(-9),"plastic_table": None},

    "PressureReinforcement": {"type": cilindros_ortotropicos, "name": "PressureReinforcement", "pitch": None, "internal_diameter": 86.3, "thickness":  6.2, "name_material": "FI15", "young_modulus": 207000,"poisson": 0.3,"density": 7.85*10**(-9), "Ix": 771, "Iy": 100, "A": 51.5, "J": 204.6, "plastic_table": None},

    "InnerAntiFrictionTape": {"type": cilindros, "name": "InnerAntiFrictionTape", "pitch": None, "internal_diameter": 94, "thickness": 1.5, "name_material": "Rilsan", "young_modulus": 301,"poisson": 0.4,"density": 0.97*10**(-9),"plastic_table": None},

    "InnerArmour": {"type": tension_armour, "name": "InnerArmour", "pitch": None, "lay_angle": 35, "number_strips": 40, "internal_diameter": 95.5, "height":6, "width": 3, "points": 1000, "name_material": "FI41", "young_modulus": 211000, "poisson": 0.3, "density": 7.85*10**(-9), "revolve_direction": OFF, "plastic_table": None, "length": 0},

    "OuterAntiFrictionTape": {"type": cilindros, "name": "OuterAntiFrictionTape", "pitch": None, "internal_diameter": 103, "thickness": 1.5, "name_material": "Rilsan", "young_modulus": 301,"poisson": 0.4,"density": 0.97*10**(-9),"plastic_table": None},

    "OuterArmour": {"type": tension_armour, "name": "OuterArmour", "pitch": None, "lay_angle": 35, "number_strips": 44, "internal_diameter": 104.5, "height":6, "width": 3, "points": 1000, "name_material": "FI41", "young_modulus": 211000, "poisson": 0.3, "density": 7.85*10**(-9), "revolve_direction": ON, "plastic_table": None, "length": 0},

    "FabricTape": {"type": cilindros, "name": "FabricTape", "pitch": None, "internal_diameter": 111, "thickness": 0.5, "name_material": "HDPE", "young_modulus": 600,"poisson": 0.45,"density": 0.97*10**(-9),"plastic_table": None}
}

layer_order = {layer: idx for idx, layer in enumerate(layers)}
def obter_chave_ordenacao(s):
    parte = s.replace("Surf-", "")
    if "-" in parte:
        parte = "-".join(parte.split("-")[:-1])
    return layer_order.get(parte, float('inf'))

def pitch(R, angle):
    p = 2*pi*R/tan(radians(angle))
    return p