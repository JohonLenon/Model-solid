"""Nessa função são definidos todos os dados gerais do modelo."""

names = {
    "Model": "ModeloRiser",
    "Step_Loading": "Step_Explicit",
}

model_length = 2 #pitches

loading = {
    "Tension": 0, #def%
    "Internal_pressure": 0, #MPa
    "External_pressure": 1, #MPa
    "Curvature": 1, #1/m
    "Hysteresis": False
}

discretization = {
    "Tension armour": 6,
    "Circumferential seed": 44
}