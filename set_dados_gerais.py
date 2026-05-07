"""Nessa função são definidos todos os dados gerais do modelo."""

names = {
    "Model": "ModeloRiser",
    "Step_Bending": "Step_Explicit",
}

model_length = 2 #pitches

loading = {
    "Tension": 0,
    "Internal_pressure": 30,
    "External_pressure": 1,
    "Curvature": 1,
    "Hysteresis": False
}

discretization = {
    "Tension armour": 6,
    "Circumferential seed": 44
}