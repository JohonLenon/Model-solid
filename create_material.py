"""
Nessa função crio todos os materiais utilizados no modelo.
"""

from abaqusConstants import * # type: ignore
from caeModules import * # type: ignore

def create_material(model):
    # Cria material
    # =====================================================================================================================
    material = model.Material(name='METALLICMaterial')

    E = 207000
    nu = 0.3
    material.Elastic(table=((E, nu),))
    material.Density(table=((7.85*10**(-9),),))

    sigma_y = 650
    Et = 1172.58
    sigma_plastic = sigma_y + 10
    epsilon_plastic = (sigma_plastic - sigma_y) / Et
    material.Plastic(table=((sigma_y, 0.0), (sigma_plastic, epsilon_plastic)))

    print(f"Material {material.name} criado.\n")

    # =====================================================================================================================
    material = model.Material(name='HDPEMaterial')

    material.Elastic(table=((571, 0.45),))
    material.Density(table=((0.97*10**(-9),),))

    plastic_table = (
        (20.74,0),
        (23.1578947368421,0.0077348066298342),
        (25.5921052631578,0.0172375690607735),
        (26.9736842105263,0.0314917127071823),
        (27.4999999999999,0.0499447513812155)
    )

    material.Plastic(table=plastic_table)
    print(f"Material {material.name} criado. \n")
    # =====================================================================================================================
    material = model.Material(name='KEVLARMaterial')

    material.Elastic(table=((120000,0.36),))
    material.Density(table=((1.44*10**(-9),),))

    plastic_table = (
        (1310,0),
        (1439.0243902439,0.0012681159420289),
        (1556.09756097561,0.002427536231884),
        (1673.17073170731,0.0040217391304347),
        (1790.24390243902,0.0056521739130434),
        (1892.68292682926,0.0076086956521739),
        (1907.31707317073,0.008659420289855)
    )

    material.Plastic(table=plastic_table)
    print(f"Material {material.name} criado. \n")
    # =====================================================================================================================