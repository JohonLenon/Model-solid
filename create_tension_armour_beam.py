"""
Nessa função são criados os fios da armadura de tração com elementos de viga.
"""

from caeModules import mesh
from abaqusConstants import * # type: ignore
from numpy import pi, sin, cos
from set_dados_gerais import model, pitch, n_pontos, numVoltas, height, width, fios

def create_tension_armour_beam():
    passo = list()
    for i, (keys, values) in enumerate(fios.items()):
        # Gera geometria
        # =====================================================================================================================
        passo.append(pitch(values[3]/2, values[0]))
        pontos = []
        for j in range(n_pontos + 1):
            t = 2 * pi * numVoltas * j / n_pontos if values[0]>0 else -2 * pi * numVoltas * j / n_pontos
            x = values[3]/2 * cos(t)
            y = (passo[i] / (2 * pi)) * t
            z = values[3]/2 * sin(t)

            if y > numVoltas*passo[0]:
                break
            pontos.append((x, y, z))

        pontos = tuple(pontos)
        part = model.Part(name=keys+'_BEAM', dimensionality=THREE_D, type=DEFORMABLE_BODY)
        p = model.parts[keys+'_BEAM']
        p.WireSpline(points=pontos)

        print(f"Geometria {keys} criada.\n")

        # Define a orientação da viga
        # =====================================================================================================================
        region = p.Set(edges=p.edges, name=f'Set-{keys}')
        # region = regionToolset.Region(edges=p.edges)

        part.assignBeamSectionOrientation(
            region=region,
            method=N1_COSINES,
            n1=(0.0, 1.0, 0.0)
        )

        print(f"Orientação da viga {keys} definida. \n")

        # Gera malha
        # =====================================================================================================================
        # p.seedPart(size=0.5)
        p.seedEdgeByNumber(edges=p.edges, number=n_pontos)
        p.setElementType(regions=(p.edges,), elemTypes=(mesh.ElemType(elemCode=B31),))
        p.generateMesh()

        print(f"Malha {keys} criada. \n")

        # Cria perfil, seção e associa material
        # =====================================================================================================================
        model.RectangularProfile(name= keys+'_Profile', a=height, b=width)
        model.BeamSection(
            name= keys+'_Section',
            integration=DURING_ANALYSIS,
            profile= keys+'_Profile',
            material='METALLICMaterial',
            # beamSectionOffset = (0,width/2) if values[0]<0 else (0,-width/2)
        )
        p.SectionAssignment(
            region=region,
            sectionName= keys+'_Section',
            thicknessAssignment=FROM_SECTION
        )

        print(f"Perfil, seção e materiais para {keys} atribuídos. \n")

        # Cria o assembly e as cópias
        # =====================================================================================================================
        assembly = model.rootAssembly
        instance_name = f'{keys}-1'
        assembly.Instance(name=instance_name, part=part, dependent=ON)
        num_instances = fios[f'{keys}'][2]
        angle_step = 360.0 / num_instances

        for j in range(1, num_instances):
            rotation_angle = j * angle_step
            instance_rotated_name = f'{keys}-{j+1}'
            assembly.Instance(name=instance_rotated_name, part=part, dependent=ON)
            assembly.rotate(instanceList=(instance_rotated_name,), axisPoint=(0, 0, 0), axisDirection=(0, 1, 0), angle=rotation_angle)
        
        # for k in g.keys():
        #     s.delete((g[k],))
        print(f"Assembly {keys} criado.\n")