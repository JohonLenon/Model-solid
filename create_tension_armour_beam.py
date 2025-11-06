# """
# Nessa função são criados os fios da armadura de tração com elementos de viga.
# """

# from caeModules import mesh
# from abaqusConstants import * # type: ignore
# from numpy import pi, sin, cos, sign
# from set_dados_gerais import pitch, n_pontos, numVoltas, height, width, limit_length, fios

# def create_tension_armour_beam(model):
#     passo = list()
#     for i, (keys, parameters) in enumerate(fios.items()):
#         # Definição de parâmetros
#         lay_angle = parameters[0]
#         number_strip = parameters[1]
#         radius = parameters[2]/2

#         passo.append(pitch(radius, lay_angle))
#         pontos = []

#         # Criação dos pontos da geometria helicoidal
#         for j in range(n_pontos + 1):
#             t = 2 * pi * numVoltas * j / n_pontos * sign(lay_angle)
#             x = radius * cos(t)
#             y = (passo[i] / (2 * pi)) * t
#             z = radius * sin(t)
#             if y > limit_length:
#                 break
#             pontos.append((x, y, z))

#         pontos = tuple(pontos)
#         part = model.Part(name=keys+'_BEAM', dimensionality=THREE_D, type=DEFORMABLE_BODY)
#         p = model.parts[keys+'_BEAM']
#         p.WireSpline(points=pontos)

#         print(f"Geometria {keys} criada.\n")

#         # Define a orientação da viga
#         region = p.Set(edges=p.edges, name=f'Set-{keys}')
#         part.assignBeamSectionOrientation(
#             region=region,
#             method=N1_COSINES,
#             n1=(0.0, 1.0, 0.0)
#         )
#         print(f"Orientação da viga {keys} definida. \n")

#         # Gera malha
#         p.seedEdgeByNumber(edges=p.edges, number=n_pontos)
#         p.setElementType(regions=(p.edges,), elemTypes=(mesh.ElemType(elemCode=B31),))
#         p.generateMesh()
#         print(f"Malha {keys} criada. \n")

#         # Cria perfil, seção e associa material
#         model.RectangularProfile(name= keys+'_Profile', a=height, b=width)
#         model.BeamSection(
#             name= keys+'_Section',
#             integration=DURING_ANALYSIS,
#             profile= keys+'_Profile',
#             material='METALLICMaterial',
#         )
#         p.SectionAssignment(
#             region=region,
#             sectionName= keys+'_Section',
#             thicknessAssignment=FROM_SECTION
#         )
#         print(f"Perfil, seção e materiais para {keys} atribuídos. \n")

#         # Cria o assembly e as cópias
#         assembly = model.rootAssembly
#         instance_name = f'{keys}-1'
#         assembly.Instance(name=instance_name, part=part, dependent=ON)
#         num_instances = parameters[1]
#         angle_step = 360.0 / num_instances

#         for j in range(1, num_instances):
#             rotation_angle = j * angle_step
#             instance_rotated_name = f'{keys}-{j+1}'
#             assembly.Instance(name=instance_rotated_name, part=part, dependent=ON)
#             assembly.rotate(instanceList=(instance_rotated_name,), axisPoint=(0, 0, 0), axisDirection=(0, 1, 0), angle=rotation_angle)
#         print(f"Assembly {keys} criado.\n")