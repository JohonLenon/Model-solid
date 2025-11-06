# from caeModules import mesh
# from abaqusConstants import * # type: ignore
# from set_dados_gerais import pitch, fios, numVoltas, cilindros, limit_length


# def create_cilindros(model, s):
#     passo = min(pitch(fios["InnerArmour"][2]/2, fios["InnerArmour"][0]), limit_length)
#     print(f"O passo é {passo}.\n")
#     partes, p2 = list(), list()
#     for i, (keys, values) in enumerate(cilindros.items()):
#         # Gera geometria
#         s.CircleByCenterPerimeter(center=(0,0), point1=(0, values[0]/2))
#         partes.append(model.Part(name=keys, dimensionality=THREE_D, 
#         type=values[1]))
#         p2.append(model.parts[keys])
#         p2[i].BaseShellExtrude(sketch=s, depth= numVoltas*passo)
#         p2[i].setValues(geometryRefinement=EXTRA_FINE)
#         print(f"Geometria {keys} criada. \n")

#         # Cria malha
#         p2[i].seedPart(size=5)
#         p2[i].setElementType(regions=((p2[i].faces,)), elemTypes=(mesh.ElemType(elemCode=values[2]),))
#         p2[i].generateMesh()
#         print(f"Malha {keys} criada. \n")

#         # Cria seção e associa material
#         model.HomogeneousShellSection(name=f'Section_{keys}', material=f'{values[3]}Material',
#         thickness=values[4])
#         region = p2[i].Set(faces=p2[i].faces, name=f'Set-{keys}')
#         p2[i].SectionAssignment(region=region,
#                                 sectionName=f'Section_{keys}',
#                                 # offset=values[4],
#                                 offsetType=MIDDLE_SURFACE,
#                                 offsetField='',
#                                 thicknessAssignment=FROM_SECTION
#                                 )

#         # Cria assembly e posiciona
#         assembly = model.rootAssembly
#         assembly.Instance(name=keys, part=partes[i], dependent=ON)
#         assembly.rotate(instanceList=[keys], axisPoint = (0,0,0), axisDirection =(1, 0, 0), angle = -90)
#         g = s.geometry
#         for k in g.keys():
#             s.delete((g[k],))
#         print(f"Assembly {keys} criado. \n")