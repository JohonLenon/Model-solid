"""Função para criação de um set."""
from abaqusConstants import * # type: ignore

def create_set_geral(model, y_fixo):
    assembly = model.rootAssembly
    verticesNoPlano = list()
    arestasNoPlano = list()
    for nome_instancia in assembly.instances.keys():
        instancia = assembly.instances[nome_instancia]
        if "armour" in nome_instancia.lower():
            vertices = instancia.vertices
            verticesNoPlano.append(vertices.getByBoundingBox(yMin=y_fixo-10, yMax=y_fixo+10))
        else:
            arestasNoPlano.append(instancia.edges.getByBoundingBox(yMin=y_fixo-10, yMax=y_fixo+10))
    return model.rootAssembly.Set(vertices=verticesNoPlano, edges=arestasNoPlano, name=f'RegiaoY-{abs(int(y_fixo))}')

def create_set_por_camada(model, y_fixo):
    assembly = model.rootAssembly
    sets, innerarmour, outerarmour = list(), list(), list()
    for nome_instancia in assembly.instances.keys():
        instancia = assembly.instances[nome_instancia]
        if "innerarmour" in nome_instancia.lower():
            innerarmour.append(instancia.vertices)
        elif "outerarmour" in nome_instancia.lower():
            outerarmour.append(instancia.vertices)
        else:
            sets.append(model.rootAssembly.Set(edges=instancia.edges.getByBoundingBox(yMin=y_fixo-10, yMax=y_fixo+10), name=f'Regiao-{nome_instancia}-{abs(int(y_fixo))}'))
            assembly.ReferencePoint(point=(0,y_fixo,0))
            RP = assembly.referencePoints[assembly.referencePoints.keys()[-1]]
            regiao = assembly.Set(referencePoints=(RP,), name=f'Regiao-RP-{nome_instancia}-{abs(int(y_fixo))}')
            model.Coupling(name=f'Coupling-{nome_instancia}-{abs(int(y_fixo))}', surface=sets[-1], controlPoint=regiao, influenceRadius=WHOLE_SURFACE, couplingType=DISTRIBUTING)
    sets.append(model.rootAssembly.Set(vertices=innerarmour, name=f'Regiao-InnerArmour-{abs(int(y_fixo))}'))
    assembly.ReferencePoint(point=(0,y_fixo,0))
    RP = assembly.referencePoints[assembly.referencePoints.keys()[-1]]
    regiao = assembly.Set(referencePoints=(RP,), name=f'Regiao-RP-InnerArmour-{abs(int(y_fixo))}')
    model.Coupling(name=f'Coupling-InnerArmour-{abs(int(y_fixo))}', surface=sets[-1], controlPoint=regiao, influenceRadius=WHOLE_SURFACE, couplingType=DISTRIBUTING)
    sets.append(model.rootAssembly.Set(vertices=outerarmour, name=f'Regiao-OuterArmour-{abs(int(y_fixo))}'))
    assembly.ReferencePoint(point=(0,y_fixo,0))
    RP = assembly.referencePoints[assembly.referencePoints.keys()[-1]]
    regiao = assembly.Set(referencePoints=(RP,), name=f'Regiao-RP-OuterArmour-{abs(int(y_fixo))}')
    model.Coupling(name=f'Coupling-OuterArmour-{abs(int(y_fixo))}', surface=sets[-1], controlPoint=regiao, influenceRadius=WHOLE_SURFACE, couplingType=DISTRIBUTING)
    return sets