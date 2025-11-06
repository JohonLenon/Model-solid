"""Função para criação de uma região."""

def create_region(model, y_fixo):
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