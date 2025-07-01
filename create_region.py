"""Função para criação de uma região."""

from set_dados_gerais import cilindros

def create_region(model, y_fixo):
    assembly = model.rootAssembly
    verticesNoPlano = list()
    arestasNoPlano = list()
    for nome_instancia in assembly.instances.keys():
        instancia = assembly.instances[nome_instancia]
        if nome_instancia.lower() not in (chave.lower() for chave in cilindros):
            vertices = instancia.vertices
            verticesNoPlano.append(vertices.getByBoundingBox(yMin=y_fixo-10, yMax=y_fixo+10))
        else:
            arestasNoPlano.append(instancia.edges.getByBoundingBox(yMin=y_fixo-10, yMax=y_fixo+10))
    return model.rootAssembly.Set(vertices=verticesNoPlano, edges=arestasNoPlano, name=f'RegiaoY-{abs(int(y_fixo))}')