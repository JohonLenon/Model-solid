"""Função para criação de um set para análise da curvatura."""

from set_dados_gerais import cilindros
from abaqusConstants import * # type: ignore

def create_set(model, y_fixo, position=1):
    assembly = model.rootAssembly
    nodes_at_point = list()
    for nome_instancia in assembly.instances.keys()[position:]:
        instancia = assembly.instances[nome_instancia]
        nodes = instancia.nodes
        nodes_at_point.append(nodes.getByBoundingBox(yMin=y_fixo-1, yMax=y_fixo+1, xMin=-1, xMax=1, zMin=0, zMax=1000))
    return model.rootAssembly.Set(nodes=nodes_at_point, name=f'RegiaoNodes-{abs(int(y_fixo))}')