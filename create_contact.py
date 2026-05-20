"""Nessa função são criados os contatos."""

from caeModules import regionToolset
from abaqusConstants import * # type: ignore
from set_dados_gerais import names
from set_camadas import layers, obter_chave_ordenacao, cylinder
from create_analysis import create_analysis

def contact_props(model, nome_contato, friction = False):
    """
    Define as propriedades de contato com e sem atrito.
    """
    contact_prop = model.ContactProperty(nome_contato)
    contact_prop.NormalBehavior(pressureOverclosure=HARD,allowSeparation=ON)
    if friction:
        contact_prop.TangentialBehavior(formulation=PENALTY,directionality=ISOTROPIC,slipRateDependency=OFF,pressureDependency=OFF,temperatureDependency=OFF,dependencies=0,table=((0.1,),),shearStressLimit=None,maximumElasticSlip=FRACTION,fraction=0.005,elasticSlipStiffness=None)
    else:
        contact_prop.TangentialBehavior(formulation=FRICTIONLESS)

def create_solid_helix_surfaces(model, nome_instancia, external=True):
    """
    Cria objetos do tipo surface no abaqus para definir as superfícies internas e externas das armaduras sólidas.
    Dessa forma, é possível estabelecer contatos com as outras camadas. Retorna o nome da superfície criada.
    """
    assembly = model.rootAssembly
    instancia = assembly.instances[nome_instancia]
    internas, externas = list(), list()
    for face in instancia.part.faces:
        if face.getNormal()[1]==0:
            dot = sum(p * n for p, n in zip(face.pointOn[0], face.getNormal()))
            if dot>0:
                externas.append(instancia.faces[face.index:face.index+1])
            else:
                internas.append(instancia.faces[face.index:face.index+1])
    faces, suffix = (externas, "Ext") if external else (internas, "Int")
    surface_name = f"Surf-{nome_instancia}-{suffix}"

    model.rootAssembly.Surface(side1Faces=faces, name=surface_name)

    return surface_name

def create_contact_surfaces(model):
    """
    Cria objetos do tipo surface no abaqus para definir as superfícies internas e externas de todas
    as camadas e as organiza em duas listas separadas e ordenadas de acordo com o dicionário de
    camadas. Retorna as listas.
    """
    assembly = model.rootAssembly
    externals, internals = {}, {}

    for key, values in layers.items():
        if issubclass(values['type'], cylinder):
            ext, inte = f'Surf-{key}-Ext', f'Surf-{key}-Int'
            faces = assembly.instances[key].faces
            model.rootAssembly.Surface(side1Faces=faces, name=ext)
            model.rootAssembly.Surface(side2Faces=faces, name=inte)
            externals[ext], internals[inte] = values['type'], values['type']
        else:
            for instance_name in (x for x in assembly.instances.keys() if key.lower() in x.lower()):
                externals[create_solid_helix_surfaces(model, instance_name, True)] = values['type']
                internals[create_solid_helix_surfaces(model, instance_name, False)] = values['type']
    
    ext, inte = list(externals.items()), list(internals.items())
    return ext, inte

def create_contact_initialization(model, nome_contato, externals, internals):
    """
    Cria uma inicialização das superfícies de contato do tipo 'Interference' com o objetivo de
    ajustar as posições dos nós antes da simulação.
    """
    assembly = model.rootAssembly
    model.ExpInitialization(name='Contact_Initialization', overclosureType=INTERFERENCE)
    assignments = []
    for i, (surface1, type1) in enumerate(externals):
        if issubclass(type1, cylinder):
            for surface2, type2 in internals[i+1:]:
                if issubclass(type2, cylinder):
                    break
                assignments.append((assembly.surfaces[surface1], assembly.surfaces[surface2], "Contact_Initialization"))
                print(f"Inicialização das superfícies {surface1} e {surface2}")
        else:
            for surface2, type2 in internals[i+1:]:
                if issubclass(type2, cylinder):
                    assignments.append((assembly.surfaces[surface1], assembly.surfaces[surface2], "Contact_Initialization"))
                    print(f"Inicialização das superfícies {surface1} e {surface2}")
                    break
    model.interactions[nome_contato].initializationAssignments.appendInStep(stepName='Initial', assignments=tuple(assignments))

def select_frictionless_layers(model, externals, internals):
    """
    Seleciona as camadas do tipo 'cylinder' para criação de um contato sem atrito.
    Retorna uma tupla de tuplas com as camadas selecionadas.
    """
    assembly = model.rootAssembly
    selected = []
    for (surface1, type1), (surface2, type2) in zip(externals, internals[1:]):
        if any(not issubclass(t, cylinder) for t in (type1, type2)):
            continue
        selected.append((assembly.surfaces[surface1], assembly.surfaces[surface2], "ContactFrictionless"))
    return tuple(selected)

def create_general(model, passo, step='Initial'):
    print(f"\n================================ {'Contato': ^24} ================================\n")
    contatos = {"ContactFriction": True, "ContactFrictionless": False}
    for item, values in contatos.items():
        contact_props(model, item, values)
    model.ContactExp(name='Initial_contact', createStepName=step, contactPropertyAssignments=((GLOBAL, SELF, list(contatos.keys())[1]),))
    create_analysis(model, passo)
    model.interactions['Initial_contact'].deactivate(names['Step_Loading'])
    ext, inte = create_contact_surfaces(model)
    create_contact_initialization(model, 'Initial_contact', ext, inte)
    frictionless_layers = select_frictionless_layers(model, ext, inte)
    model.ContactExp(name='Simulation_contact',
                     createStepName=names['Step_Loading'], contactPropertyAssignments=((GLOBAL, SELF, list(contatos.keys())[0]),) + frictionless_layers)