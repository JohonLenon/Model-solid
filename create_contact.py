"""Nessa função são criados os contatos."""

from caeModules import regionToolset
from abaqusConstants import * # type: ignore
from set_dados_gerais import names
from set_camadas import layers, obter_chave_ordenacao, cilindros_ortotropicos, cilindros_shell
from numpy import sqrt
from create_analysis import create_analysis

def contact_props(model, nome_contato, tension = False):
    contact_prop = model.ContactProperty(nome_contato)
    contact_prop.NormalBehavior(
        pressureOverclosure=HARD,
        allowSeparation=ON
    )
    if tension:
        contact_prop.TangentialBehavior(
        formulation=PENALTY,
        directionality=ISOTROPIC,
        slipRateDependency=OFF,
        pressureDependency=OFF,
        temperatureDependency=OFF,
        dependencies=0,
        table=((0.1,),),
        shearStressLimit=None,
        maximumElasticSlip=FRACTION,
        fraction=0.005,
        elasticSlipStiffness=None)
    else:
        contact_prop.TangentialBehavior(
            formulation=FRICTIONLESS
        )
def create_contact(model, step = 'Initial'):
    print("====================== Contato =========================")
    assembly = model.rootAssembly
    contatos = {
        "ContatoTension": True,
        "ContatoShell": False
    }
    for item, values in contatos.items():
        contact_props(model, item, values)

    for key in layers.keys():
        if "Armour" not in key:
            model.rootAssembly.Surface(side1Faces=assembly.instances[key].faces, name=f'Surf-{key}-1')
            model.rootAssembly.Surface(side2Faces=assembly.instances[key].faces, name=f'Surf-{key}-2')

    surf_1 = [x for x in list(assembly.surfaces.keys()) if x.endswith('1')]
    surf_1.append(f'Surf-InnerArmour-1')
    # surf_1.append(f'Surf-OuterArmour-1')
    surf_1 = sorted(surf_1, key=obter_chave_ordenacao)
    surf_2 = [x for x in list(assembly.surfaces.keys()) if x.endswith('2')]
    surf_2.append(f'Surf-InnerArmour-2')
    # surf_2.append(f'Surf-OuterArmour-2')
    surf_2 = sorted(surf_2, key=obter_chave_ordenacao)

    for camada_1, camada_2 in zip(surf_1, surf_2[1:]):
        try:
            if 'innerarmour' in camada_1.lower():
                for i, nome_instancia in enumerate([x for x in assembly.instances.keys() if "innerarmour" in x.lower()], 1):
                    instancia = assembly.instances[nome_instancia]
                    region = regionToolset.Region(edges=instancia.edges)
                    model.SurfaceToSurfaceContactExp(
                        name=f'Contato_{camada_1}_{camada_2}-{i}',
                        createStepName=step,
                        main=assembly.surfaces[camada_2],
                        secondary=region,
                        sliding=FINITE,
                        interactionProperty=list(contatos.keys())[0],
                        mechanicalConstraint=KINEMATIC
                    )
            elif 'innerarmour' in camada_2.lower():
                for i, nome_instancia in enumerate([x for x in assembly.instances.keys() if "innerarmour" in x.lower()], 1):
                    instancia = assembly.instances[nome_instancia]
                    region = regionToolset.Region(edges=instancia.edges)
                    model.SurfaceToSurfaceContactExp(
                        name=f'Contato_{camada_1}_{camada_2}-{i}',
                        createStepName=step,
                        main=assembly.surfaces[camada_1],
                        secondary=region,
                        sliding=FINITE,
                        interactionProperty=list(contatos.keys())[0],
                        mechanicalConstraint=KINEMATIC
                    )
            elif 'outerarmour' in camada_1.lower():
                    for i, nome_instancia in enumerate([x for x in assembly.instances.keys() if "outerarmour" in x.lower()], 1):
                        instancia = assembly.instances[nome_instancia]
                        region = regionToolset.Region(edges=instancia.edges)
                        model.SurfaceToSurfaceContactExp(
                            name=f'Contato_{camada_1}_{camada_2}-{i}',
                            createStepName=step,
                            main=assembly.surfaces[camada_2],
                            secondary=region,
                            sliding=FINITE,
                            interactionProperty=list(contatos.keys())[0],
                            mechanicalConstraint=KINEMATIC
                        )
            elif 'outerarmour' in camada_2.lower():
                for i, nome_instancia in enumerate([x for x in assembly.instances.keys() if "outerarmour" in x.lower()], 1):
                    instancia = assembly.instances[nome_instancia]
                    region = regionToolset.Region(edges=instancia.edges)
                    model.SurfaceToSurfaceContactExp(
                        name=f'Contato_{camada_1}_{camada_2}-{i}',
                        createStepName=step,
                        main=assembly.surfaces[camada_1],
                        secondary=region,
                        sliding=FINITE,
                        interactionProperty=list(contatos.keys())[0],
                        mechanicalConstraint=KINEMATIC
                    )
            else:
                print(f"Contato entre {camada_1} e {camada_2}.\n")
                model.SurfaceToSurfaceContactExp(
                    name=f'Contato_{camada_1}_{camada_2}',
                    createStepName=step,
                    main=assembly.surfaces[camada_1],
                    secondary=assembly.surfaces[camada_2],
                    sliding=FINITE,
                    interactionProperty=list(contatos.keys())[1],
                    mechanicalConstraint=KINEMATIC
                )
        except Exception as e:
            print(f"{e.__class__.__name__}: {str(e)}.\n")

def create_surfaces_armour(model, nome_instancia, lado):
    assembly = model.rootAssembly
    instancia = assembly.instances[nome_instancia]
    internas, externas = list(), list()
    for face in instancia.faces:
        if face.getNormal()[1]==0:
            dot = sum(p * n for p, n in zip(face.pointOn[0], face.getNormal()))
            if dot>0:
                externas.append(instancia.faces[face.index:face.index+1])
            else:
                internas.append(instancia.faces[face.index:face.index+1])
    if lado == "Externo":
        model.rootAssembly.Surface(side1Faces=externas, name=f'Surf-{nome_instancia}-1')
    elif lado == "Interno":
        model.rootAssembly.Surface(side1Faces=internas, name=f'Surf-{nome_instancia}-2')

def create_contact_surfaces(model, nome_contato = "GeneralContact"):
    print(f"\n============================ {'Contato': ^24} ============================\n")
    assembly = model.rootAssembly
    contatos = {
        "ContatoTension": True,
        "ContatoShell": False
    }
    for item, values in contatos.items():
        contact_props(model, item, values)

    for key, values in layers.items():
        if "Armour" not in key and values['type'] not in (cilindros_ortotropicos, cilindros_shell):
            instancia = assembly.instances[key]
            internas, externas = list(), list()
            for face in instancia.faces:
                normal = face.getNormal()
                if abs(normal[0]) > 0.99:
                    continue
                if abs(normal[1]) < 1e-6:
                    dot = sum(p * n for p, n in zip(face.pointOn[0], normal))
                    if dot > 0:
                        externas.append(instancia.faces[face.index:face.index+1])
                    else:
                        internas.append(instancia.faces[face.index:face.index+1])
            model.rootAssembly.Surface(side1Faces=externas, name=f'Surf-{key}-1')
            model.rootAssembly.Surface(side1Faces=internas, name=f'Surf-{key}-2')
        elif values['type'] in (cilindros_ortotropicos, cilindros_shell):
            model.rootAssembly.Surface(side1Faces=assembly.instances[key].faces, name=f'Surf-{key}-1')
            model.rootAssembly.Surface(side2Faces=assembly.instances[key].faces, name=f'Surf-{key}-2')

    surf_1 = [x for x in list(assembly.surfaces.keys()) if x.endswith('1')]
    surf_1.append(f'Surf-InnerArmour-1')
    surf_1.append(f'Surf-OuterArmour-1')
    surf_1 = sorted(surf_1, key=obter_chave_ordenacao)
    surf_2 = [x for x in list(assembly.surfaces.keys()) if x.endswith('2')]
    surf_2.append(f'Surf-InnerArmour-2')
    surf_2.append(f'Surf-OuterArmour-2')
    surf_2 = sorted(surf_2, key=obter_chave_ordenacao)

    assignments = []

    for camada_1, camada_2 in zip(surf_1, surf_2[1:]):
        try:
            if 'innerarmour' in camada_1.lower():
                for nome_instancia in [x for x in assembly.instances.keys() if "innerarmour" in x.lower()]:
                    create_surfaces_armour(model, nome_instancia, "Externo")
                    assignments.append((assembly.surfaces["Surf-"+nome_instancia+"-1"], assembly.surfaces[camada_2], "Contact_Initialization"))
            elif 'innerarmour' in camada_2.lower():
                for nome_instancia in [x for x in assembly.instances.keys() if "innerarmour" in x.lower()]:
                    create_surfaces_armour(model, nome_instancia, "Interno")
                    assignments.append((assembly.surfaces[camada_1], assembly.surfaces["Surf-"+nome_instancia+"-2"],"Contact_Initialization"))
            elif 'outerarmour' in camada_1.lower():
                for nome_instancia in [x for x in assembly.instances.keys() if "outerarmour" in x.lower()]:
                    create_surfaces_armour(model, nome_instancia, "Externo")
                    assignments.append((assembly.surfaces["Surf-"+nome_instancia+"-1"], assembly.surfaces[camada_2], "Contact_Initialization"))
            elif 'outerarmour' in camada_2.lower():
                for nome_instancia in [x for x in assembly.instances.keys() if "outerarmour" in x.lower()]:
                    create_surfaces_armour(model, nome_instancia, "Interno")
                    assignments.append((assembly.surfaces[camada_1], assembly.surfaces["Surf-"+nome_instancia+"-2"],"Contact_Initialization"))
            else:
                assignments.append((assembly.surfaces[camada_1], assembly.surfaces[camada_2], "Contact_Initialization"))
        except Exception as e:
            print(f"{e.__class__.__name__}: {str(e)}.\t Camadas: {camada_1} e {camada_2}.\n")
        print(f"Contato entre {camada_1} e {camada_2} criado.\n")
    model.interactions[nome_contato].initializationAssignments.appendInStep(stepName='Initial', assignments=tuple(assignments))

def create_general(model, passo, step='Initial'):
    contatos = {
        "ContatoTension": True,
        "ContatoShell": False
    }
    nome_contato = 'GeneralContact'
    for item, values in contatos.items():
        contact_props(model, item, values)
    model.ContactExp(name=nome_contato, createStepName=step, contactPropertyAssignments=((GLOBAL, SELF, list(contatos.keys())[1]),))
    model.ExpInitialization(name='Contact_Initialization', overclosureType=INTERFERENCE)
    create_contact_surfaces(model)
    create_analysis(model, passo)
    model.interactions[nome_contato].deactivate(names['Step_Bending'])
    model.ContactExp(name=f"{nome_contato}-2", createStepName=names['Step_Bending'], contactPropertyAssignments=((GLOBAL, SELF, list(contatos.keys())[0]),))