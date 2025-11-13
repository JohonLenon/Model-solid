"""Nessa função são criados os contatos."""

from caeModules import regionToolset
from abaqusConstants import * # type: ignore
from set_dados_gerais import layers, obter_chave_ordenacao

def contact_props(model, nome_contato, tension = False):
    contact_prop = model.ContactProperty(nome_contato)
    contact_prop.NormalBehavior(
        pressureOverclosure=HARD,
        allowSeparation=ON
    )
    if tension:
        contact_prop.TangentialBehavior(
            formulation=FRICTIONLESS,
            directionality=ISOTROPIC,
            table=((0.1, ), ),
            elasticSlipStiffness=None,
            fraction=1,
            maximumElasticSlip=FRACTION
        )
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

def create_general(model, step='Initial'):
    print("====================== Contato =========================")
    contatos = {
        "ContatoTension": True,
        "ContatoShell": False
    }
    nome_contato = 'GeneralContact'
    for item, values in contatos.items():
        contact_props(model, item, values)
    model.ContactExp(name=nome_contato, createStepName=step, contactPropertyAssignments=((GLOBAL, SELF, list(contatos.keys())[0]),))
    # model.interactions[nome_contato].setValues(globalSmoothing=True)