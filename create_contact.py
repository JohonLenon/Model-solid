"""Nessa função são criados os contatos."""

from caeModules import regionToolset
from abaqusConstants import * # type: ignore
from set_dados_gerais import model, obter_chave_ordenacao, nome_step, cilindros

def create_contact(step = 'Initial', gerar_surf = True):
    assembly = model.rootAssembly
    if gerar_surf:
        for i, (keys, values) in enumerate(cilindros.items()):
            model.rootAssembly.Surface(side1Faces=assembly.instances[keys].faces, name=f'Surf-{keys}-1')
            model.rootAssembly.Surface(side2Faces=assembly.instances[keys].faces, name=f'Surf-{keys}-2')
    # else:
    #     for key, values in model.interactions.items():
    #         if step not in key:
    #             model.interactions[key].deactivate(step)

    surf_1 = [x for x in list(assembly.surfaces.keys()) if x.endswith('1')]
    surf_1.append(f'Surf-InnerArmour-1')
    surf_1.append(f'Surf-OuterArmour-1')
    surf_1 = sorted(surf_1, key=obter_chave_ordenacao)
    surf_2 = [x for x in list(assembly.surfaces.keys()) if x.endswith('2')]
    surf_2.append(f'Surf-InnerArmour-2')
    surf_2.append(f'Surf-OuterArmour-2')
    surf_2 = sorted(surf_2, key=obter_chave_ordenacao)

    nome_contato = "ContatoProp" if step == nome_step else "ContatoProp-Damping"
    contact_prop = model.ContactProperty(nome_contato)
    contact_prop.NormalBehavior(
        pressureOverclosure=HARD,
        allowSeparation=ON
    )
    contact_prop.TangentialBehavior(
        formulation=PENALTY,
        directionality=ISOTROPIC,
        table=((0.1, ), ),
        elasticSlipStiffness=None,
        fraction=1,
        maximumElasticSlip=FRACTION
    )

    for camada_1, camada_2 in zip(surf_1, surf_2[1:]):
        try:
            if 'innerarmour' in camada_1.lower():
                for i, nome_instancia in enumerate([x for x in assembly.instances.keys() if "innerarmour" in x.lower()], 1):
                    instancia = assembly.instances[nome_instancia]
                    region = regionToolset.Region(edges=instancia.edges)
                    model.SurfaceToSurfaceContactExp(
                        name=f'Contato_{camada_1}_{camada_2}-{i}-{step}',
                        createStepName=step,
                        main=assembly.surfaces[camada_2],
                        secondary=region,
                        sliding=FINITE,
                        interactionProperty=nome_contato,
                        mechanicalConstraint=KINEMATIC# if step == nome_step else PENALTY,
                    )
            elif 'innerarmour' in camada_2.lower():
                for i, nome_instancia in enumerate([x for x in assembly.instances.keys() if "innerarmour" in x.lower()], 1):
                    instancia = assembly.instances[nome_instancia]
                    region = regionToolset.Region(edges=instancia.edges)
                    model.SurfaceToSurfaceContactExp(
                        name=f'Contato_{camada_1}_{camada_2}-{i}-{step}',
                        createStepName=step,
                        main=assembly.surfaces[camada_1],
                        secondary=region,
                        sliding=FINITE,
                        interactionProperty=nome_contato,
                        mechanicalConstraint=KINEMATIC# if step == nome_step else PENALTY,
                    )
            elif 'outerarmour' in camada_1.lower():
                    for i, nome_instancia in enumerate([x for x in assembly.instances.keys() if "outerarmour" in x.lower()], 1):
                        instancia = assembly.instances[nome_instancia]
                        region = regionToolset.Region(edges=instancia.edges)
                        model.SurfaceToSurfaceContactExp(
                            name=f'Contato_{camada_1}_{camada_2}-{i}-{step}',
                            createStepName=step,
                            main=assembly.surfaces[camada_2],
                            secondary=region,
                            sliding=FINITE,
                            interactionProperty=nome_contato,
                            mechanicalConstraint=KINEMATIC# if step == nome_step else PENALTY,
                        )
            elif 'outerarmour' in camada_2.lower():
                for i, nome_instancia in enumerate([x for x in assembly.instances.keys() if "outerarmour" in x.lower()], 1):
                    instancia = assembly.instances[nome_instancia]
                    region = regionToolset.Region(edges=instancia.edges)
                    model.SurfaceToSurfaceContactExp(
                        name=f'Contato_{camada_1}_{camada_2}-{i}-{step}',
                        createStepName=step,
                        main=assembly.surfaces[camada_1],
                        secondary=region,
                        sliding=FINITE,
                        interactionProperty=nome_contato,
                        mechanicalConstraint=KINEMATIC# if step == nome_step else PENALTY,
                    )
            else:
                model.SurfaceToSurfaceContactExp(
                    name=f'Contato_{camada_1}_{camada_2}-{step}',
                    createStepName=step,
                    main=assembly.surfaces[camada_1],
                    secondary=assembly.surfaces[camada_2],
                    sliding=FINITE,
                    interactionProperty=nome_contato,
                    mechanicalConstraint=KINEMATIC# if step == nome_step else PENALTY,
                )
        except Exception as e:
            print(f"{e.__class__.__name__}: {str(e)}.\n")