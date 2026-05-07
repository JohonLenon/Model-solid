"""Nessa função é criada a análise."""

from abaqus import mdb
from abaqusConstants import * # type: ignore
from create_set import create_set_geral
from set_dados_gerais import names, loading
from set_camadas import layers

def create_coupling(model, assembly, name, passo, origin = True):
    if origin:
        end = create_set_geral(model, 0)
        assembly.ReferencePoint(point=(0,0,0))
        ref = assembly.referencePoints[assembly.referencePoints.keys()[-1]]
        regiao = assembly.Set(referencePoints=(ref,), name=f'Regiao-{name}')
        model.Coupling(name=f'Coupling-{name}', surface=end, controlPoint=regiao, influenceRadius=WHOLE_SURFACE, couplingType=KINEMATIC)
    else:
        end = create_set_geral(model, passo)
        assembly.ReferencePoint(point=(0,passo,0))
        ref = assembly.referencePoints[assembly.referencePoints.keys()[-2]]
        regiao = assembly.Set(referencePoints=(ref,), name=f'Regiao-{name}')
        model.Coupling(name=f'Coupling-{name}', surface=end, controlPoint=regiao, influenceRadius=WHOLE_SURFACE, couplingType=KINEMATIC)
    return regiao

def create_amplitude(model, name, data, smooth=True):
    if smooth:
        model.SmoothStepAmplitude(name=name, data=data)
    else:
        model.TabularAmplitude(name=name, data=data)
    return name

def apply_tension(model, assembly, timePeriod, passo, regiao, displacement):
    model.ExplicitDynamicsStep(name= names['Step_Tension'], previous='Initial', timePeriod=timePeriod, timeIncrementationMethod=AUTOMATIC_GLOBAL)
    amplitude = create_amplitude(model, 'Amplitude', ((0,0),(1,1),(3,-1),(4,0))) if loading.get("Hysteresis") else create_amplitude(model, 'Amplitude', ((0,0),(1,1),))
    model.DisplacementBC(name="endB_Tension", createStepName= names['Step_Tension'], region=regiao, u1=UNSET, u2=displacement, u3=UNSET, ur1=UNSET, ur2=UNSET, ur3=UNSET, amplitude=amplitude)

    model.FieldOutputRequest(name='F-Output-Tension', createStepName=names['Step_Tension'], variables=('RM', 'UR'), region=assembly.sets['Regiao-RPEndB'])
    model.FieldOutputRequest(name='F-Output-TVisualization', createStepName=names['Step_Tension'], variables=('S','LE','U','V','A','RF','CSTRESS','CDISP','CSTATUS', 'EMSF'))
    model.HistoryOutputRequest(name='H-Output-Tension', createStepName=names['Step_Tension'], variables=('ALLAE', 'ALLCD', 'ALLDMD', 'ALLFD', 'ALLIE', 'ALLKE', 'ALLPD', 'ALLSE', 'ALLVD', 'ALLWK', 'ETOTAL', 'DMASS'))
    model.fieldOutputRequests['F-Output-Tension'].setValues(numIntervals=400,)
    model.fieldOutputRequests['F-Output-TVisualization'].setValues(numIntervals=10,)

def apply_bending(model, assembly, timePeriod, passo, regiao, curvature):
    rotation = passo*curvature
    model.ExplicitDynamicsStep(name= names['Step_Bending'], previous='Initial', timePeriod=timePeriod, timeIncrementationMethod=AUTOMATIC_GLOBAL)
    amplitude = create_amplitude(model, 'Amplitude', ((0,0),(1,1),(3,-1),(4,0))) if loading.get("Hysteresis") else create_amplitude(model, 'Amplitude', ((0,0),(1,1),))
    model.DisplacementBC(name="endB_Bending", createStepName= names['Step_Bending'], region=regiao, u1=UNSET, u2=UNSET, u3=UNSET, ur1=UNSET, ur2=UNSET, ur3=rotation, amplitude=amplitude)

    model.FieldOutputRequest(name='F-Output-Bending', createStepName=names['Step_Bending'], variables=('RM', 'UR'), region=assembly.sets['Regiao-RPEndB'])
    model.FieldOutputRequest(name='F-Output-BVisualization', createStepName=names['Step_Bending'], variables=('S','LE','U','V','A','RF','CSTRESS','CDISP','CSTATUS', 'EMSF'))
    model.HistoryOutputRequest(name='H-Output-Bending', createStepName=names['Step_Bending'], variables=('ALLAE', 'ALLCD', 'ALLDMD', 'ALLFD', 'ALLIE', 'ALLKE', 'ALLPD', 'ALLSE', 'ALLVD', 'ALLWK', 'ETOTAL', 'DMASS'))
    model.fieldOutputRequests['F-Output-Bending'].setValues(numIntervals=400,)
    model.fieldOutputRequests['F-Output-BVisualization'].setValues(numIntervals=10,)

def define_tension(pressure, external = True):
    if external:
        mid_diameter, thickness = (list(layers.values())[-1]["mid_diameter"], list(layers.values())[-1]["thickness"])
        radius = (mid_diameter+thickness)/2
        tension = (pressure * radius) / thickness
    else:
        mid_diameter, thickness = (list(layers.values())[1]["mid_diameter"], list(layers.values())[1]["thickness"])
        radius = (mid_diameter-thickness)/2
        tension = (pressure * radius) / thickness
    return tension if external else -tension

def apply_external_pressure(model, assembly, pressure):
    tension = define_tension(pressure=pressure, external=True)
    ultima_instancia = assembly.instances[next(reversed(assembly.instances.keys()))]
    regiao_ultima = ultima_instancia.sets[ultima_instancia.sets.keys()[0]]
    model.Stress(name='Tensao externa', region=regiao_ultima, distributionType=UNIFORM, sigma11=tension, sigma22=0.0, sigma12=0.0, sigma33=None, sigma13=None, sigma23=None)

def apply_internal_pressure(model, assembly, pressure):
    tension = define_tension(pressure=pressure, external=False)
    segunda_instancia = assembly.instances[assembly.instances.keys()[1]]
    regiao_segunda = segunda_instancia.sets[segunda_instancia.sets.keys()[0]]
    model.Stress(name='Tensao interna', region=regiao_segunda, distributionType=UNIFORM, sigma11=tension, sigma22=0.0, sigma12=0.0, sigma33=None, sigma13=None, sigma23=None)

def create_analysis(model, passo):
    assembly = model.rootAssembly
    periodoAnalise = 4 if loading.get("Hysteresis") else 1
    
    regiao_refEndA = create_coupling(model, assembly, 'RPEndA', passo)
    regiao_refEndB = create_coupling(model, assembly, 'RPEndB', passo, origin=False)
    model.DisplacementBC(name="endA", createStepName= 'Initial', region=regiao_refEndA, u1=SET, u2=SET, u3=SET, ur1=SET, ur2=SET, ur3=SET, amplitude=UNSET)

    initial_conditions = {
        "Internal_pressure": apply_internal_pressure,
        "External_pressure": apply_external_pressure,
    }

    for key, func in initial_conditions.items():
        value = loading.get(key)
        if value:
            func(model, assembly, pressure=value)

    step_handlers = {
        "Tension":   (apply_tension,  regiao_refEndB),
        "Curvature": (apply_bending,  regiao_refEndB),
    }

    for key, (func, regiao) in step_handlers.items():
        value = loading.get(key)
        if value:
            func(model, assembly, periodoAnalise, passo, regiao, value)

    job_name = 'Job-Explicit'
    mdb.Job(name=job_name, model=names['Model'], type=ANALYSIS, explicitPrecision=DOUBLE, nodalOutputPrecision=SINGLE, numCpus=48, numDomains=48, multiprocessingMode=DEFAULT)