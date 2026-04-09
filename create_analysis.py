"""Nessa função é criada a análise."""

from abaqus import mdb
from abaqusConstants import * # type: ignore
from create_set import create_set_geral, create_set_por_camada
from set_dados_gerais import names, layers

def create_analysis(model, passo):
    assembly = model.rootAssembly
    periodoAnalise = 1.5

    endA = create_set_geral(model, 0)
    endB = create_set_geral(model, passo)
    assembly.ReferencePoint(point=(0,0,0))
    assembly.ReferencePoint(point=(0,passo,0))
    refEndA = assembly.referencePoints[assembly.referencePoints.keys()[-1]]
    regiao_refEndA = assembly.Set(referencePoints=(refEndA,), name='Regiao-RPEndA')
    refEndB = assembly.referencePoints[assembly.referencePoints.keys()[-2]]
    regiao_refEndB = assembly.Set(referencePoints=(refEndB,), name='Regiao-RPEndB')
    model.Coupling(name='CouplingEndA', surface=endA, controlPoint=regiao_refEndA, influenceRadius=WHOLE_SURFACE, couplingType=KINEMATIC)
    model.Coupling(name='CouplingEndB', surface=endB, controlPoint=regiao_refEndB, influenceRadius=WHOLE_SURFACE, couplingType=KINEMATIC)
    
    model.SmoothStepAmplitude(name='Amplitude', data=((0,0),(periodoAnalise,1),))

    try:
        model.ExplicitDynamicsStep(name= names['Step1'], previous='Initial', timePeriod=periodoAnalise/3, timeIncrementationMethod=AUTOMATIC_GLOBAL)
        model.FieldOutputRequest(name='F-Output-1',createStepName=names['Step1'],variables=('S','LE','U','V','A','RF','CSTRESS','CDISP','CSTATUS', 'EMSF'))
        model.fieldOutputRequests['F-Output-1'].setValues(numIntervals=10,)
        external_surface = [y for (x,y) in list(assembly.surfaces.items()) if next(reversed(layers.keys())) in x][0]
        model.SmoothStepAmplitude(name='Amplitude_pressure', data=((0,0),(periodoAnalise/3,1),))
        model.Pressure(name="Pressure", createStepName = names['Step1'], region=external_surface, magnitude=2, amplitude='Amplitude_pressure')
        step_anterior = names['Step1']
    except:
        print("==\tModelo sem step de pressão externa\t==")
        step_anterior = "Initial"

    model.ExplicitDynamicsStep(name= names['Step2'], previous=step_anterior, timePeriod=periodoAnalise, timeIncrementationMethod=AUTOMATIC_GLOBAL)
    model.FieldOutputRequest(name='F-Output-2', createStepName=names['Step2'], variables=('S','LE','U','V','A','RF','CSTRESS','CDISP','CSTATUS', 'EMSF'))
    model.HistoryOutputRequest(name='H-Output-1', createStepName=names['Step2'], variables=('ALLAE', 'ALLCD', 'ALLDMD', 'ALLFD', 'ALLIE', 'ALLKE', 'ALLPD', 'ALLSE', 'ALLVD', 'ALLWK', 'ETOTAL', 'DMASS'))
    model.fieldOutputRequests['F-Output-2'].setValues(numIntervals=400,)
    model.DisplacementBC(name="endA_Bending", createStepName= step_anterior, region=regiao_refEndA, u1=SET, u2=SET, u3=SET, ur1=SET, ur2=SET, ur3=SET, amplitude=UNSET)
    model.DisplacementBC(name="endB_Bending", createStepName= names['Step2'], region=regiao_refEndB, u1=UNSET, u2=UNSET, u3=UNSET, ur1=UNSET, ur2=UNSET, ur3=passo/1000, amplitude='Amplitude')
    # model.DisplacementBC(name="endB_Tension", createStepName= names['Step2'], region=regiao_refEndB, u1=UNSET, u2=0.006*passo, u3=UNSET, ur1=UNSET, ur2=UNSET, ur3=UNSET, amplitude='Amplitude')

    job_name = 'Job-Explicit'
    mdb.Job(
        name=job_name,
        model=names['Model'],
        type=ANALYSIS,
        explicitPrecision=DOUBLE,
        nodalOutputPrecision=SINGLE,
        numCpus=12,
        numDomains=12,
        multiprocessingMode=DEFAULT
    )