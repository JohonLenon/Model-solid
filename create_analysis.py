"""Nessa função é criada a análise."""

from abaqus import mdb
from abaqusConstants import * # type: ignore
from create_region import create_region
from set_dados_gerais import names

def create_analysis(model, passo):
    assembly = model.rootAssembly
    periodoAnalise = 1

    model.ExplicitDynamicsStep(
    name= names['Step'], previous='Initial', timePeriod=periodoAnalise, 
    timeIncrementationMethod=AUTOMATIC_GLOBAL, maxIncrement = 0.0000002)

    endA = create_region(model, 0)
    endB = create_region(model, passo)

    model.TabularAmplitude(name='Amplitude', data=((0,0),(0.001,0),(1,1/periodoAnalise),))

    assembly.ReferencePoint(point=(0,0,0))
    assembly.ReferencePoint(point=(0,passo,0))

    refEndA = assembly.referencePoints[assembly.referencePoints.keys()[-1]]
    regiao_refEndA = assembly.Set(referencePoints=(refEndA,), name='Regiao-RPEndA')

    refEndB = assembly.referencePoints[assembly.referencePoints.keys()[-2]]
    regiao_refEndB = assembly.Set(referencePoints=(refEndB,), name='Regiao-RPEndB')

    model.Coupling(name='CouplingEndA', surface=endA, controlPoint=regiao_refEndA, influenceRadius=WHOLE_SURFACE, couplingType=KINEMATIC)
    model.Coupling(name='CouplingEndB', surface=endB, controlPoint=regiao_refEndB, influenceRadius=WHOLE_SURFACE, couplingType=KINEMATIC)

    # model.HistoryOutputRequest(
    # name='H-Output-EndA',
    # createStepName=names['Step'],
    # variables=('RF', 'UR', 'RM'),
    # timeInterval=0.0005,
    # region=assembly.sets['Regiao-RPEndA'])

    # model.HistoryOutputRequest(
    # name='H-Output-EndB',
    # createStepName=names['Step'],
    # variables=('RF', 'UR', 'RM'),
    # timeInterval=0.0005,
    # region=assembly.sets['Regiao-RPEndB'])

    model.FieldOutputRequest(
    name='F-Output-1',
    createStepName=names['Step'],
    variables=('S','E','U','V','A','RF','CSTRESS','CDISP','CSTATUS')
    )

    model.fieldOutputRequests['F-Output-1'].setValues(
        numIntervals=1000,
    )

    model.DisplacementBC(name="endA", createStepName= names['Step'], region=regiao_refEndA, u1=SET, u2=SET, u3=SET, ur1=SET, ur2=SET,
    ur3=SET, amplitude=UNSET)
    model.DisplacementBC(name="endB", createStepName= names['Step'], region=regiao_refEndB, u1=UNSET, u2=0.006*passo, u3=UNSET, ur1=UNSET, ur2=UNSET,
    ur3=UNSET, amplitude='Amplitude')

    job_name = 'Job-Explicit'
    mdb.Job(
        name=job_name,
        model=names['Model'],
        type=ANALYSIS,
        explicitPrecision=DOUBLE,
        nodalOutputPrecision=SINGLE,
        numCpus=8,
        numDomains=8,
        multiprocessingMode=DEFAULT
    )