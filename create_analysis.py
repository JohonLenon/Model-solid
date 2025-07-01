"""Nessa função é criada a análise."""

from abaqus import mdb
from abaqusConstants import * # type: ignore
from set_dados_gerais import model, nome_step, pitch, fios, nome_buildup, create_region, numVoltas, nome_modelo

def create_analysis():
    passo = pitch(fios["InnerArmour"][3]/2, fios["InnerArmour"][0])
    assembly = model.rootAssembly
    periodoAnalise = 3

    model.ExplicitDynamicsStep(
    name= nome_buildup, previous='Initial', timePeriod=0.001, 
    timeIncrementationMethod=AUTOMATIC_GLOBAL, maxIncrement = 0.1)

    model.ExplicitDynamicsStep(
    name= nome_step, previous=nome_buildup, timePeriod=periodoAnalise, 
    timeIncrementationMethod=AUTOMATIC_GLOBAL, maxIncrement = 0.0000002)

    endA = create_region(0)
    endB = create_region(numVoltas*passo)

    model.TabularAmplitude(name='Amplitude', data=((0,0),(0.001,0),(1,1/periodoAnalise),))

    assembly.ReferencePoint(point=(0,0,0))
    assembly.ReferencePoint(point=(0,numVoltas*passo,0))

    refEndA = assembly.referencePoints[assembly.referencePoints.keys()[-1]]
    regiao_refEndA = assembly.Set(referencePoints=(refEndA,), name='Regiao-RPEndA')

    refEndB = assembly.referencePoints[assembly.referencePoints.keys()[-2]]
    regiao_refEndB = assembly.Set(referencePoints=(refEndB,), name='Regiao-RPEndB')

    model.Coupling(name='CouplingEndA', surface=endA, controlPoint=regiao_refEndA, influenceRadius=WHOLE_SURFACE, couplingType=KINEMATIC)
    model.Coupling(name='CouplingEndB', surface=endB, controlPoint=regiao_refEndB, influenceRadius=WHOLE_SURFACE, couplingType=KINEMATIC)

    model.HistoryOutputRequest(
    name='H-Output-EndA',
    createStepName=nome_step,
    variables=('RF', 'UR', 'RM'),
    timeInterval=0.0005,
    region=assembly.sets['Regiao-RPEndA'])

    model.HistoryOutputRequest(
    name='H-Output-EndB',
    createStepName=nome_step,
    variables=('RF', 'UR', 'RM'),
    timeInterval=0.0005,
    region=assembly.sets['Regiao-RPEndB'])

    model.FieldOutputRequest(
    name='F-Output-1',
    createStepName=nome_step,
    variables=PRESELECT
    )

    model.fieldOutputRequests['F-Output-1'].setValues(
        numIntervals=1000,
    )

    model.DisplacementBC(name="endA", createStepName= nome_step, region=regiao_refEndA, u1=SET, u2=SET, u3=SET, ur1=SET, ur2=SET,
    ur3=SET, amplitude=UNSET)
    model.DisplacementBC(name="endB", createStepName= nome_step, region=regiao_refEndB, u1=UNSET, u2=UNSET, u3=UNSET, ur1=UNSET, ur2=UNSET,
    ur3=0.03, amplitude='Amplitude')

    job_name = 'Job-Explict'
    mdb.Job(
    name=job_name,
    model=nome_modelo,
    type=ANALYSIS,
    explicitPrecision=DOUBLE,
    nodalOutputPrecision=SINGLE,
    numCpus=4,
    numDomains=8,
    multiprocessingMode=DEFAULT)
    # mdb.jobs[job_name].submit(consistencyChecking=OFF)