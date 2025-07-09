"""Nessa função é criada a análise."""

from abaqus import mdb
from abaqusConstants import * # type: ignore
from create_region import create_region
from create_set_curvature import create_set
from set_dados_gerais import nome_step, pitch, fios, nome_buildup, numVoltas, nome_modelo

def create_analysis(model):
    # passo = pitch(fios["InnerArmour"][2]/2, fios["InnerArmour"][0])
    passo = 500
    assembly = model.rootAssembly
    periodoAnalise = 1

    # model.ExplicitDynamicsStep(
    # name= nome_buildup, previous='Initial', timePeriod=0.001, 
    # timeIncrementationMethod=AUTOMATIC_GLOBAL, maxIncrement = 0.1)

    model.ExplicitDynamicsStep(
    name= nome_step, previous='Initial', timePeriod=periodoAnalise, 
    timeIncrementationMethod=AUTOMATIC_GLOBAL, maxIncrement = 0.0000002)

    endA = create_region(model, 0)
    endB = create_region(model, passo)

    curv_left = create_set(model, 245)
    curv_center = create_set(model, 250)
    curv_right = create_set(model, 255)

    model.TabularAmplitude(name='Amplitude', data=((0,0),(1,1/periodoAnalise),))

    assembly.ReferencePoint(point=(0,0,0))
    assembly.ReferencePoint(point=(0,passo,0))

    refEndA = assembly.referencePoints[assembly.referencePoints.keys()[-1]]
    regiao_refEndA = assembly.Set(referencePoints=(refEndA,), name='Regiao-RPEndA')

    refEndB = assembly.referencePoints[assembly.referencePoints.keys()[-2]]
    regiao_refEndB = assembly.Set(referencePoints=(refEndB,), name='Regiao-RPEndB')

    model.Coupling(name='CouplingEndA', surface=endA, controlPoint=regiao_refEndA, influenceRadius=WHOLE_SURFACE, couplingType=KINEMATIC)
    model.Coupling(name='CouplingEndB', surface=endB, controlPoint=regiao_refEndB, influenceRadius=WHOLE_SURFACE, couplingType=KINEMATIC, u1=OFF, u2=OFF, u3=OFF, ur1=OFF, ur2=OFF)

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

    model.HistoryOutputRequest(
    name='H-Output-Left',
    createStepName=nome_step,
    variables=PRESELECT,
    timeInterval=0.0005,
    region=assembly.sets['RegiaoNodes-245']
    )

    model.HistoryOutputRequest(
    name='H-Output-Center',
    createStepName=nome_step,
    variables=PRESELECT,
    timeInterval=0.0005,
    region=assembly.sets['RegiaoNodes-250']
    )

    model.HistoryOutputRequest(
    name='H-Output-Right',
    createStepName=nome_step,
    variables=PRESELECT,
    timeInterval=0.0005,
    region=assembly.sets['RegiaoNodes-255']
    )

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
    ur3=0.3, amplitude='Amplitude')

    job_name = 'Job-Explict'
    mdb.Job(
    name=job_name,
    model=nome_modelo,
    type=ANALYSIS,
    explicitPrecision=DOUBLE,
    nodalOutputPrecision=SINGLE,
    # numCpus=4,
    # numDomains=8,
    # multiprocessingMode=DEFAULT
    )
    # mdb.jobs[job_name].submit(consistencyChecking=OFF)