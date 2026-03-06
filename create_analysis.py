"""Nessa função é criada a análise."""

from abaqus import mdb
from abaqusConstants import * # type: ignore
from create_region import create_region
from set_dados_gerais import names, layers

def create_analysis(model, passo):
    assembly = model.rootAssembly
    periodoAnalise = 1

    model.ExplicitDynamicsStep(
    name= names['Step1'], previous='Initial', timePeriod=periodoAnalise/1000, 
    timeIncrementationMethod=AUTOMATIC_GLOBAL)

    model.ExplicitDynamicsStep(
    name= names['Step2'], previous=names['Step1'], timePeriod=periodoAnalise, 
    timeIncrementationMethod=AUTOMATIC_GLOBAL)

    endA = create_region(model, 0)
    endB = create_region(model, passo)

    model.TabularAmplitude(name='Amplitude', data=((0,0),(1,1/periodoAnalise),))

    assembly.ReferencePoint(point=(0,0,0))
    assembly.ReferencePoint(point=(0,passo,0))

    assembly.DatumCsysByThreePoints(name="Cilindrico", coordSysType= CYLINDRICAL, origin=(0,0,0), point1=(1,0,0), point2=(0,0,1))

    refEndA = assembly.referencePoints[assembly.referencePoints.keys()[-1]]
    regiao_refEndA = assembly.Set(referencePoints=(refEndA,), name='Regiao-RPEndA')

    refEndB = assembly.referencePoints[assembly.referencePoints.keys()[-2]]
    regiao_refEndB = assembly.Set(referencePoints=(refEndB,), name='Regiao-RPEndB')

    k = [k for k, v in assembly.datums.items() if v.__class__.__name__ == 'DatumCsys']

    model.Coupling(name='CouplingEndA', surface=endA, controlPoint=regiao_refEndA, u1 = OFF, influenceRadius=WHOLE_SURFACE, couplingType=KINEMATIC, localCsys=assembly.datums[k[0]])
    model.Coupling(name='CouplingEndB', surface=endB, controlPoint=regiao_refEndB, u1 = OFF, influenceRadius=WHOLE_SURFACE, couplingType=KINEMATIC, localCsys=assembly.datums[k[0]])

    model.FieldOutputRequest(name='F-Output-1',createStepName=names['Step1'],variables=('S','LE','U','V','A','RF','CSTRESS','CDISP','CSTATUS'))
    model.fieldOutputRequests['F-Output-1'].setValues(numIntervals=10,)

    model.FieldOutputRequest(name='F-Output-2',createStepName=names['Step2'],variables=('S','LE','U','V','A','RF','CSTRESS','CDISP','CSTATUS'))
    model.fieldOutputRequests['F-Output-2'].setValues(numIntervals=1000,)

    model.DisplacementBC(name="endA", createStepName= names['Step1'], region=regiao_refEndA, u1=UNSET, u2=SET, u3=UNSET, ur1=UNSET, ur2=UNSET,
    ur3=UNSET, amplitude=UNSET)
    model.DisplacementBC(name="endB", createStepName= names['Step1'], region=regiao_refEndB, u1=UNSET, u2=SET, u3=UNSET, ur1=UNSET, ur2=UNSET,
    ur3=UNSET, amplitude=UNSET)

    model.boundaryConditions['endA'].deactivate(names['Step2'])
    model.boundaryConditions['endB'].deactivate(names['Step2'])

    model.DisplacementBC(name="endA_Bending", createStepName= names['Step2'], region=regiao_refEndA, u1=SET, u2=SET, u3=SET, ur1=SET, ur2=SET,
    ur3=SET, amplitude=UNSET)
    # model.DisplacementBC(name="endB_Bending", createStepName= names['Step2'], region=regiao_refEndB, u1=UNSET, u2=UNSET, u3=UNSET, ur1=UNSET, ur2=UNSET,
    # ur3=passo/1000, amplitude='Amplitude')    
    model.DisplacementBC(name="endB_Bending", createStepName= names['Step2'], region=regiao_refEndB, u1=UNSET, u2=0.06*passo, u3=UNSET, ur1=UNSET, ur2=UNSET,
    ur3=UNSET, amplitude='Amplitude')

    if "Step3" in names:
        model.fieldOutputRequests['F-Output-2'].setValues(numIntervals=10,)
        
        model.ExplicitDynamicsStep(name= names['Step3'], previous=names['Step2'], timePeriod=periodoAnalise, timeIncrementationMethod=AUTOMATIC_GLOBAL)

        model.FieldOutputRequest(name='F-Output-3',createStepName=names['Step3'],variables=('S','LE','U','V','A','RF','CSTRESS','CDISP','CSTATUS'))
        
        model.fieldOutputRequests['F-Output-3'].setValues(numIntervals=1000,)
        model.boundaryConditions["endB_Bending"].move(fromStepName=names["Step2"], toStepName=names["Step3"])
        
        external_surface = [y for (x,y) in list(assembly.surfaces.items()) if next(reversed(layers.keys())) in x][0]
        model.Pressure(name="Pressure", createStepName = names['Step2'], region=external_surface, magnitude=2, amplitude='Amplitude')

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