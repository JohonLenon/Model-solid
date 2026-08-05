from caeModules import mesh
from abaqus import mdb, session
from abaqusConstants import * # type: ignore
from numpy import pi, sin, cos, tan, radians, array, diag, ceil, sqrt
from math import asin
import json
import re

t_seed, c_seed = (6, 44)
coef_friction = 0.1

def pitch(R, angle):
    p = 2*pi*R/tan(radians(abs(angle)))
    return p

def read_json(file):
    with open(file, "r") as f:
        layers = json.load(f)
    types = {
        "isotropic_cylinder": Isotropic_cylinder,
        "orthotropic_cylinder": Orthotropic_cylinder,
        "orthotropic_cylinder_csv": Orthotropic_cylinder_csv,
        "solid_isotropic_cylinder": Solid_isotropic_cylinder,
        "solid_helix": Solid_helix,
        "shell_helix": Shell_helix
    }
    constantes = {
        "ON": ON,
        "OFF": OFF
    }
    for layer in layers.values():
        layer["type"] = types[layer["type"]]
        if "revolve_direction" in layer:
            layer["revolve_direction"] = constantes[layer["revolve_direction"]]
    return layers

def read_stiffness_matrix(file):
    with open('risers/'+file + '.csv', 'r') as f:
        lines = f.readlines()

    matrix = []
    for line in lines[1:]:  # pula o cabeçalho
        matrix.append(list(map(float, line.strip().split(';'))))

    stiffnessMatrix = tuple(
        matrix[i][j]
        for i in range(6)
        for j in range(i + 1)
    )
    return stiffnessMatrix

class Model:
    def __init__(self, name, cases, inp_job = None):
        """
        Cria um modelo a partir da sua categoria. Tipos possíveis:
        b   - Modelo de flexão com step explícito
        bs  - Modelo de flexão com step estático e contato surface to surface
        sb  - Modelo de flexão com step estático
        tb  - Modelo de flexão com constraint tie
        t   - Modelo de tração com step explicíto
        st  - Modelo de tração com step estático
        tt  - Modelo de tração com constraint tie
        is  - Modelo com pressão externa inicial
        ip  - Modelo com pressão interna com step explícito
        ips - Modelo com pressão interna com step explícito e contato surface to surface
        ep  - Modelo com pressão external com step explícito
        f   - Modelo de frequência
        """
        t = {
            "s": (Settling, Explicit_contact),
            "ss": (Static_settling, Standard_contact),
            "b": (Bending, Explicit_contact),
            "bes": (Bending, Surf_to_surf_Explicit_contact),
            "bis": (Implicit_bending, Surf_to_surf_Static_contact),
            "bt": (Bending, Tied_contact),
            "bs": (Static_bending, Surf_to_surf_Static_contact),
            "sb": (Static_bending, Standard_contact),
            "tb": (Static_bending, Tied_contact),
            "t": (Tension, Explicit_contact),
            "st": (Static_tension, Standard_contact),
            "tt": (Static_tension, Tied_contact),
            "is": (Initial_state, None),
            "ip": (Internal_pressure, Explicit_contact),
            "ips": (Internal_pressure, Surf_to_surf_Explicit_contact),
            "ipt": (Internal_pressure, Tied_contact),
            "ep": (External_pressure, Explicit_contact),
            "eps": (External_pressure, Surf_to_surf_Explicit_contact),
            "f": (Frequency, Standard_contact)
        }
        self.name = name
        self.cases = [(t[case], params) for case, params in cases]
        self.inp_job = inp_job
    def create_model(self):
        if 'Model-1' not in mdb.models:
            mdb.Model(name='Model-1')
        if self.name in mdb.models:
            del mdb.models[self.name]
        mdb.Model(name=self.name)
        del mdb.models['Model-1']
        model = mdb.models[self.name]
        return model
    def define_model_length(self, number_pitches, model_length, layers):
        if helix_layers := [l for l in layers.values() if issubclass(l.get("type"), Helix)]:
            for l in helix_layers:
                l["pitch"] = pitch(l["mid_diameter"]/2, l["lay_angle"])
            proj = min(l["base"] / tan(radians(l["lay_angle"])) for l in helix_layers)
            model_length = ceil(number_pitches * min(abs(l["pitch"]) for l in helix_layers) / proj) * proj
            for layer in layers.values():
                layer["length"]=model_length
        else:
            for layer in layers.values():
                layer["length"]=model_length
        return (layers, model_length)
    def build_assembly(self, model, layers):
        self.objects = {}
        for layer in layers.values():
            cls = layer["type"]
            params = {k: v for k, v in layer.items() if k != "type"}
            obj = cls(**params)
            obj.build(model)
            self.objects[layer["name"]] = obj
    def set_colors(self, layers):
        colors = [
            "#a6cee3",
            "#1f78b4",
            "#b2df8a",
            "#33a02c",
            "#fb9a99",
            "#e31a1c",
            "#fdbf6f",
            "#ff7f00",
            "#cab2d6",
            "#6a3d9a",
            "#ffff99",
            "#b15928",
        ]

        vp = session.viewports['Viewport: 1']
        a = mdb.models[self.name].rootAssembly
        vp.setValues(displayedObject=a)
        vp.assemblyDisplay.setValues(optimizationTasks=OFF, geometricRestrictions=OFF, stopConditions=OFF)
        layer_colors = {
            layer: (True, colors[i % len(colors)], 'Default', colors[i % len(colors)])
            for i, layer in enumerate(layers.keys())
        }
        vp.enableMultipleColors()
        vp.setColor(initialColor='#BDBDBD')
        cmap = vp.colorMappings['Part']
        cmap.updateOverrides(overrides=layer_colors)
        vp.setColor(colorMapping=cmap)
    def create_analysis(self, model, layers, model_length):
        case_class, contact_class = self.cases[0][0]
        c = case_class(model, layers, model_length)
        ctc = contact_class(model, layers, self.objects) if contact_class is not None else None
        nameEndA, endA = c.create_coupling("endA", (0,0,0))
        nameEndB, endB = c.create_coupling("endB", (0,model_length,0))

        previous = 'Initial'
        ctc_done = False
        for (case_class, contact_class), param in self.cases:
            c = case_class(model, layers, model_length)
            if ctc is not None:
                c.build((nameEndA, endA), (nameEndB, endB), previous, ctc.ext, ctc.inte, **param)
                previous = c.step_name
                if not ctc_done:
                    ctc.build(c.step_name)
                    ctc_done=True
        if self.inp_job is not None:
            model.InitialState(updateReferenceConfiguration=OFF, fileName=self.inp_job, endStep=LAST_STEP, endIncrement=STEP_END, name='ContactState', createStepName='Initial', instances=tuple(model.rootAssembly.instances.values()))
    def build(self, number_pitches, model_length, layers):
        model = self.create_model()
        layers, model_length = self.define_model_length(number_pitches, model_length, layers)
        self.build_assembly(model, layers)
        # self.set_colors(layers)
        self.create_analysis(model, layers, model_length)
        job_name = f'Job-{self.name}'
        mdb.Job(name=job_name, model=self.name, type=ANALYSIS, explicitPrecision=DOUBLE, nodalOutputPrecision=SINGLE, numCpus=24, numDomains=24, multiprocessingMode=DEFAULT)
        return job_name

class Cylinder:
    def __init__(self, *, name, length, mid_diameter, thickness, name_material, young_modulus, poisson, density, **kwargs):
        self.name = name
        self.length = length
        self.mid_diameter = mid_diameter
        self.radius = mid_diameter/2
        self.thickness = thickness
        self.name_material = name_material
        self.young_modulus = young_modulus
        self.poisson = poisson
        self.density = density
    def create_geometry(self, model):
        """
        Cria a geometria da casca cilíndrica e retorna um objeto de parte do abaqus e um objeto de sketch.
        """
        print(f"\n================================ {self.name: ^24} ================================\n")
        sketch = model.ConstrainedSketch(name=f'sketch_{self.name}', sheetSize=200.0)
        sketch.CircleByCenterPerimeter(center=(0,0), point1=(0, self.radius))
        part = model.Part(name=self.name, dimensionality=THREE_D, type=DEFORMABLE_BODY)
        part.BaseShellExtrude(sketch=sketch, depth= self.length)
        part.setValues(geometryRefinement=EXTRA_FINE)
        print(f"Geometria da camada {self.name} criada.")
        return part, sketch
    def create_mesh(self, part):
        """
        Cria uma malha estruturada com elemento S4R com controle de hourglass enhanced.
        """
        part.seedEdgeByNumber(edges=part.edges, number=c_seed)
        part.DatumPlaneByPrincipalPlane(principalPlane=XZPLANE,offset=0)
        part.PartitionFaceByDatumPlane(faces=part.faces, datumPlane=part.datums[3])
        # part.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE,offset=0)
        # part.PartitionFaceByDatumPlane(faces=part.faces, datumPlane=part.datums[5])
        processed = set()
        for edge in part.edges:
            if edge.getSize() < pi * self.radius:
                pos = edge.pointOn[0][2]
                key = round(pos, 6)
                if key in processed:
                    continue
                processed.add(key)
                same_z_edges = [e for e in part.edges if e.getSize() < pi * self.radius and abs(e.pointOn[0][2] - pos) < 1e-6]
                if len(same_z_edges) >= 2:
                    part.MergeEdges(edgeList=[same_z_edges[0], same_z_edges[-1]], extendSelection=False)
        base, lay_angle = self.get_solid_helix_props(layers)
        if (base is not None) and (lay_angle is not None):
            self.ndl = int(self.length/((base)/sin(radians(lay_angle))))
            part.seedEdgeByNumber(edges=[a for a in part.edges if a.getSize()>2*pi*self.radius], number=self.ndl)
            # part.seedEdgeByBias(biasMethod=DOUBLE, endEdges=[a for a in part.edges if a.getSize()<2*pi*self.radius], ratio = 5, number=c_seed)
        else:
            part.seedEdgeByNumber(edges=[a for a in part.edges if a.getSize()>2*pi*self.radius], number=int(self.length*250))
        part.setMeshControls(regions=part.faces, technique=STRUCTURED)
        part.setElementType(regions=((part.faces,)), elemTypes=(mesh.ElemType(elemCode=S4R,elemLibrary=EXPLICIT,hourglassControl=ENHANCED),))
        part.generateMesh()
        print(f"Malha da camada {self.name} criada.")
    def create_material(self, model):
        """
        Cria um material elástico linear com módulo de elasticidade, poisson e densidade. A densidade é necessária
        para a análise explícita.
        """
        try:
            material = model.materials[self.name_material]
        except:
            material = model.Material(name=self.name_material)
            material.Elastic(table=((self.young_modulus, self.poisson),))
            material.Density(table=((self.density,),))
        print(f"Material {material.name} da camada {self.name} criado.")
    def create_assembly(self, model, part, sketch):
        """
        Cria a parte no assembly e posiciona rotacionando-a para corresponder ao eixo das armaduras de tração e
        apaga o sketch criado na etapa de criação da geometria.
        """
        assembly = model.rootAssembly
        assembly.Instance(name=self.name, part=part, dependent=ON)
        assembly.rotate(instanceList=[self.name], axisPoint = (0,0,0), axisDirection =(1, 0, 0), angle = -90)
        g = sketch.geometry
        for k in g.keys():
            sketch.delete((g[k],))
        print(f"Assembly da camada {self.name} criado.")
    def get_solid_helix_props(self, layers):
        """
        Acessa e retorna os valores da base da seção da armadura de tração e do seu ângulo de hélice para ajuste das
        dimensões da malha estruturada.
        """
        tension_layers = [layer for layer in layers.values() if issubclass(layer.get("type"), Helix)]
        if not tension_layers:
            print("Nenhuma layer válida do tipo 'solid_helix' encontrada.")
            base, lay_angle = None, None
            return base, lay_angle
        selected = min(tension_layers, key=lambda l: abs(l['base']))
        base = selected['base']
        lay_angle = abs(selected['lay_angle'])
        return base, lay_angle

class Helix:
    def __init__(self, *, name, pitch, mid_diameter, thickness, name_material, young_modulus, poisson, density, **kwargs):
        self.name = name
        self.pitch = pitch
        self.mid_diameter = mid_diameter
        self.radius = mid_diameter/2
        self.thickness = thickness
        self.name_material = name_material
        self.young_modulus = young_modulus
        self.poisson = poisson
        self.density = density
    def create_material(self, model):
        try:
            material = model.materials[self.name_material]
        except:
            material = model.Material(name=f'{self.name_material}')
            material.Elastic(table=((self.young_modulus, self.poisson),))
            material.Density(table=((self.density,),))

            print(f"Material {material.name} da camada {self.name} criado.")

class Isotropic_cylinder(Cylinder):
    """
    Representa uma camada de casca cilíndrica. Cria uma casca cilíndrica com malha estruturada e alinhada com as dimensões
    dos fios da armadura de tração. Possui material linear elástico. Ao incluir no assembly é girada para corresponder ao
    eixo da armadura de tração.

    name            -   Nome da parte                       (input)
    length          -   Comprimento do modelo em metros     (input)
    mid_diameter    -   Diâmetro médio em metros            (input)
    thickness       -   Espessura em metros                 (input)
    name_material   -   Nome do material                    (input)
    young_modulus   -   Módulo de elasticidade em Pa        (input)
    poisson         -   Coeficiente de poisson              (input)
    density         -   Densidade do material em kg/m³      (input)
    """ 
    def create_section(self, model, part):
        """
        Cria a seção de shell e associa ao material criado anteriormente. Importante atentar ao offsetType para a
        utilização da superfície média devido ao diâmetro médio utilizado na definição da geometria.
        """
        model.HomogeneousShellSection(name=f'Section_{self.name}', material=self.name_material, thickness=self.thickness)
        region = part.Set(faces=part.faces, name=f'Set-{self.name}')
        part.SectionAssignment(region=region, sectionName=f'Section_{self.name}', offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_SECTION)
    def build(self, model):
        """
        Constrói toda a camada.
        """
        part, sketch = self.create_geometry(model)
        self.create_mesh(part)
        self.create_material(model)
        self.create_section(model, part)
        self.create_assembly(model, part, sketch)

class Orthotropic_cylinder(Cylinder):
    """
    Representa uma camada de casca cilíndrica ortotrópica. Cria uma casca cilíndrica ortotrópica com malha estruturada e 
    alinhada com as dimensões dos fios da armadura de tração. Possui material linear elástico. Ao incluir no assembly é
    girada para corresponder ao eixo da armadura de tração.

    name            -   Nome da parte                       (input)
    length          -   Comprimento do modelo em metros     (input)
    mid_diameter    -   Diâmetro médio em metros            (input)
    thickness       -   Espessura em metros                 (input)
    name_material   -   Nome do material                    (input)
    young_modulus   -   Módulo de elasticidade em Pa        (input)
    poisson         -   Coeficiente de poisson              (input)
    density         -   Densidade do material em kg/m³      (input)
    n               -   Número de fios                      (input)
    Ix              -   Menor momento de inércia            (input)
    Iy              -   Maior momento de inércia            (input)
    A               -   Área da seção transversal           (input)
    J               -   Momento de inércia à torção         (input)
    lay_angle       -   Ângulo de hélice                    (input)
    """ 
    def __init__(self, *, n, Ix, Iy, A, J, lay_angle, **kwargs):
        super().__init__(**kwargs)
        self.n = n
        self.Ix = Ix
        self.Iy = Iy
        self.A = A
        self.J = J
        self.lay_angle = lay_angle
    def create_mesh(self, part):
        """
        Cria uma malha estruturada com elemento S4R com controle de hourglass enhanced.
        """
        part.seedEdgeByNumber(edges=part.edges, number=c_seed)
        part.DatumPlaneByPrincipalPlane(principalPlane=XZPLANE,offset=0)
        part.PartitionFaceByDatumPlane(faces=part.faces, datumPlane=part.datums[3])
        # part.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE,offset=0)
        # part.PartitionFaceByDatumPlane(faces=part.faces, datumPlane=part.datums[5])
        # processed = set()
        # for edge in part.edges:
        #     if edge.getSize() < pi * self.radius:
        #         pos = edge.pointOn[0][2]
        #         key = round(pos, 6)
        #         if key in processed:
        #             continue
        #         processed.add(key)
        #         same_z_edges = [e for e in part.edges if e.getSize() < pi * self.radius and abs(e.pointOn[0][2] - pos) < 1e-6]
        #         if len(same_z_edges) >= 2:
        #             part.MergeEdges(edgeList=[same_z_edges[0], same_z_edges[-1]], extendSelection=False)
        base, lay_angle = self.get_solid_helix_props(layers)
        if (base is not None) and (lay_angle is not None):
            self.ndl = int(self.length/((base)/sin(radians(lay_angle))))
            part.seedEdgeByNumber(edges=[a for a in part.edges if a.getSize()>2*pi*self.radius], number=self.ndl)
            # part.seedEdgeByBias(biasMethod=DOUBLE, endEdges=[a for a in part.edges if a.getSize()<2*pi*self.radius], ratio = 5, number=c_seed)
        else:
            part.seedEdgeByNumber(edges=[a for a in part.edges if a.getSize()>2*pi*self.radius], number=int(self.length*250))
        part.setMeshControls(regions=part.faces, technique=STRUCTURED)
        part.setElementType(regions=((part.faces,)), elemTypes=(mesh.ElemType(elemCode=S4R,elemLibrary=EXPLICIT,hourglassControl=ENHANCED),))
        part.generateMesh()
        print(f"Malha da camada {self.name} criada.")
    def create_section(self, model, part):
        """
        Cria uma seção com material ortotrópico por meio de uma analogia com compósitos a partir dos
        momentos de inércia, módulo de elasticidade, pitch e número de fios. Além disso, rotaciona a
        orientação do material para que corresponda à realidade da rigidez da camada ortotrópica. Uma
        espessura é atribuída 'manualmente' para o devido funcionamento do contato, já que com a matriz
        ABD não há definição explícita da espessura da camada.
        """
        passo = pitch(self.radius, self.lay_angle)
        a = radians(self.lay_angle)
        c, s = cos(a), sin(a)
        K = 1

        EAeq = self.n*self.young_modulus*self.A/(abs(passo*s))
        I2 = (self.Ix+self.Iy)/2-(((self.Iy-self.Ix)**2+4*self.J**2)**(1/2))/2
        I2 = self.Ix if I2<0 else I2
        EIeq = K*self.n*self.young_modulus*I2/(abs(passo*s))
        G = self.young_modulus/(2*(1+self.poisson))
        GJeq = self.n*G*self.J/(abs(passo*s))

        eps=1E-3
        T = array([
                [ c**2,    s**2,   c*s],
                [ s**2,    c**2,  -c*s],
                [-2*c*s,  2*c*s,  c**2 - s**2]
                ])
        
        A12 = diag([EAeq, eps * EAeq, eps * EAeq])
        D12 = diag([EIeq, eps * EIeq, GJeq])

        Axy = T.T @ A12 @ T
        Dxy = T.T @ D12 @ T

        D11, D12_, D13 = Axy[0, 0], Axy[0, 1], Axy[0, 2]
        D22, D23       = Axy[1, 1], Axy[1, 2]
        D33            = Axy[2, 2]

        D44, D45, D46 = Dxy[0, 0], Dxy[0, 1], Dxy[0, 2]
        D55, D56       = Dxy[1, 1], Dxy[1, 2]
        D66            = Dxy[2, 2]
        
        D14 = D15 = D16 = D24 = D25 = D26 = D34 = D35 = D36 = 0
        stiffnessMatrix = (
                            D11,  
                            D12_, D22,
                            D13,  D23,  D33,
                            D14,  D24,  D34,  D44,
                            D15,  D25,  D35,  D45, D55,
                            D16,  D26, D36,  D46,  D56, D66
                        )

        model.GeneralStiffnessSection(
                name              = f'Section_{self.name}',
                stiffnessMatrix   = stiffnessMatrix,
                useDensity        = ON,
                density           = self.density*self.thickness,
        )

        region = part.Set(faces=part.faces, name=f'Set-{self.name}')
        part.SectionAssignment(region=region, sectionName=f'Section_{self.name}', offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_GEOMETRY)
        part.MaterialOrientation(
            region           = region,
            orientationType  = SYSTEM,
            localCsys        = None,
            axis             = AXIS_2,
            angle            = 0.0,
            stackDirection   = STACK_3,
            fieldName        = '',
            additionalRotationType = ROTATION_NONE,
        )
        part.assignThickness(faces=part.faces, thickness=self.thickness)
        print(f"Seção de matriz ABD da camada {self.name} criada.")
    def build(self, model):
        """
        Constrói toda a camada.
        """
        part, sketch = self.create_geometry(model)
        self.create_mesh(part)
        self.create_section(model, part)
        self.create_assembly(model, part, sketch)

class Orthotropic_cylinder_csv(Orthotropic_cylinder):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
    def create_section(self, model, part):
        stiffnessMatrix = read_stiffness_matrix(self.name)

        model.GeneralStiffnessSection(
                name              = f'Section_{self.name}',
                stiffnessMatrix   = stiffnessMatrix,
                useDensity        = ON,
                density           = self.density*self.thickness,
        )

        region = part.Set(faces=part.faces, name=f'Set-{self.name}')
        part.SectionAssignment(region=region, sectionName=f'Section_{self.name}', offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_GEOMETRY)
        # part.MaterialOrientation(
        #     region           = region,
        #     orientationType  = SYSTEM,
        #     localCsys        = None,
        #     axis             = AXIS_2,
        #     angle            = 0.0,
        #     stackDirection   = STACK_3,
        #     fieldName        = '',
        #     additionalRotationType = ROTATION_NONE,
        # )
        part.assignThickness(faces=part.faces, thickness=self.thickness)
        print(f"Seção de matriz ABD da camada {self.name} criada.")

class Solid_isotropic_cylinder(Cylinder):
    """
    Representa uma camada de cilindro sólido. Cria um cilindro sólido com malha estruturada e alinhada com as dimensões
    dos fios da armadura de tração. Possui material linear elástico. Ao incluir no assembly é girada para corresponder ao
    eixo da armadura de tração.

    name            -   Nome da parte                       (input)
    pitch           -   Comprimento do pitch em metros      (input)
    mid_diameter    -   Diâmetro médio em metros            (input)
    thickness       -   Espessura em metros                 (input)
    name_material   -   Nome do material                    (input)
    young_modulus   -   Módulo de elasticidade em Pa        (input)
    poisson         -   Coeficiente de poisson              (input)
    density         -   Densidade do material em kg/m³      (input)
    """ 
    def create_geometry(self, model):
        """
        Cria a geometria do cilindro sólido e retorna um objeto de parte do abaqus e um objeto de sketch.
        """
        print(f"\n================================ {self.name: ^24} ================================\n")
        sketch = model.ConstrainedSketch(name='rect', sheetSize=200.0)
        sketch.CircleByCenterPerimeter(center=(0,0), point1=(0, self.radius-self.thickness/2))
        sketch.CircleByCenterPerimeter(center=(0,0), point1=(0, self.radius+self.thickness/2))
        part = model.Part(name=self.name, dimensionality=THREE_D, type=DEFORMABLE_BODY)
        part.BaseSolidExtrude(sketch=sketch, depth= self.length)
        part.setValues(geometryRefinement=EXTRA_FINE)
        print(f"Geometria da camada {self.name} criada.")
        return part, sketch
    def create_mesh(self, part):
        """
        Cria uma malha estruturada com elemento C3D8I.
        """
        part.seedEdgeByNumber(edges=part.edges, number=c_seed)
        part.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE,offset=0)
        part.PartitionCellByDatumPlane(cells=part.cells, datumPlane=part.datums[3])
        base, lay_angle = self.get_solid_helix_props(layers)
        if (base is not None) and (lay_angle is not None):
            part.seedEdgeByNumber(edges=[a for a in part.edges if a.getSize()>2*pi*self.radius], number=int(self.length/((base)/sin(radians(lay_angle)))))
        else:
            part.seedEdgeByNumber(edges=[a for a in part.edges if a.getSize()>2*pi*self.radius], number=int(self.length*100))
        part.setMeshControls(regions=part.cells, technique=STRUCTURED)
        part.setElementType(regions=((part.cells,)), elemTypes=(mesh.ElemType(elemCode=C3D8I,),))
        part.generateMesh()
        print(f"Malha da camada {self.name} criada.")
    def create_section(self, model, part):
        """
        Cria a seção sólida e associa ao material criado anteriormente. Importante atentar ao offsetType para a
        utilização da superfície média devido ao diâmetro médio utilizado na definição da geometria.
        """
        model.HomogeneousSolidSection(name=f'Section_{self.name}', material=self.name_material)
        region = part.Set(cells=part.cells, name=f'Set-{self.name}')
        part.SectionAssignment(region=region, sectionName=f'Section_{self.name}', offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_SECTION)
    def build(self, model):
        """
        Constrói toda a camada.
        """
        part, sketch = self.create_geometry(model)
        self.create_mesh(part)
        self.create_material(model)
        self.create_section(model, part)
        self.create_assembly(model, part, sketch)

class Solid_helix(Helix):
    """
    Representa uma camada de armadura de tração sólida. Cria uma camada de armadura de tração com malha estruturada.
    Possui material linear elástico. Ao incluir no assembly é multiplicada para uma quantidade n de fios.

    name                -   Nome da parte                                   (input)
    pitch               -   Comprimento do pitch em metros                  (input)
    mid_diameter        -   Diâmetro médio em metros                        (input)
    thickness           -   Espessura em metros                             (input)
    name_material       -   Nome do material                                (input)
    young_modulus       -   Módulo de elasticidade em Pa                    (input)
    poisson             -   Coeficiente de poisson                          (input)
    density             -   Densidade do material em kg/m³                  (input)
    lay_angle           -   Ângulo de hélice                                (input)
    n                   -   Número de fios                                  (input)
    base                -   Comprimento da base da seção transversal        (input)
    revolve_direction   -   Direção de giro da hélice                       (input)
    length              -   Comprimento total do modelo                     (input)
    """ 
    def __init__(self, *, lay_angle, n, base, revolve_direction, length = 0, **kwargs):
        super().__init__(**kwargs)
        self.lay_angle = abs(lay_angle)
        self.n = n
        self.base = base
        self.revolve_direction = revolve_direction
        self.length = length
    def create_geometry(self, model):
        """
        Cria a geometria do fio helicoidal com um comprimento extra (10% maior), realiza um corte no plano transversal ao pitch
        a fim de manter as armaduras no plano das seções das extremidades e ajusta as partições automáticas da hélice para permitir
        a criação de uma malha estruturada. Retorna um objeto de parte do abaqus e um objeto de sketch.
        """
        print(f"\n================================ {self.name: ^24} ================================\n")
        sketch = model.ConstrainedSketch(name=f'sketch_{self.name}', sheetSize=200.0)
        sketch.rectangle(point1=(self.radius-self.thickness/2, 0), point2=(self.radius+self.thickness/2, self.base/sin(radians(self.lay_angle))))
        sketch.ConstructionLine(point1=(0.0, -self.length), point2=(0.0, self.length))
        p = model.Part(name=self.name, dimensionality=THREE_D, type=DEFORMABLE_BODY)
        extend_factor = 1 + (self.base/sin(radians(self.lay_angle)))/self.length
        p.BaseSolidRevolve(sketch=sketch, angle=extend_factor*self.length/self.pitch*360.0, flipRevolveDirection=self.revolve_direction, pitch=self.pitch, flipPitchDirection = OFF, moveSketchNormalToPath=OFF)
        p.setValues(geometryRefinement=EXTRA_FINE)
        g = sketch.geometry
        # for k in g.keys():
        #     sketch.delete((g[k],))
        
        # Apaga o excesso da armadura de tração
        # planes = [(extend_factor-1)/2, (extend_factor+1)/2]
        planes = [0, 1]
        extrude_dir = [ON, OFF]
        for i, (plan, dir) in enumerate(list(zip(planes, extrude_dir))):
            p.DatumPlaneByPrincipalPlane(principalPlane=XZPLANE, offset = -plan*self.length)
            datums = p.datums.values()
            ns = model.ConstrainedSketch(gridSpacing=63.45, name='novo_sketch', sheetSize=2600, transform=p.MakeSketchTransform(sketchPlane=datums[i+1], sketchPlaneSide=SIDE1, sketchUpEdge=p.edges[6], sketchOrientation=RIGHT, origin=(0.0, self.length, 0.0)))
            ns.rectangle(point1=(-200, 200), point2=(200, -200))
            
            p.CutExtrude(flipExtrudeDirection=dir, sketch=ns, sketchOrientation=RIGHT, sketchPlane=datums[i+1], sketchPlaneSide=SIDE1, sketchUpEdge=p.edges[6])
            del ns
        
        # Ajusta a partição automática do Abaqus para possibilitar uma malha estruturada
        comprimento_minimo = 2*self.base
        sorted_edges = sorted(p.edges, key=lambda e: e.pointOn[0][1])
        indices_edges_manter = set(
            [p.edges.index(e) for e in sorted_edges[:4]] +
            [p.edges.index(e) for e in sorted_edges[-4:]]
        )
        for i, e in enumerate(p.edges):
            if e.getSize() >= comprimento_minimo:
                indices_edges_manter.add(i)
        edges_ignorar = [e for i, e in enumerate(p.edges) if i not in indices_edges_manter]
        sorted_verts = sorted(p.vertices, key=lambda v: v.pointOn[0][1])
        indices_verts_manter = set(
            [p.vertices.index(v) for v in sorted_verts[:4]] +
            [p.vertices.index(v) for v in sorted_verts[-4:]]
        )
        verts_ignorar = [v for i, v in enumerate(p.vertices) if i not in indices_verts_manter]
        p.ignoreEntity(entities=(edges_ignorar,verts_ignorar))

        print(f"Geometria da camada {self.name} criada.")
        return p, extend_factor, sketch, comprimento_minimo
    def create_mesh(self, p, comprimento_minimo):
        """
        Cria uma malha estruturada com elemento C3D8I.
        """
        p.seedPart(size=t_seed)
        p.setMeshControls(regions=p.cells, technique=STRUCTURED)
        # p.seedEdgeByNumber(edges=[a for a in p.edges if a.getSize()<comprimento_minimo], number=1)
        p.seedEdgeByNumber(edges=[a for a in p.edges if a.getSize()>comprimento_minimo], number=int(self.length/(self.base/sin(radians(self.lay_angle)))))
        p.setElementType(regions=(p.cells[0],), elemTypes=(mesh.ElemType(elemCode=C3D8I, hourglassControl=ENHANCED),))
        p.generateMesh()
        print(f"Malha da camada {self.name} criada.")
    def create_section(self, model, p):
        # Cria seção e associa material
        model.HomogeneousSolidSection(name=f'Section_{self.name_material}', material=self.name_material, thickness=None)
        region = p.Set(cells=p.cells, name=f'Set-{self.name}')
        p.SectionAssignment(region=region, sectionName=f'Section_{self.name_material}', offset=0.0, offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_SECTION)
    def create_assembly(self, model, p, extend_factor, sketch):
        # Cria o assembly e as cópias
        assembly = model.rootAssembly
        instance_name = f'{self.name}-1'
        assembly.Instance(name=instance_name, part=p, dependent=ON)
        assembly.translate(instanceList=(instance_name,), vector=(0,self.length,0))
        num_instances = self.n
        angle_step = 360.0 / num_instances

        for j in range(0, num_instances):
            rotation_angle = j * angle_step
            instance_rotated_name = f'{self.name}-{j+1}'
            assembly.Instance(name=instance_rotated_name, part=p, dependent=ON)
            assembly.rotate(instanceList=(instance_rotated_name,), axisPoint=(0, 0, 0), axisDirection=(0, 1, 0), angle=rotation_angle)
            assembly.translate(instanceList=(instance_rotated_name,), vector=(0,self.length,0))
            # assembly.translate(instanceList=(instance_rotated_name,), vector=(0,(extend_factor+1)/2*self.length,0))
                
        # g = sketch.geometry
        # for k in g.keys():
        #     sketch.delete((g[k],))

        # assembly.suppressFeatures((instance_name,))
        print(f"Assembly da camada {self.name} criado.") 
    def build(self, model):
        """
        Constrói toda a camada.
        """
        p, extend_factor, sketch, comprimento_minimo = self.create_geometry(model)
        self.create_mesh(p, comprimento_minimo)
        self.create_material(model)
        self.create_section(model, p)
        self.create_assembly(model, p, extend_factor, sketch)

class Shell_helix(Helix):
    """
    Representa uma camada de armadura de tração sólida. Cria uma camada de armadura de tração com malha estruturada.
    Possui material linear elástico. Ao incluir no assembly é multiplicada para uma quantidade n de fios.

    name                -   Nome da parte                                   (input)
    pitch               -   Comprimento do pitch em metros                  (input)
    mid_diameter        -   Diâmetro médio em metros                        (input)
    thickness           -   Espessura em metros                             (input)
    name_material       -   Nome do material                                (input)
    young_modulus       -   Módulo de elasticidade em Pa                    (input)
    poisson             -   Coeficiente de poisson                          (input)
    density             -   Densidade do material em kg/m³                  (input)
    lay_angle           -   Ângulo de hélice                                (input)
    n                   -   Número de fios                                  (input)
    base                -   Comprimento da base da seção transversal        (input)
    revolve_direction   -   Direção de giro da hélice                       (input)
    length              -   Comprimento total do modelo                     (input)
    """ 
    def __init__(self, *, lay_angle, n, base, revolve_direction, length = 0, **kwargs):
        super().__init__(**kwargs)
        self.lay_angle = abs(lay_angle)
        self.n = n
        self.base = base
        self.revolve_direction = revolve_direction
        self.length = length
    def create_geometry(self, model):
        print(f"\n================================ {self.name: ^24} ================================\n")
        sketch = model.ConstrainedSketch(name=f'sketch_{self.name}', sheetSize=200.0)
        sketch.Line(
            point1=(self.radius, 0),
            point2=(self.radius, self.base / sin(radians(self.lay_angle)))
        )
        sketch.ConstructionLine(point1=(0.0, -self.length), point2=(0.0, self.length))

        p = model.Part(name=self.name, dimensionality=THREE_D, type=DEFORMABLE_BODY)
        extend_factor = 1.1
        p.BaseShellRevolve(
            sketch=sketch,
            angle=extend_factor * self.length / self.pitch * 360.0,
            flipRevolveDirection=self.revolve_direction,
            pitch=self.pitch,
            flipPitchDirection=OFF,
            moveSketchNormalToPath=OFF
        )
        p.setValues(geometryRefinement=EXTRA_FINE)

        # Apaga o excesso da armadura de tração
        planes = [(extend_factor-1)/2, (extend_factor+1)/2]
        extrude_dir = [ON, OFF]
        for i, (plan, dir) in enumerate(list(zip(planes, extrude_dir))):
            p.DatumPlaneByPrincipalPlane(principalPlane=XZPLANE, offset = -plan*self.length)
            datums = p.datums.values()
            ns = model.ConstrainedSketch(gridSpacing=63.45, name='novo_sketch', sheetSize=2600, transform=p.MakeSketchTransform(sketchPlane=datums[i+1], sketchPlaneSide=SIDE1, sketchUpEdge=p.edges[8], sketchOrientation=RIGHT, origin=(0.0, self.length, 0.0)))
            ns.rectangle(point1=(-200, 200), point2=(200, -200))
            p.CutExtrude(flipExtrudeDirection=dir, sketch=ns, sketchOrientation=RIGHT, sketchPlane=datums[i+1], sketchPlaneSide=SIDE1, sketchUpEdge=p.edges[8])
            del ns
        
        # # Ajusta a partição automática do Abaqus para possibilitar uma malha estruturada
        comprimento_minimo = 2*self.base
        # sorted_edges = sorted(p.edges, key=lambda e: e.pointOn[0][1])
        # indices_edges_manter = set(
        #     [p.edges.index(e) for e in sorted_edges[:4]] +
        #     [p.edges.index(e) for e in sorted_edges[-4:]]
        # )
        # for i, e in enumerate(p.edges):
        #     if e.getSize() >= comprimento_minimo:
        #         indices_edges_manter.add(i)
        # edges_ignorar = [e for i, e in enumerate(p.edges) if i not in indices_edges_manter]
        # sorted_verts = sorted(p.vertices, key=lambda v: v.pointOn[0][1])
        # indices_verts_manter = set(
        #     [p.vertices.index(v) for v in sorted_verts[:4]] +
        #     [p.vertices.index(v) for v in sorted_verts[-4:]]
        # )
        # verts_ignorar = [v for i, v in enumerate(p.vertices) if i not in indices_verts_manter]
        # p.ignoreEntity(entities=(edges_ignorar,verts_ignorar))

        # print(f"Geometria da camada {self.name} criada.")
        return p, extend_factor, sketch, comprimento_minimo
    def create_mesh(self, p, comprimento_minimo):
        """
        Cria uma malha estruturada com elemento C3D8I.
        """
        p.seedPart(size=int(self.length))
        p.setMeshControls(regions=p.faces, technique=STRUCTURED)
        # p.seedEdgeByNumber(edges=[a for a in p.edges if a.getSize()>comprimento_minimo], number=int(self.length/(self.base/sin(radians(self.lay_angle))))) # Conferir generalidade
        p.setElementType(regions=(p.faces,), elemTypes=(mesh.ElemType(elemCode=S4R, hourglassControl=ENHANCED),))
        p.generateMesh()
        print(f"Malha da camada {self.name} criada.")
    def create_section(self, model, p):
        # Cria seção e associa material
        model.HomogeneousShellSection(name=f'Section_{self.name}', material=self.name_material, thickness=self.thickness)
        region = p.Set(faces=p.faces, name=f'Set-{self.name}')
        p.SectionAssignment(region=region, sectionName=f'Section_{self.name}', offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_SECTION)
    def create_assembly(self, model, p, extend_factor, sketch):
        # Cria o assembly e as cópias
        assembly = model.rootAssembly
        instance_name = f'{self.name}-1'
        assembly.Instance(name=instance_name, part=p, dependent=ON)
        assembly.translate(instanceList=(instance_name,), vector=(0,self.length,0))
        num_instances = self.n
        angle_step = 360.0 / num_instances

        for j in range(0, num_instances):
            rotation_angle = j * angle_step
            instance_rotated_name = f'{self.name}-{j+1}'
            assembly.Instance(name=instance_rotated_name, part=p, dependent=ON)
            assembly.rotate(instanceList=(instance_rotated_name,), axisPoint=(0, 0, 0), axisDirection=(0, 1, 0), angle=rotation_angle)
            assembly.translate(instanceList=(instance_rotated_name,), vector=(0,(extend_factor+1)/2*self.length,0))
                
        # g = sketch.geometry
        # for k in g.keys():
        #     sketch.delete((g[k],))

        # assembly.suppressFeatures((instance_name,))
        print(f"Assembly da camada {self.name} criado.") 
    def build(self, model):
        """
        Constrói toda a camada.
        """
        p, extend_factor, sketch, comprimento_minimo = self.create_geometry(model)
        self.create_mesh(p, comprimento_minimo)
        self.create_material(model)
        self.create_section(model, p)
        self.create_assembly(model, p, extend_factor, sketch)

class Analysis:
    def __init__(self, model, layers, model_length):
        self.model = model
        self.assembly = model.rootAssembly
        self.layers = layers
        self.model_length = model_length
    def create_set(self, y):
        selected_vertices, selected_edges = list(), list()
        for key, values in self.layers.items():
            inf_length = values['length']/10000
            for instance_name in (x for x in self.assembly.instances.keys() if key.lower() in x.lower()):
                instance = self.assembly.instances[instance_name]
                if issubclass(values['type'], Solid_helix):
                    vertices = instance.vertices
                    selected_vertices.append(vertices.getByBoundingBox(yMin=y-inf_length, yMax=y+inf_length))
                elif issubclass(values['type'], Cylinder):
                    selected_edges.append(instance.edges.getByBoundingBox(yMin=y-inf_length, yMax=y+inf_length))
        return self.assembly.Set(vertices=selected_vertices, edges=selected_edges, name=f'RegiaoY-{abs(int(1000*y))}')
    def create_coupling(self, name, point):
        end = self.create_set(point[1])
        rp = self.assembly.ReferencePoint(point=point)
        ref = self.assembly.referencePoints[rp.id]
        rg_name = f'Regiao-{name}'
        regiao = self.assembly.Set(referencePoints=(ref,), name=rg_name)
        self.model.Coupling(name=f'Coupling-{name}', surface=end, controlPoint=regiao, influenceRadius=WHOLE_SURFACE, couplingType=KINEMATIC)#, ur1 = OFF, ur2 = OFF, ur3 = OFF)
        return (rg_name, regiao)
    def create_amplitude(self, name, data, smooth=True):
        if smooth:
            self.model.SmoothStepAmplitude(name=name, data=data)
        else:
            self.model.TabularAmplitude(name=name, data=data)
        return name
    def define_stress(self, pressure, external = True):
        if external:
            mid_diameter, thickness = (list(self.layers.values())[-1]["mid_diameter"], list(self.layers.values())[-1]["thickness"])
            radius = (mid_diameter+thickness)/2
            tension = (pressure * radius) / thickness
        else:
            mid_diameter, thickness = (list(self.layers.values())[1]["mid_diameter"], list(self.layers.values())[1]["thickness"])
            radius = (mid_diameter-thickness)/2
            tension = (pressure * radius) / thickness
        return tension if external else -tension
        
class Bending(Analysis):
    def build(self, endA, endB, previous, ext, inte, curvature, timePeriod, hysteresis = False):
        self.step_name = "Step_Bending"
        nameEndA, regionEndA = endA
        nameEndB, regionEndB = endB

        rotation = self.model_length * curvature
        self.model.ExplicitDynamicsStep(name= self.step_name, previous=previous, timePeriod=timePeriod)
        try:
            self.model.boundaryConditions['endA']
            self.model.boundaryConditions['endB']
        except:
            self.model.DisplacementBC(name="endA", createStepName= self.step_name, region=regionEndA, u1=SET, u2=SET, u3=SET, ur1=SET, ur2=SET)
            self.model.DisplacementBC(name="endB", createStepName= self.step_name, region=regionEndB, u1=SET, u3=SET, ur1=SET, ur2=SET)
        # self.model.steps[self.step_name].Restart(numberIntervals=4, overlay = True)
        # for bc in self.model.boundaryConditions.keys():
        #     self.model.boundaryConditions[bc].deactivate(stepName=self.step_name)
        amplitude = self.create_amplitude('Amplitude', ((0,0),(timePeriod/4,1),(timePeriod/4*3,-1),(timePeriod,0)))if hysteresis else self.create_amplitude('Amplitude', ((0,0),(timePeriod,1),))
        self.model.DisplacementBC(name="endA_Bending", createStepName= self.step_name, region=regionEndA, ur3=-rotation/2, amplitude=amplitude)
        self.model.DisplacementBC(name="endB_Bending", createStepName= self.step_name, region=regionEndB, ur3=rotation/2, amplitude=amplitude)

        # for name, surf in self.assembly.surfaces.items():
        #     self.model.HistoryOutputRequest(name=f'H-Output-{name[-20:]}', createStepName=self.step_name, variables=('CAREA',), region=self.assembly.surfaces[name])
        #     self.model.historyOutputRequests[f'H-Output-{name[-20:]}'].setValues(numIntervals=200,)

        self.model.FieldOutputRequest(name='F-Output-EndA', createStepName=self.step_name, variables=('RM', 'UR'), region=self.assembly.sets[nameEndA])
        self.model.FieldOutputRequest(name='F-Output-EndB', createStepName=self.step_name, variables=('RM', 'UR'), region=self.assembly.sets[nameEndB])
        self.model.FieldOutputRequest(name='F-Output-Bending', createStepName=self.step_name, variables=('S','LE','U','V','A','RF','CSTRESS','CDISP','CSTATUS', 'EMSF', 'ENER', 'ELEN'))
        self.model.HistoryOutputRequest(name='H-Output-Bending', createStepName=self.step_name, variables=('ALLEN',))
        # self.model.HistoryOutputRequest(name='H-Output-Bending', createStepName=self.step_name, variables=('ALLAE', 'ALLCD', 'ALLDMD', 'ALLFD', 'ALLIE', 'ALLKE', 'ALLPD', 'ALLSE', 'ALLVD', 'ALLWK', 'ETOTAL', 'DMASS'))
        self.model.fieldOutputRequests['F-Output-EndA'].setValues(numIntervals=400,)
        self.model.fieldOutputRequests['F-Output-EndB'].setValues(numIntervals=400,)
        self.model.fieldOutputRequests['F-Output-Bending'].setValues(numIntervals=50,)

class Implicit_bending(Analysis):
    def build(self, endA, endB, previous, ext, inte, curvature, timePeriod, hysteresis = False):
        self.step_name = "Step_Explict"
        nameEndA, regionEndA = endA
        nameEndB, regionEndB = endB

        rotation = self.model_length * curvature
        self.model.ImplicitDynamicsStep(name= self.step_name, previous=previous, timePeriod=timePeriod)
        amplitude = self.create_amplitude('Amplitude', ((0,0),(timePeriod,1),(3*timePeriod,-1),(4*timePeriod,0)))if hysteresis else self.create_amplitude('Amplitude', ((0,0),(timePeriod,1),))
        self.model.DisplacementBC(name="endA_Bending", createStepName= self.step_name, region=regionEndA, ur3=-rotation/2, amplitude=amplitude)
        self.model.DisplacementBC(name="endB_Bending", createStepName= self.step_name, region=regionEndB, ur3=rotation/2, amplitude=amplitude)

        self.model.FieldOutputRequest(name='F-Output-EndA', createStepName=self.step_name, variables=('RM', 'UR'), region=self.assembly.sets[nameEndA])
        self.model.FieldOutputRequest(name='F-Output-EndB', createStepName=self.step_name, variables=('RM', 'UR'), region=self.assembly.sets[nameEndB])
        # self.model.FieldOutputRequest(name='F-Output-Bending', createStepName=self.step_name, variables=('S','LE','U','V','A','RF','CSTRESS','CDISP','CSTATUS', 'EMSF'))
        # self.model.HistoryOutputRequest(name='H-Output-Bending', createStepName=self.step_name, variables=('ALLAE', 'ALLCD', 'ALLDMD', 'ALLFD', 'ALLIE', 'ALLKE', 'ALLPD', 'ALLSE', 'ALLVD', 'ALLWK', 'ETOTAL', 'DMASS'))
        self.model.fieldOutputRequests['F-Output-EndA'].setValues(numIntervals=400,)
        self.model.fieldOutputRequests['F-Output-EndB'].setValues(numIntervals=400,)
        # self.model.fieldOutputRequests['F-Output-Bending'].setValues(numIntervals=50,)

class Static_bending(Analysis):
    def build(self, endA, endB, previous, ext, inte, curvature, timePeriod, hysteresis = False):
        self.step_name = "Step_Static"
        nameEndA, regionEndA = endA
        nameEndB, regionEndB = endB

        rotation = self.model_length * curvature
        self.model.StaticStep(name= self.step_name, previous=previous, timePeriod=timePeriod, maxNumInc=1000, initialInc=0.1, minInc=1E-10, maxInc=1)
        amplitude = self.create_amplitude('Amplitude', ((0,0),(timePeriod,1),(3*timePeriod,-1),(4*timePeriod,0)))if hysteresis else self.create_amplitude('Amplitude', ((0,0),(timePeriod,1),))
        self.model.DisplacementBC(name="endA_Static-Bending", createStepName= self.step_name, region=regionEndA, ur3=-rotation/2, amplitude=amplitude)
        self.model.DisplacementBC(name="endB_Static-Bending", createStepName= self.step_name, region=regionEndB, ur3=rotation/2, amplitude=amplitude)

        self.model.FieldOutputRequest(name='F-Output-Static-BendingEndA', createStepName=self.step_name, variables=('RM', 'UR'), region=self.assembly.sets[nameEndA])
        self.model.FieldOutputRequest(name='F-Output-Static-BendingEndB', createStepName=self.step_name, variables=('RM', 'UR'), region=self.assembly.sets[nameEndB])
        self.model.FieldOutputRequest(name='F-Output-Static-Bending', createStepName=self.step_name)
        self.model.HistoryOutputRequest(name='H-Output-Static-Bending', createStepName=self.step_name)
        self.model.fieldOutputRequests['F-Output-Static-BendingEndA'].setValues(numIntervals=400,)
        self.model.fieldOutputRequests['F-Output-Static-BendingEndB'].setValues(numIntervals=400,)
        self.model.fieldOutputRequests['F-Output-Static-Bending'].setValues(numIntervals=50,)

class Frequency(Analysis):
    def build(self, endA, endB, previous, ext, inte):
        self.step_name='Step_Frequency'
        nameEndA, regionEndA = endA
        nameEndB, regionEndB = endB
        self.model.FrequencyStep(name=self.step_name, previous = previous, eigensolver = LANCZOS, numEigen = 2)
        self.model.DisplacementBC(name="endA", createStepName= self.step_name, region=regionEndA, u1=SET, u2=SET, u3=SET, ur1=SET, ur2=SET)
        self.model.DisplacementBC(name="endB", createStepName= self.step_name, region=regionEndB, u1=SET, u3=SET, ur1=SET, ur2=SET)

class Initial_state(Analysis):
    def build(self, endA, endB, previous, ext, inte, pressure):
        self.step_name = None
        stress = self.define_stress(pressure=pressure)
        external_layer = self.assembly.instances[next(reversed(self.assembly.instances.keys()))]
        external_region = external_layer.sets[external_layer.sets.keys()[0]]
        self.model.Stress(name='Tensao externa', region=external_region, distributionType=UNIFORM, sigma11=stress, sigma22=0.0, sigma12=0.0, sigma33=None, sigma13=None, sigma23=None)

class External_pressure(Analysis):
    def build(self, endA, endB, previous, ext, inte, pressure, timePeriod):
        self.step_name = "Step_External_Pressure"
        nameEndA, regionEndA = endA
        nameEndB, regionEndB = endB

        self.model.ExplicitDynamicsStep(name= self.step_name, previous=previous, timePeriod=timePeriod)
        try:
            self.model.boundaryConditions['endA']
            self.model.boundaryConditions['endB']
        except:
            self.model.DisplacementBC(name="endA", createStepName= self.step_name, region=regionEndA, u1=SET, u2=SET, u3=SET, ur1=SET, ur2=SET)
            self.model.DisplacementBC(name="endB", createStepName= self.step_name, region=regionEndB, u1=SET, u3=SET, ur1=SET, ur2=SET)
        external_region = self.assembly.surfaces[ext[-1][0]]

        amplitude = self.create_amplitude('Amplitude_external_pressure', ((0,0),(timePeriod,1),))

        self.model.Pressure(name='External Pressure', createStepName=self.step_name, region = external_region, magnitude = pressure, amplitude=amplitude)

        self.model.FieldOutputRequest(name='F-Output-External_Pressure', createStepName=self.step_name, variables=('S','LE','U','V','A','RF','CSTRESS','CDISP','CSTATUS', 'EMSF', 'ENER', 'ELEN'))
        self.model.HistoryOutputRequest(name='H-Output-External_Pressure', createStepName=self.step_name, variables=('ALLEN',))
        # self.model.HistoryOutputRequest(name='H-Output-External_Pressure', createStepName=self.step_name, variables=('ALLAE', 'ALLCD', 'ALLDMD', 'ALLFD', 'ALLIE', 'ALLKE', 'ALLPD', 'ALLSE', 'ALLVD', 'ALLWK', 'ETOTAL', 'DMASS'))
        self.model.fieldOutputRequests['F-Output-External_Pressure'].setValues(numIntervals=50,)

class Internal_pressure(Analysis):
    def build(self, endA, endB, previous, ext, inte, pressure, timePeriod):
        self.step_name = "Step_Internal_Pressure"
        nameEndA, regionEndA = endA
        nameEndB, regionEndB = endB

        self.model.ExplicitDynamicsStep(name= self.step_name, previous=previous, timePeriod=timePeriod)
        try:
            self.model.boundaryConditions['endA']
            self.model.boundaryConditions['endB']
        except:
            self.model.DisplacementBC(name="endA", createStepName= self.step_name, region=regionEndA, u1=SET, u2=SET, u3=SET, ur1=SET, ur2=SET)
            self.model.DisplacementBC(name="endB", createStepName= self.step_name, region=regionEndB, u1=SET, u3=SET, ur1=SET, ur2=SET)

        internal_region = self.assembly.surfaces[inte[1][0]]

        amplitude = self.create_amplitude('Amplitude_internal_pressure', ((0,0),(timePeriod,1),))

        self.model.Pressure(name='Internal Pressure', createStepName=self.step_name, region = internal_region, magnitude = pressure, amplitude = amplitude)

        self.model.FieldOutputRequest(name='F-Output-Internal_Pressure', createStepName=self.step_name, variables=('S','LE','U','V','A','RF','CSTRESS','CDISP','CSTATUS', 'EMSF'))
        self.model.HistoryOutputRequest(name='H-Output-Internal_Pressure', createStepName=self.step_name, variables=('ALLEN',))
        # self.model.HistoryOutputRequest(name='H-Output-Internal_Pressure', createStepName=self.step_name, variables=('ALLAE', 'ALLCD', 'ALLDMD', 'ALLFD', 'ALLIE', 'ALLKE', 'ALLPD', 'ALLSE', 'ALLVD', 'ALLWK', 'ETOTAL', 'DMASS'))
        self.model.fieldOutputRequests['F-Output-Internal_Pressure'].setValues(numIntervals=50,)

class Tension(Analysis):
    def build(self, endA, endB, previous, ext, inte, deformation, timePeriod):
        self.step_name = "Step_Tension"
        nameEndA, regionEndA = endA
        nameEndB, regionEndB = endB

        displacement = self.model_length * deformation
        self.model.ExplicitDynamicsStep(name= self.step_name, previous=previous, timePeriod=timePeriod)
        amplitude = self.create_amplitude('Amplitude_tension', ((0,0),(timePeriod,1),))
        self.model.DisplacementBC(name="endB_Tension", createStepName= self.step_name, region=regionEndB, u2=displacement, amplitude=amplitude)

        self.model.FieldOutputRequest(name='F-Output-TensionEndA', createStepName=self.step_name, variables=('RF', 'U'), region=self.assembly.sets[nameEndA])
        self.model.FieldOutputRequest(name='F-Output-TensionEndB', createStepName=self.step_name, variables=('RF', 'U'), region=self.assembly.sets[nameEndB])
        self.model.FieldOutputRequest(name='F-Output-Tension', createStepName=self.step_name, variables=('S','LE','U','V','A','RF','CSTRESS','CDISP','CSTATUS', 'EMSF'))
        self.model.HistoryOutputRequest(name='H-Output-Tension', createStepName=self.step_name, variables=('ALLAE', 'ALLCD', 'ALLDMD', 'ALLFD', 'ALLIE', 'ALLKE', 'ALLPD', 'ALLSE', 'ALLVD', 'ALLWK', 'ETOTAL', 'DMASS'))
        self.model.fieldOutputRequests['F-Output-TensionEndA'].setValues(numIntervals=400,)
        self.model.fieldOutputRequests['F-Output-TensionEndB'].setValues(numIntervals=400,)
        self.model.fieldOutputRequests['F-Output-Tension'].setValues(numIntervals=50,)

class Static_tension(Analysis):
    def build(self, endA, endB, previous, ext, inte, deformation, timePeriod):
        self.step_name = "Step_static_tension"
        nameEndA, regionEndA = endA
        nameEndB, regionEndB = endB

        displacement = self.model_length * deformation
        self.model.StaticStep(name= self.step_name, previous=previous, timePeriod=timePeriod, maxNumInc=1000, initialInc=0.1, minInc=1E-10, maxInc=1)
        amplitude = self.create_amplitude('Amplitude_Static-tension', ((0,0),(timePeriod,1),))
        self.model.DisplacementBC(name="endB_Static-Tension", createStepName= self.step_name, region=regionEndB, u2=displacement, amplitude=amplitude)

        self.model.FieldOutputRequest(name='F-Output-Static-TensionEndA', createStepName=self.step_name, variables=('RF', 'U'), region=self.assembly.sets[nameEndA])
        self.model.FieldOutputRequest(name='F-Output-Static-TensionEndB', createStepName=self.step_name, variables=('RF', 'U'), region=self.assembly.sets[nameEndB])
        self.model.FieldOutputRequest(name='F-Output-Static-Tension', createStepName=self.step_name, variables=('S','LE','U','RF','CSTRESS','CDISP'))
        self.model.HistoryOutputRequest(name='H-Output-Static-Tension', createStepName=self.step_name)
        self.model.fieldOutputRequests['F-Output-Static-TensionEndA'].setValues(numIntervals=400,)
        self.model.fieldOutputRequests['F-Output-Static-TensionEndB'].setValues(numIntervals=400,)
        self.model.fieldOutputRequests['F-Output-Static-Tension'].setValues(numIntervals=50,)

class Settling(Analysis):
    def build(self, endA, endB, previous, ext, inte, timePeriod):
        self.step_name = "Step_Settling"
        for bc in self.model.boundaryConditions.keys():
            self.model.boundaryConditions[bc].suppress()

        self.model.constraints["Coupling-endA"].suppress()
        self.model.constraints["Coupling-endB"].suppress()

        self.model.ExplicitDynamicsStep(name= self.step_name, previous=previous, timePeriod=timePeriod)
        self.model.DisplacementBC(name="endA_Settling", createStepName= self.step_name, region=self.assembly.sets[f'RegiaoY-0'], u2 = SET)
        self.model.DisplacementBC(name="endB_Settling", createStepName= self.step_name, region=self.assembly.sets[f'RegiaoY-{abs(int(1000*self.model_length))}'], u2 = SET)

class Static_settling(Analysis):
    def build(self, endA, endB, previous, ext, inte, timePeriod):
        self.step_name = "Step_Settling"
        for bc in self.model.boundaryConditions.keys():
            self.model.boundaryConditions[bc].suppress()

        self.model.constraints["Coupling-endA"].suppress()
        self.model.constraints["Coupling-endB"].suppress()

        self.model.StaticStep(name= self.step_name, previous=previous, timePeriod=timePeriod, maxNumInc=1000, initialInc=0.1, minInc=1E-10, maxInc=1)
        self.model.DisplacementBC(name="endA_Settling", createStepName= self.step_name, region=self.assembly.sets[f'RegiaoY-0'], u2 = SET)
        self.model.DisplacementBC(name="endB_Settling", createStepName= self.step_name, region=self.assembly.sets[f'RegiaoY-{abs(int(1000*self.model_length))}'], u2 = SET)

class Contact:
    def __init__(self, model, layers, objects):
        self.model = model
        self.layers = layers
        self.objects = objects
        self.assembly = model.rootAssembly
        contacts = {"ContactFriction": coef_friction, "ContactFrictionless": None}

        for name, mu in contacts.items():
            prop = self.model.ContactProperty(name)
            prop.NormalBehavior(pressureOverclosure=HARD, allowSeparation=ON)
            if mu is None:
                prop.TangentialBehavior(formulation=FRICTIONLESS)
            else:
                prop.TangentialBehavior(formulation=PENALTY, directionality=ISOTROPIC, slipRateDependency=OFF, pressureDependency=OFF, temperatureDependency=OFF, dependencies=0, table=((mu,),), shearStressLimit=None, maximumElasticSlip=FRACTION, fraction=0.005, elasticSlipStiffness=None)
        self.ext, self.inte = self.create_contact_surfaces()
    def create_solid_helix_surfaces(self, nome_instancia, external=True):
        """
        Cria objetos do tipo surface no abaqus para definir as superfícies internas e externas das armaduras sólidas.
        Dessa forma, é possível estabelecer contatos com as outras camadas. Retorna o nome da superfície criada.
        """
        instancia = self.assembly.instances[nome_instancia]
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

        self.model.rootAssembly.Surface(side1Faces=faces, name=surface_name)
        return surface_name
    def create_contact_surfaces(self):
        """
        Cria objetos do tipo surface no abaqus para definir as superfícies internas e externas de todas
        as camadas e as organiza em duas listas separadas e ordenadas de acordo com o dicionário de
        camadas. Retorna as listas.
        """
        externals, internals = {}, {}

        for key, values in layers.items():
            if issubclass(values['type'], Cylinder):
                ext, inte = f'Surf-{key}-Ext', f'Surf-{key}-Int'
                faces = self.assembly.instances[key].faces
                self.model.rootAssembly.Surface(side1Faces=faces, name=ext)
                self.model.rootAssembly.Surface(side2Faces=faces, name=inte)
                externals[ext], internals[inte] = values['type'], values['type']
            elif issubclass(values['type'], Shell_helix):
                for instance_name in (x for x in self.assembly.instances.keys() if key.lower() in x.lower()):
                    if not self.assembly.features[instance_name].isSuppressed():
                        ext, inte = f'Surf-{instance_name}-Ext', f'Surf-{instance_name}-Int'
                        faces = self.assembly.instances[instance_name].faces
                        self.model.rootAssembly.Surface(side2Faces=faces, name=ext)
                        self.model.rootAssembly.Surface(side1Faces=faces, name=inte)
                        externals[ext], internals[inte] = values['type'], values['type']
            else:
                for instance_name in (x for x in self.assembly.instances.keys() if key.lower() in x.lower()):
                    if not self.assembly.features[instance_name].isSuppressed():
                        externals[self.create_solid_helix_surfaces(instance_name, True)] = values['type']
                        internals[self.create_solid_helix_surfaces(instance_name, False)] = values['type']
        
        ext, inte = list(externals.items()), list(internals.items())
        return ext, inte
    def create_contact_initialization(self, contact_name, externals, internals):
        """
        Cria uma inicialização das superfícies de contato do tipo 'Interference' com o objetivo de
        ajustar as posições dos nós antes da simulação.
        """
        self.model.ExpInitialization(name='Contact_Initialization', overclosureType=INTERFERENCE)
        assignments = []
        for i, (surface1, type1) in enumerate(externals):
            for j, (surface2, type2) in enumerate(internals[i+1:]):
                cyl1 = issubclass(type1, Cylinder)
                cyl2 = issubclass(type2, Cylinder)
                if cyl1 and cyl2 and j != 0:
                    break
                if not cyl1 and not cyl2:
                    continue
                assignments.append((self.assembly.surfaces[surface1],self.assembly.surfaces[surface2],"Contact_Initialization"))
                print(f"Inicialização das superfícies {surface1} e {surface2}")
                if not cyl1 or cyl2:
                    break
        self.model.interactions[contact_name].initializationAssignments.appendInStep(stepName='Initial', assignments=tuple(assignments))
    def compute_contact_stiffness(self, externals, internals):
        def get_layer_name(surf: str):
            padrao = r"^Surf-(.*?)(?:-\d+)?(?:-(?:Ext|Int))?$"
            m = re.match(padrao, surf)
            return m.group(1) if m else None
        contacts = []
        for i, (surface1, type1) in enumerate(externals):
            for j, (surface2, type2) in enumerate(internals[i+1:]):
                cyl1 = issubclass(type1, Cylinder)
                cyl2 = issubclass(type2, Cylinder)
                if cyl1 and cyl2 and j != 0:
                    break
                if not cyl1 and not cyl2:
                    continue
                contact_name = f"Contact - {surface1} - {surface2}"
                prop = self.model.ContactProperty(contact_name)
                E1 = self.objects[get_layer_name(surface1)].young_modulus
                h1 = self.objects[get_layer_name(surface1)].thickness
                r1 = self.objects[get_layer_name(surface1)].radius
                E2 = self.objects[get_layer_name(surface2)].young_modulus
                h2 = self.objects[get_layer_name(surface2)].thickness
                r2 = self.objects[get_layer_name(surface2)].radius
                length = self.objects[get_layer_name(surface2)].length
                ndc = c_seed
                if cyl1 and cyl2:
                    prop.TangentialBehavior(formulation=FRICTIONLESS)
                    ndl1 = self.objects[get_layer_name(surface1)].ndl
                    ndl2 = self.objects[get_layer_name(surface2)].ndl
                    ndl = max(ndl1, ndl2)
                    stiffness = 2*E1*E2/(E1*h2 + E2*h1) * ((2 * pi * r2 * length) / (ndc * ndl))
                    prop.NormalBehavior(pressureOverclosure=LINEAR, contactStiffness=stiffness, allowSeparation=ON)
                else:
                    prop.TangentialBehavior(formulation=PENALTY, directionality=ISOTROPIC, slipRateDependency=OFF, pressureDependency=OFF, temperatureDependency=OFF, dependencies=0, table=((0.1,),), shearStressLimit=None, maximumElasticSlip=FRACTION, fraction=0.005, elasticSlipStiffness=None)
                    if cyl1:
                        ndl = self.objects[get_layer_name(surface1)].ndl
                        stiffness = 2*E1/h1 * ((2 * pi * r1 * length) / (ndc * ndl))
                        prop.NormalBehavior(pressureOverclosure=LINEAR, contactStiffness=stiffness, allowSeparation=ON)
                    elif cyl2:
                        ndl = self.objects[get_layer_name(surface2)].ndl
                        stiffness = 2*E2/h2 * ((2 * pi * r2 * length) / (ndc * ndl))
                        prop.NormalBehavior(pressureOverclosure=LINEAR, contactStiffness=stiffness, allowSeparation=ON)
                contacts.append((self.assembly.surfaces[surface1],self.assembly.surfaces[surface2], contact_name))
                if not cyl1 or cyl2:
                    break
        return tuple(contacts)
    def select_frictionless_layers(self, externals, internals):
        """
        Seleciona as camadas do tipo 'cylinder' para criação de um contato sem atrito.
        Retorna uma tupla de tuplas com as camadas selecionadas.
        """
        selected = []
        for (surface1, type1), (surface2, type2) in zip(externals, internals[1:]):
            if any(not issubclass(t, Cylinder) for t in (type1, type2)):
                continue
            selected.append((self.assembly.surfaces[surface1], self.assembly.surfaces[surface2], "ContactFrictionless"))
        return tuple(selected)
    def create_contact_pairs(self, externals, internals):
        """
        Cria uma inicialização das superfícies de contato do tipo 'Interference' com o objetivo de
        ajustar as posições dos nós antes da simulação.
        """
        self.model.ExpInitialization(name='Contact_Initialization', overclosureType=INTERFERENCE)
        pairs = []
        for i, (surface1, type1) in enumerate(externals):
            for j, (surface2, type2) in enumerate(internals[i+1:]):
                cyl1 = issubclass(type1, Cylinder)
                cyl2 = issubclass(type2, Cylinder)
                if cyl1 and cyl2 and j != 0:
                    break
                if not cyl1 and not cyl2:
                    continue
                pairs.append((self.assembly.surfaces[surface1],self.assembly.surfaces[surface2]))
                print(f"Inicialização das superfícies {surface1} e {surface2}")
                if not cyl1 or cyl2:
                    break
        return tuple(pairs)
    def build(self, step_name):
        print(f"\n================================ {'Contact': ^24} ================================\n")
        # self.model.ContactExp(name='Initial_contact', createStepName='Initial', contactPropertyAssignments=((GLOBAL, SELF, "ContactFrictionless"),))
        # self.model.interactions['Initial_contact'].deactivate(step_name)
        # self.model.interactions['Initial_contact'].setValues(globalSmoothing=True)
        # self.create_contact_initialization('Initial_contact', self.ext, self.inte)
        frictionless_layers = self.select_frictionless_layers(self.ext, self.inte)
        # pairs = self.create_contact_pairs(self.ext, self.inte)
        # contacts = self.compute_contact_stiffness(self.ext, self.inte)
        # self.model.ContactExp(name=f'Contact_{step_name}', createStepName=step_name, includedPairs = pairs,contactPropertyAssignments=((GLOBAL, SELF, "ContactFriction"),) + frictionless_layers)
        self.model.ContactExp(name=f'Contact_{step_name}', createStepName=step_name, contactPropertyAssignments=((GLOBAL, SELF, "ContactFriction"),) + frictionless_layers)
        self.model.interactions[f'Contact_{step_name}'].setValues(globalSmoothing=True)
        # self.model.ContactExp(name=f'Contact_{step_name}', createStepName=step_name, contactPropertyAssignments=((GLOBAL, SELF, "ContactFriction"),) + contacts)

class Explicit_contact(Contact):
    pass
    # def build(self, step_name):
    #     print(f"\n================================ {'Explicit Contact': ^24} ================================\n")
    #     self.model.ContactExp(name='Initial_contact', createStepName='Initial', contactPropertyAssignments=((GLOBAL, SELF, "ContactFrictionless"),))
    #     self.model.interactions['Initial_contact'].deactivate(step_name)
    #     self.create_contact_initialization('Initial_contact', self.ext, self.inte)
    #     frictionless_layers = self.select_frictionless_layers(self.ext, self.inte)
    #     self.model.ContactExp(name=f'Contact_{step_name}', createStepName=step_name, contactPropertyAssignments=((GLOBAL, SELF, "ContactFriction"),) + frictionless_layers)

class Standard_contact(Contact):
    def build(self, step_name):
        print(f"\n================================ {'Standard Contact': ^24} ================================\n")
        self.model.ContactStd(name='Initial_contact', createStepName='Initial', contactPropertyAssignments=((GLOBAL, SELF, "ContactFrictionless"),))
        self.create_contact_initialization('Initial_contact', self.ext, self.inte)

class Tied_contact(Contact):
    def build(self, step_name):
        print(f"\n================================ {'Tied Contact': ^24} ================================\n")
        for i, ((s1, t1), (s2, t2)) in enumerate(zip(self.ext, self.inte[1:])):
            self.model.Tie(name=f"Tie-{i}", main=self.assembly.surfaces[s1], secondary=self.assembly.surfaces[s2])

class Surf_to_surf_Static_contact(Contact):
    def create_static_surf_surf(self, step_name, externals, internals):
        """
        Cria um contato explícito do tipo surface to surface.
        """
        def choose_main_secondary(surf1, type1, surf2, type2):
            if issubclass(type1, Isotropic_cylinder) and (not issubclass(type2, Isotropic_cylinder)):
                return surf2, surf1
            return surf1, surf2

        for i, (surface1, type1) in enumerate(externals):
            for j, (surface2, type2) in enumerate(internals[i+1:]):
                cyl1 = issubclass(type1, Cylinder)
                cyl2 = issubclass(type2, Cylinder)
                if cyl1 and cyl2:
                    if j == 0:
                        main, secondary = choose_main_secondary(surface1, type1, surface2, type2)
                        self.model.SurfaceToSurfaceContactStd(name=f'Contact_{main}-{secondary}', createStepName=step_name, main = self.assembly.surfaces[main], secondary = self.assembly.surfaces[secondary], sliding = FINITE,interactionProperty="ContactFrictionless", adjustMethod=OVERCLOSED)#, mechanicalConstraint=PENALTY)
                        print(f"Contato entre as superfícies {surface1} e {surface2}")
                        break
                    else:
                        break
                if not cyl1 and not cyl2:
                    continue
                main, secondary = choose_main_secondary(surface1, type1, surface2, type2)
                self.model.SurfaceToSurfaceContactStd(name=f'Contact_{main}-{secondary}', createStepName=step_name, main = self.assembly.surfaces[main], secondary = self.assembly.surfaces[secondary], sliding = FINITE,interactionProperty="ContactFriction", adjustMethod=OVERCLOSED)#, mechanicalConstraint=PENALTY)
                print(f"Contato entre as superfícies {surface1} e {surface2}")
                if not cyl1 or cyl2:
                    break
    def build(self, step_name):
        self.create_static_surf_surf('Initial', self.ext, self.inte)

class Surf_to_surf_Explicit_contact(Contact):
    def create_explict_surf_surf(self, step_name, externals, internals):
        """
        Cria um contato explícito do tipo surface to surface.
        """
        def choose_main_secondary(surf1, type1, surf2, type2):
            if issubclass(type1, Isotropic_cylinder) and (not issubclass(type2, Isotropic_cylinder)):
                return surf2, surf1
            return surf1, surf2

        for i, (surface1, type1) in enumerate(externals):
            for j, (surface2, type2) in enumerate(internals[i+1:]):
                cyl1 = issubclass(type1, Cylinder)
                cyl2 = issubclass(type2, Cylinder)
                if cyl1 and cyl2:
                    if j == 0:
                        main, secondary = choose_main_secondary(surface1, type1, surface2, type2)
                        self.model.SurfaceToSurfaceContactExp(name=f'Contact_{main}-{secondary}', createStepName=step_name, main = self.assembly.surfaces[main], secondary = self.assembly.surfaces[secondary], sliding = FINITE,interactionProperty="ContactFrictionless", mechanicalConstraint=PENALTY)
                        print(f"Contato entre as superfícies {surface1} e {surface2}")
                        break
                    else:
                        break
                if not cyl1 and not cyl2:
                    continue
                main, secondary = choose_main_secondary(surface1, type1, surface2, type2)
                self.model.SurfaceToSurfaceContactExp(name=f'Contact_{main}-{secondary}', createStepName=step_name, main = self.assembly.surfaces[main], secondary = self.assembly.surfaces[secondary], sliding = FINITE,interactionProperty="ContactFriction", mechanicalConstraint=PENALTY)
                print(f"Contato entre as superfícies {surface1} e {surface2}")
                if not cyl1 or cyl2:
                    break
    def build(self, step_name):
        self.create_explict_surf_surf(step_name, self.ext, self.inte)
        # for i, name in enumerate(list(self.model.interactions.keys())):
        #     self.model.HistoryOutputRequest(name=f'H-Output-A{i}', createStepName=step_name, variables=ALL, interactions=(name,))
        #     self.model.historyOutputRequests[f'H-Output-A{i}'].setValues(numIntervals=200,)

layers = read_json(r"risers\witz-si.json")