from caeModules import mesh
from abaqus import mdb
from abaqusConstants import * # type: ignore
from numpy import pi, sin, cos, tan, radians, array, diag, ceil
import json

t_seed, c_seed = (6, 44)

def pitch(R, angle):
    p = 2*pi*R/tan(radians(angle))
    return p

def read_json(file):
    with open(file, "r") as f:
        layers = json.load(f)
    types = {
        "isotropic_cylinder": Isotropic_cylinder,
        "orthotropic_cylinder": Orthotropic_cylinder,
        "solid_isotropic_cylinder": Solid_isotropic_cylinder,
        "solid_helix": Solid_helix
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

class Model:
    def __init__(self, name, cases):
        """
        Cria um modelo a partir da sua categoria. Tipos possíveis:
        b   - Modelo de flexão com step explícito
        t   - Modelo de tração com step explicíto
        is  - Modelo com pressão externa inicial
        ip  - Modelo com pressão interna com step explícito
        ep  - Modelo com pressão external com step explícito
        f   - Modelo de frequência
        """
        t = {
            "b": Bending,
            "t": Tension,
            "is": Initial_state,
            "ip": Internal_pressure,
            "ep": External_pressure,
            "f": Frequency
        }
        self.name = name
        self.cases = [(t[case], params) for case, params in cases]
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
        if helix_layers := [l for l in layers.values() if l.get("type") == Solid_helix]:
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
        for layer in layers.values():
            cls = layer["type"]
            params = {k: v for k, v in layer.items() if k != "type"}
            cls(**params).build(model)
    def create_analysis(self, model, layers, model_length):
        c = self.cases[0][0](model, layers, model_length)      
        ctc = Standard_contact(model) if isinstance(c, Frequency) else Explicit_contact(model)
        nameEndA, endA = c.create_coupling("endA", (0,0,0))
        nameEndB, endB = c.create_coupling("endB", (0,model_length,0))
        model.DisplacementBC(name="endA", createStepName= 'Initial', region=endA, u1=SET, u2=SET, u3=SET, ur1=SET, ur2=SET)
        model.DisplacementBC(name="endB", createStepName= 'Initial', region=endB, u1=SET, u3=SET, ur1=SET, ur2=SET)

        previous = 'Initial'
        ctc_done = False
        for case, param in self.cases:
            c = case(model, layers, model_length)
            c.build((nameEndA, endA), (nameEndB, endB), previous, ctc.ext, ctc.inte, **param)
            if c.step_name is not None:
                previous = c.step_name
                if not ctc_done:
                    ctc.build(c.step_name)
                    ctc_done=True
    def build(self, number_pitches, model_length, layers):
        model = self.create_model()
        layers, model_length = self.define_model_length(number_pitches, model_length, layers)
        self.build_assembly(model, layers)
        self.create_analysis(model, layers, model_length)
        mdb.Job(name=f'Job-{self.name}', model=self.name, type=ANALYSIS, explicitPrecision=DOUBLE, nodalOutputPrecision=SINGLE, numCpus=24, numDomains=24, multiprocessingMode=DEFAULT)

class Cylinder:
    def __init__(self, name, length, mid_diameter, thickness, name_material, young_modulus, poisson, density):
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
        part.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE,offset=0)
        part.PartitionFaceByDatumPlane(faces=part.faces, datumPlane=part.datums[3])
        base, lay_angle = self.get_solid_helix_props(layers)
        if (base is not None) and (lay_angle is not None):
            part.seedEdgeByNumber(edges=[a for a in part.edges if a.getSize()>2*pi*self.radius], number=int(self.length/((base)/sin(radians(lay_angle)))))
        else:
            part.seedEdgeByNumber(edges=[a for a in part.edges if a.getSize()>2*pi*self.radius], number=int(self.length*100))
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
    def __init__(self, name, pitch, mid_diameter, thickness, name_material, young_modulus, poisson, density):
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
    def __init__(self, name, length, mid_diameter, thickness, name_material, young_modulus, poisson, density, n, Ix, Iy, A, J, lay_angle):
        super().__init__(name, length, mid_diameter, thickness, name_material, young_modulus, poisson, density)
        self.n = n
        self.Ix = Ix
        self.Iy = Iy
        self.A = A
        self.J = J
        self.lay_angle = lay_angle
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

        EAeq = self.n*self.young_modulus*self.A/(passo*s)
        I2 = (self.Ix+self.Iy)/2-(((self.Iy-self.Ix)**2+4*self.J**2)**(1/2))/2
        EIeq = K*self.n*self.young_modulus*I2/(passo*s)
        G = self.young_modulus/(2*(1+self.poisson))
        GJeq = self.n*G*self.J/(passo*s)
        
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
            part.seedEdgeByNumber(edges=[a for a in part.edges if a.getSize()>2*pi*self.radius], number=int(self.length*10))
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
    def __init__(self, name, pitch, mid_diameter, thickness, name_material, young_modulus, poisson, density, lay_angle, n, base, revolve_direction, length = 0):
        super().__init__(name, pitch, mid_diameter, thickness, name_material, young_modulus, poisson, density)
        self.lay_angle = lay_angle
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
        sketch.rectangle(point1=(self.radius-self.thickness/2,0), point2=(self.radius+self.thickness/2, self.base/sin(radians(self.lay_angle))))
        sketch.ConstructionLine(point1=(0.0, -self.length), point2=(0.0, self.length))
        p = model.Part(name=self.name, dimensionality=THREE_D, type=DEFORMABLE_BODY)
        extend_factor = 1.1
        p.BaseSolidRevolve(sketch=sketch, angle=extend_factor*self.length/self.pitch*360.0, flipRevolveDirection=self.revolve_direction, pitch=self.pitch, flipPitchDirection = OFF, moveSketchNormalToPath=OFF)
        p.setValues(geometryRefinement=EXTRA_FINE)
        g = sketch.geometry
        for k in g.keys():
            sketch.delete((g[k],))
        
        # Apaga o excesso da armadura de tração
        planes = [(extend_factor-1)/2, (extend_factor+1)/2]
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
        p.seedEdgeByNumber(edges=[a for a in p.edges if a.getSize()>comprimento_minimo], number=int(self.length/(self.base/sin(radians(self.lay_angle))))) # Conferir generalidade
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
            assembly.translate(instanceList=(instance_rotated_name,), vector=(0,(extend_factor+1)/2*self.length,0))
                
        g = sketch.geometry
        for k in g.keys():
            sketch.delete((g[k],))

        assembly.suppressFeatures((instance_name,))
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
        self.model.Coupling(name=f'Coupling-{name}', surface=end, controlPoint=regiao, influenceRadius=WHOLE_SURFACE, couplingType=KINEMATIC)
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
        self.step_name = "Step_Explict"
        nameEndA, regionEndA = endA
        nameEndB, regionEndB = endB

        rotation = (self.model_length * curvature) / 1000 if (self.model_length * curvature) > 1 else (self.model_length * curvature)
        self.model.ExplicitDynamicsStep(name= self.step_name, previous=previous, timePeriod=timePeriod)
        amplitude = self.create_amplitude('Amplitude', ((0,0),(timePeriod,1),(3*timePeriod,-1),(4*timePeriod,0)))if hysteresis else self.create_amplitude('Amplitude', ((0,0),(timePeriod,1),))
        self.model.DisplacementBC(name="endA_Bending", createStepName= self.step_name, region=regionEndA, ur3=-rotation/2, amplitude=amplitude)
        self.model.DisplacementBC(name="endB_Bending", createStepName= self.step_name, region=regionEndB, ur3=rotation/2, amplitude=amplitude)

        self.model.FieldOutputRequest(name='F-Output-EndA', createStepName=self.step_name, variables=('RM', 'UR'), region=self.assembly.sets[nameEndA])
        self.model.FieldOutputRequest(name='F-Output-EndB', createStepName=self.step_name, variables=('RM', 'UR'), region=self.assembly.sets[nameEndB])
        self.model.FieldOutputRequest(name='F-Output-Bending', createStepName=self.step_name, variables=('S','LE','U','V','A','RF','CSTRESS','CDISP','CSTATUS', 'EMSF'))
        self.model.HistoryOutputRequest(name='H-Output-Bending', createStepName=self.step_name, variables=('ALLAE', 'ALLCD', 'ALLDMD', 'ALLFD', 'ALLIE', 'ALLKE', 'ALLPD', 'ALLSE', 'ALLVD', 'ALLWK', 'ETOTAL', 'DMASS'))
        self.model.fieldOutputRequests['F-Output-EndA'].setValues(numIntervals=400,)
        self.model.fieldOutputRequests['F-Output-EndB'].setValues(numIntervals=400,)
        self.model.fieldOutputRequests['F-Output-Bending'].setValues(numIntervals=50,)

class Frequency(Analysis):
    def build(self, endA, endB, previous, ext, inte):
        self.step_name='Step_Frequency'
        self.model.FrequencyStep(name=self.step_name, previous = previous, eigensolver = LANCZOS, numEigen = 2)

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

        self.model.ExplicitDynamicsStep(name= self.step_name, previous=previous, timePeriod=timePeriod)

        external_region = self.assembly.surfaces[ext[-1][0]]

        amplitude = self.create_amplitude('Amplitude_external_pressure', ((0,0),(timePeriod,1),))

        self.model.Pressure(name='External Pressure', createStepName=self.step_name, region = external_region, magnitude = pressure, amplitude=amplitude)

        self.model.FieldOutputRequest(name='F-Output-External_Pressure', createStepName=self.step_name, variables=('S','LE','U','V','A','RF','CSTRESS','CDISP','CSTATUS', 'EMSF'))
        self.model.HistoryOutputRequest(name='H-Output-External_Pressure', createStepName=self.step_name, variables=('ALLAE', 'ALLCD', 'ALLDMD', 'ALLFD', 'ALLIE', 'ALLKE', 'ALLPD', 'ALLSE', 'ALLVD', 'ALLWK', 'ETOTAL', 'DMASS'))
        self.model.fieldOutputRequests['F-Output-External_Pressure'].setValues(numIntervals=50,)

class Internal_pressure(Analysis):
    def build(self, endA, endB, previous, ext, inte, pressure, timePeriod):
        self.step_name = "Step_Internal_Pressure"

        self.model.ExplicitDynamicsStep(name= self.step_name, previous=previous, timePeriod=timePeriod)

        internal_region = self.assembly.surfaces[inte[1][0]]

        amplitude = self.create_amplitude('Amplitude_internal_pressure', ((0,0),(timePeriod,1),))

        self.model.Pressure(name='Internal Pressure', createStepName=self.step_name, region = internal_region, magnitude = pressure, amplitude = amplitude)

        self.model.FieldOutputRequest(name='F-Output-Internal_Pressure', createStepName=self.step_name, variables=('S','LE','U','V','A','RF','CSTRESS','CDISP','CSTATUS', 'EMSF'))
        self.model.HistoryOutputRequest(name='H-Output-Internal_Pressure', createStepName=self.step_name, variables=('ALLAE', 'ALLCD', 'ALLDMD', 'ALLFD', 'ALLIE', 'ALLKE', 'ALLPD', 'ALLSE', 'ALLVD', 'ALLWK', 'ETOTAL', 'DMASS'))
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

class Contact:
    def __init__(self, model):
        self.model = model
        self.assembly = model.rootAssembly
        contacts = {"ContactFriction": 0.1, "ContactFrictionless": None}

        for name, mu in contacts.items():
            prop = model.ContactProperty(name)
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
            else:
                for instance_name in (x for x in self.assembly.instances.keys() if key.lower() in x.lower()):
                    if not self.assembly.features[instance_name].isSuppressed():
                        externals[self.create_solid_helix_surfaces(instance_name, True)] = values['type']
                        internals[self.create_solid_helix_surfaces(instance_name, False)] = values['type']
        
        ext, inte = list(externals.items()), list(internals.items())
        return ext, inte
    def create_contact_initialization(self, nome_contato, externals, internals):
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
        self.model.interactions[nome_contato].initializationAssignments.appendInStep(stepName='Initial', assignments=tuple(assignments))
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
    def build(self, step_name):
        print(f"\n================================ {'Contact': ^24} ================================\n")
        self.model.interactions['Initial_contact'].deactivate(step_name)
        self.create_contact_initialization('Initial_contact', self.ext, self.inte)
        frictionless_layers = self.select_frictionless_layers(self.ext, self.inte)
        self.model.ContactExp(name=f'Contact_{step_name}', createStepName=step_name, contactPropertyAssignments=((GLOBAL, SELF, "ContactFriction"),) + frictionless_layers)

class Explicit_contact(Contact):
    def build(self, step_name):
        print(f"\n================================ {'Explicit Contact': ^24} ================================\n")
        self.model.ContactExp(name='Initial_contact', createStepName='Initial', contactPropertyAssignments=((GLOBAL, SELF, "ContactFrictionless"),))
        self.model.interactions['Initial_contact'].deactivate(step_name)
        self.create_contact_initialization('Initial_contact', self.ext, self.inte)
        frictionless_layers = self.select_frictionless_layers(self.ext, self.inte)
        self.model.ContactExp(name=f'Contact_{step_name}', createStepName=step_name, contactPropertyAssignments=((GLOBAL, SELF, "ContactFriction"),) + frictionless_layers)

class Standard_contact(Contact):
    def build(self, step_name):
        print(f"\n================================ {'Standard Contact': ^24} ================================\n")
        self.model.ContactStd(name='Initial_contact', createStepName='Initial', contactPropertyAssignments=((GLOBAL, SELF, "ContactFrictionless"),))
        self.create_contact_initialization('Initial_contact', self.ext, self.inte)

layers = read_json(r"risers\witz-si.json")