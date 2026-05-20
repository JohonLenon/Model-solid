from caeModules import mesh
from abaqusConstants import * # type: ignore
from numpy import pi, sin, cos, tan, radians, array, diag
from set_dados_gerais import discretization
import json

t_seed, c_seed = (discretization["Tension armour"], discretization["Circumferential seed"])

def pitch(R, angle):
    p = 2*pi*R/tan(radians(angle))
    return p

class cylinder:
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
    def create_geometry(self, model):
        """
        Cria a geometria da casca cilíndrica e retorna um objeto de parte do abaqus e um objeto de sketch.
        """
        print(f"\n================================ {self.name: ^24} ================================\n")
        sketch = model.ConstrainedSketch(name=f'sketch_{self.name}', sheetSize=200.0)
        sketch.CircleByCenterPerimeter(center=(0,0), point1=(0, self.radius))
        part = model.Part(name=self.name, dimensionality=THREE_D, type=DEFORMABLE_BODY)
        part.BaseShellExtrude(sketch=sketch, depth= self.pitch)
        part.setValues(geometryRefinement=EXTRA_FINE)
        print(f"Geometria da camada {self.name} criada. \n")
        return part, sketch
    def create_mesh(self, part):
        """
        Cria uma malha estruturada com elemento S4R com controle de hourglass enhanced.
        """
        part.seedEdgeByNumber(edges=part.edges, number=c_seed)
        part.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE,offset=0)
        part.PartitionFaceByDatumPlane(faces=part.faces, datumPlane=part.datums[3])
        base, lay_angle = self.get_solid_helix_props(layers)
        part.seedEdgeByNumber(edges=[a for a in part.edges if a.getSize()>2*pi*self.radius], number=int(self.pitch/((base)/sin(radians(lay_angle)))))
        part.setMeshControls(regions=part.faces, technique=STRUCTURED)
        part.setElementType(regions=((part.faces,)), elemTypes=(mesh.ElemType(elemCode=S4R,elemLibrary=EXPLICIT,hourglassControl=ENHANCED),))
        part.generateMesh()
        print(f"Malha da camada {self.name} criada. \n")
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
        print(f"Material {material.name} da camada {self.name} criado. \n")
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
        print(f"Assembly da camada {self.name} criado. \n")
    def get_solid_helix_props(self, layers):
        """
        Acessa e retorna os valores da base da seção da armadura de tração e do seu ângulo de hélice para ajuste das
        dimensões da malha estruturada.
        """
        tension_layers = [layer for layer in layers.values() if layer.get("type") in armour_types and 'base' in layer and 'lay_angle' in layer]
        if not tension_layers:
            raise ValueError("Nenhuma layer válida do tipo 'solid_helix' encontrada.")
        selected = min(tension_layers, key=lambda l: abs(l['base']))
        base = selected['base']
        lay_angle = abs(selected['lay_angle'])
        return base, lay_angle

class helix:
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

            print(f"Material {material.name} da camada {self.name} criado.\n")

class shell_cylinder(cylinder):
    """
    Representa uma camada de casca cilíndrica. Cria uma casca cilíndrica com malha estruturada e alinhada com as dimensões
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

class orthotropic_cylinder(cylinder):
    """
    Representa uma camada de casca cilíndrica ortotrópica. Cria uma casca cilíndrica ortotrópica com malha estruturada e 
    alinhada com as dimensões dos fios da armadura de tração. Possui material linear elástico. Ao incluir no assembly é
    girada para corresponder ao eixo da armadura de tração.

    name            -   Nome da parte                       (input)
    pitch           -   Comprimento do pitch em metros      (input)
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
    def __init__(self, name, pitch, mid_diameter, thickness, name_material, young_modulus, poisson, density, n, Ix, Iy, A, J, lay_angle):
        super().__init__(name, pitch, mid_diameter, thickness, name_material, young_modulus, poisson, density)
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
        
        eps=10**(-3)
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
                density           = self.density,
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
        print(f"Seção de matriz ABD da camada {self.name} criada. \n")
    def build(self, model):
        """
        Constrói toda a camada.
        """
        part, sketch = self.create_geometry(model)
        self.create_mesh(part)
        self.create_section(model, part)
        self.create_assembly(model, part, sketch)

class solid_cylinder(cylinder):
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
        part.BaseSolidExtrude(sketch=sketch, depth= self.pitch)
        part.setValues(geometryRefinement=EXTRA_FINE)
        print(f"Geometria da camada {self.name} criada. \n")
        return part, sketch
    def create_mesh(self, part):
        """
        Cria uma malha estruturada com elemento C3D8I.
        """
        part.seedEdgeByNumber(edges=part.edges, number=c_seed)
        part.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE,offset=0)
        part.PartitionCellByDatumPlane(cells=part.cells, datumPlane=part.datums[3])
        base, lay_angle = self.get_solid_helix_props(layers)
        part.seedEdgeByNumber(edges=[a for a in part.edges if a.getSize()>2*pi*self.radius], number=int(self.pitch/((base)/sin(radians(lay_angle)))))
        part.setMeshControls(regions=part.cells, technique=STRUCTURED)
        part.setElementType(regions=((part.cells,)), elemTypes=(mesh.ElemType(elemCode=C3D8I,),))
        part.generateMesh()
        print(f"Malha da camada {self.name} criada. \n")
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

class solid_helix(helix):
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

        print(f"Geometria da camada {self.name} criada. \n")
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
        print(f"Malha da camada {self.name} criada. \n")

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
        print(f"Assembly da camada {self.name} criado. \n")
    
    def build(self, model):
        """
        Constrói toda a camada.
        """
        p, extend_factor, sketch, comprimento_minimo = self.create_geometry(model)
        self.create_mesh(p, comprimento_minimo)
        self.create_material(model)
        self.create_section(model, p)
        self.create_assembly(model, p, extend_factor, sketch)

with open(r"risers\witz-mpa.json", "r") as f:
    layers = json.load(f)
types = {
    "shell_cylinder": shell_cylinder,
    "orthotropic_cylinder": orthotropic_cylinder,
    "solid_cylinder": solid_cylinder,
    "solid_helix": solid_helix
}
constantes = {
    "ON": ON,
    "OFF": OFF
}
for layer in layers.values():
    layer["type"] = types[layer["type"]]
    if "revolve_direction" in layer:
        layer["revolve_direction"] = constantes[layer["revolve_direction"]]

layer_order = {layer: idx for idx, layer in enumerate(layers)}
def obter_chave_ordenacao(s):
    parte = s.replace("Surf-", "")
    if "-" in parte:
        parte = "-".join(parte.split("-")[:-1])
    return layer_order.get(parte, float('inf'))

armour_types = [t['type'] for t in layers.values()
         if 'helix' in t['type'].__name__.lower()]