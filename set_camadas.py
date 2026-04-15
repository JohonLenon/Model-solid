from caeModules import mesh
from abaqusConstants import * # type: ignore
from numpy import pi, sin, cos, tan, radians, sign, array, diag

numero_elementos = 44
discretizacao_tension_armour = 6


def pitch(R, angle, number_pitches=1):
    p = number_pitches*2*pi*R/tan(radians(angle))
    return p

class cilindros_shell:
    def __init__(self, name, pitch, mid_diameter, thickness, name_material, young_modulus, poisson, density, plastic_table = None, **kwargs):
        self.name = name
        self.pitch = pitch
        self.mid_diameter = mid_diameter
        self.radius = mid_diameter/2
        self.thickness = thickness
        self.name_material = name_material
        self.young_modulus = young_modulus
        self.poisson = poisson
        self.density = density
        self.plastic_table = plastic_table
    def create_geometry(self, model):
        # Cria geometria
        print("========================================================")
        sk = model.ConstrainedSketch(name='rect', sheetSize=200.0)
        sk.CircleByCenterPerimeter(center=(0,0), point1=(0, self.radius))
        part = model.Part(name=self.name, dimensionality=THREE_D, type=DEFORMABLE_BODY)
        part.BaseShellExtrude(sketch=sk, depth= self.pitch)
        part.setValues(geometryRefinement=EXTRA_FINE)
        print(f"Geometria {self.name} criada. \n")

        # Cria malha
        part.seedEdgeByNumber(edges=part.edges, number=numero_elementos)
        part.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE,offset=0)
        part.PartitionFaceByDatumPlane(faces=part.faces, datumPlane=part.datums[3])
        part.seedEdgeByNumber(edges=[a for a in part.edges if a.getSize()>2*pi*self.radius], number=int(self.pitch/(6/sin(radians(35)))))
        part.setMeshControls(regions=part.faces, technique=STRUCTURED)
        part.setElementType(regions=((part.faces,)), elemTypes=(mesh.ElemType(elemCode=S4R,elemLibrary=EXPLICIT,hourglassControl=ENHANCED),))
        part.generateMesh()
        print(f"Malha {self.name} criada. \n")

        # Cria material
        try:
            material = model.materials[self.name_material]
        except:
            material = model.Material(name=self.name_material)
            material.Elastic(table=((self.young_modulus, self.poisson),))
            material.Density(table=((self.density,),))
            if self.plastic_table is not None:
                material.Plastic(table=self.plastic_table) 
        print(f"Material {material.name} criado. \n")

        # Cria seção e associa material
        model.HomogeneousShellSection(name=f'Section_{self.name}', material=self.name_material, thickness=self.thickness)
        region = part.Set(faces=part.faces, name=f'Set-{self.name}')
        part.SectionAssignment(region=region, sectionName=f'Section_{self.name}', offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_SECTION)

        # Cria assembly e posiciona
        assembly = model.rootAssembly
        assembly.Instance(name=self.name, part=part, dependent=ON)
        assembly.rotate(instanceList=[self.name], axisPoint = (0,0,0), axisDirection =(1, 0, 0), angle = -90)
        g = sk.geometry
        for k in g.keys():
            sk.delete((g[k],))
        print(f"Assembly {self.name} criado. \n")

class cilindros_ortotropicos:
    def __init__(self, name, n, pitch, mid_diameter, thickness, name_material, young_modulus, poisson, density, Ix, Iy, A, J, lay_angle, plastic_table = None, **kwargs):
        self.name = name
        self.n = n
        self.pitch = pitch
        self.mid_diameter = mid_diameter
        self.radius = mid_diameter/2
        self.thickness = thickness
        self.name_material = name_material
        self.young_modulus = young_modulus
        self.poisson = poisson
        self.density = density
        self.Ix = Ix
        self.Iy = Iy
        self.A = A
        self.J = J
        self.lay_angle = lay_angle
        self.plastic_table = plastic_table
    def create_geometry(self, model):
        # Cria geometria
        print("========================================================")
        sk = model.ConstrainedSketch(name='rect', sheetSize=200.0)
        sk.CircleByCenterPerimeter(center=(0,0), point1=(0, self.radius))
        part = model.Part(name=self.name, dimensionality=THREE_D, type=DEFORMABLE_BODY)
        part.BaseShellExtrude(sketch=sk, depth= self.pitch)
        part.setValues(geometryRefinement=EXTRA_FINE)
        print(f"Geometria {self.name} criada. \n")

        # Cria malha
        # part.seedPart(size=discretizacao)
        part.seedEdgeByNumber(edges=part.edges, number=numero_elementos)
        part.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE,offset=0)
        part.PartitionFaceByDatumPlane(faces=part.faces, datumPlane=part.datums[3])
        part.setMeshControls(regions=part.cells, algorithm=MEDIAL_AXIS)
        part.setElementType(regions=((part.faces,)), elemTypes=(mesh.ElemType(elemCode=S4R,elemLibrary=EXPLICIT,hourglassControl=ENHANCED),))
        part.generateMesh()
        print(f"Malha {self.name} criada. \n")

        # Cria material
        passo = pitch(self.radius, self.lay_angle)
        K = 1

        EAeq = self.n*self.young_modulus*self.A/passo
        I2 = (self.Ix+self.Iy)/2-(((self.Iy-self.Ix)**2+4*self.J**2)**(1/2))/2
        EIeq = K*self.n*self.young_modulus*I2/passo
        
        eps=10**(-6)
        a = radians(self.lay_angle)
        c, s = cos(a), sin(a)
        T = array([
                [ c**2,    s**2,   c*s],
                [ s**2,    c**2,  -c*s],
                [-2*c*s,  2*c*s,  c**2 - s**2]
                ])
        A12 = diag([EAeq, eps * EAeq, eps * EAeq])
        D12 = diag([EIeq, eps * EIeq, eps * EIeq])
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

        print(f"Matriz ABD da {self.name} criada. \n")

        # Cria seção e associa material
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

        # Cria assembly e posiciona
        assembly = model.rootAssembly
        assembly.Instance(name=self.name, part=part, dependent=ON)
        assembly.rotate(instanceList=[self.name], axisPoint = (0,0,0), axisDirection =(1, 0, 0), angle = -90)
        g = sk.geometry
        for k in g.keys():
            sk.delete((g[k],))
        print(f"Assembly {self.name} criado. \n")

class cilindros_solidos:
    def __init__(self, name, pitch, mid_diameter, thickness, name_material, young_modulus, poisson, density, plastic_table = None, **kwargs):
        self.name = name
        self.pitch = pitch
        self.mid_diameter = mid_diameter
        self.radius = mid_diameter/2
        self.thickness = thickness
        self.name_material = name_material
        self.young_modulus = young_modulus
        self.poisson = poisson
        self.density = density
        self.plastic_table = plastic_table
    def create_geometry(self, model):
        # Cria geometria
        print("========================================================")
        s = model.ConstrainedSketch(name='rect', sheetSize=200.0)
        s.CircleByCenterPerimeter(center=(0,0), point1=(0, self.radius-self.thickness/2))
        s.CircleByCenterPerimeter(center=(0,0), point1=(0, self.radius+self.thickness/2))
        part = model.Part(name=self.name, dimensionality=THREE_D, type=DEFORMABLE_BODY)
        # part = model.parts[self.name]
        part.BaseSolidExtrude(sketch=s, depth= self.pitch)
        # part.BaseShellExtrude(sketch=s, depth= self.pitch)
        part.setValues(geometryRefinement=EXTRA_FINE)
        print(f"Geometria {self.name} criada. \n")

        # Cria malha
        # part.seedPart(size=discretizacao)
        part.seedEdgeByNumber(edges=part.edges, number=numero_elementos)
        part.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE,offset=0)
        part.PartitionCellByDatumPlane(cells=part.cells, datumPlane=part.datums[3])
        part.setMeshControls(regions=part.cells, algorithm=MEDIAL_AXIS)
        part.setElementType(regions=((part.cells,)), elemTypes=(mesh.ElemType(elemCode=C3D8R,hourglassControl=ENHANCED),))
        part.generateMesh()
        print(f"Malha {self.name} criada. \n")

        # Cria material
        try:
            material = model.materials[self.name_material]
        except:
            material = model.Material(name=self.name_material)
            material.Elastic(table=((self.young_modulus, self.poisson),))
            material.Density(table=((self.density,),))
            if self.plastic_table is not None:
                material.Plastic(table=self.plastic_table) 
        print(f"Material {material.name} criado. \n")

        # Cria seção e associa material
        model.HomogeneousSolidSection(name=f'Section_{self.name}', material=self.name_material)
        region = part.Set(cells=part.cells, name=f'Set-{self.name}')
        part.SectionAssignment(region=region, sectionName=f'Section_{self.name}', offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_SECTION)

        # Cria assembly e posiciona
        assembly = model.rootAssembly
        assembly.Instance(name=self.name, part=part, dependent=ON)
        assembly.rotate(instanceList=[self.name], axisPoint = (0,0,0), axisDirection =(1, 0, 0), angle = -90)
        g = s.geometry
        for k in g.keys():
            s.delete((g[k],))
        print(f"Assembly {self.name} criado. \n")

class tension_armour:
    def __init__(self, name, pitch, lay_angle, n, mid_diameter, base, thickness, name_material, young_modulus, poisson, density, revolve_direction = ON, plastic_table = None, length = 0):
        self.name = name
        self.pitch = pitch
        self.length = length
        self.lay_angle = lay_angle
        self.n = n
        self.mid_diameter = mid_diameter
        self.radius = mid_diameter/2
        self.base = base
        self.thickness = thickness
        self.name_material = name_material
        self.young_modulus = young_modulus
        self.poisson = poisson
        self.density = density
        self.plastic_table = plastic_table
        self.revolve_direction = revolve_direction
    def create_geometry(self, model):
        # Gera geometria
        print("========================================================")
        s = model.ConstrainedSketch(name='rect', sheetSize=200.0)
        s.rectangle(point1=(self.radius-self.thickness/2,0), point2=(self.radius+self.thickness/2, self.base/sin(radians(self.lay_angle))))
        #/sin(radians(self.lay_angle))
        g = s.geometry
        s.ConstructionLine(point1=(0.0, -self.length), point2=(0.0, self.length))
        part = model.Part(name=self.name, dimensionality=THREE_D, type=DEFORMABLE_BODY)
        p = model.parts[self.name]
        factor = 1.1
        p.BaseSolidRevolve(sketch=s, angle=factor*self.length/self.pitch*360.0, flipRevolveDirection=self.revolve_direction, pitch=self.pitch, flipPitchDirection = OFF, moveSketchNormalToPath=OFF)
        p.setValues(geometryRefinement=EXTRA_FINE)
        for k in g.keys():
            s.delete((g[k],))
        
        # Apaga o excesso da armadura de tração
        planes = [(factor-1)/2, (factor+1)/2]
        extrude_dir = [ON, OFF]
        for i, (plan, dir) in enumerate(list(zip(planes, extrude_dir))):
            p.DatumPlaneByPrincipalPlane(principalPlane=XZPLANE, offset = -plan*self.length)
            datums = p.datums.values()
            ns = model.ConstrainedSketch(gridSpacing=63.45, name='novo_sketch', sheetSize=2600, transform=p.MakeSketchTransform(sketchPlane=datums[i+1], sketchPlaneSide=SIDE1, sketchUpEdge=p.edges[6], sketchOrientation=RIGHT, origin=(0.0, self.length, 0.0)))
            ns.rectangle(point1=(-200, 200), point2=(200, -200))
            
            p.CutExtrude(
                flipExtrudeDirection=dir, 
                sketch=ns, 
                sketchOrientation=RIGHT, 
                sketchPlane=datums[i+1], 
                sketchPlaneSide=SIDE1, 
                sketchUpEdge=p.edges[6])
            del ns
        
        # Ajusta a partição automática do Abaqus para possibilitar uma malha estruturada
        comprimento_minimo = 50.0
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

        print(f"Geometria {self.name} criada. \n")

        # Gera malha
        # p.seedPart(size=discretizacao_tension_armour)
        part.seedEdgeByNumber(edges=[a for a in part.edges if a.getSize()>50], number=int(self.length/(self.base/sin(radians(self.lay_angle)))))
        p.setElementType(regions=(p.cells[0],), elemTypes=(mesh.ElemType(elemCode=C3D8R, hourglassControl=ENHANCED),))
        p.generateMesh()
        print(f"Malha {self.name} criada. \n")

        try:
            material = model.materials[self.name_material]
        except:
            material = model.Material(name=f'{self.name_material}')
            material.Elastic(table=((self.young_modulus, self.poisson),))
            material.Density(table=((self.density,),))

            print(f"Material {material.name} criado.\n")

        # Cria seção e associa material
        model.HomogeneousSolidSection(name=f'Section_{self.name_material}', material=self.name_material, thickness=None)
        region = p.Set(cells=p.cells, name=f'Set-{self.name}')
        p.SectionAssignment(region=region, sectionName=f'Section_{self.name_material}', offset=0.0, offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_SECTION)

        # Cria o assembly e as cópias
        assembly = model.rootAssembly
        instance_name = f'{self.name}-1'
        assembly.Instance(name=instance_name, part=part, dependent=ON)
        assembly.translate(instanceList=(instance_name,), vector=(0,self.length,0))
        num_instances = self.n
        angle_step = 360.0 / num_instances

        for j in range(0, num_instances):
            rotation_angle = j * angle_step
            instance_rotated_name = f'{self.name}-{j+1}'
            assembly.Instance(name=instance_rotated_name, part=part, dependent=ON)
            assembly.rotate(instanceList=(instance_rotated_name,), axisPoint=(0, 0, 0), axisDirection=(0, 1, 0), angle=rotation_angle)
            assembly.translate(instanceList=(instance_rotated_name,), vector=(0,(factor+1)/2*self.length,0))
        
        for k in g.keys():
            s.delete((g[k],))

        # assembly.suppressFeatures((instance_name,))
        print(f"Assembly {self.name} criado. \n")

class carcaca:
    def __init__(self, name, pitch, lay_angle, n, mid_diameter, thickness, name_material, young_modulus, poisson, density, revolve_direction = ON, plastic_table = None, length = 0, **kwargs):
        self.name = name
        self.lay_angle = lay_angle
        self.n = n
        self.mid_diameter = mid_diameter
        self.thickness = thickness
        self.pitch = pitch
        self.length = pitch
        self.radius = mid_diameter/2
        self.name_material = name_material
        self.young_modulus = young_modulus
        self.poisson = poisson
        self.density = density
        self.plastic_table = plastic_table
        self.revolve_direction = revolve_direction
    def create_geometry(self, model):
        # Gera geometria
        self.pitch = pitch(self.mid_diameter/2-self.thickness/2, self.lay_angle)
        print("========================================================")
        s = model.ConstrainedSketch(name='carcaca', sheetSize=200.0)
        # points = [(1.75, 0.35), (-1.05, 0.35), (-1.05, 6.8), (-0.35, 6.8), (-0.35, 3.45), (0.35, 3.45), (0.35, 7.5), (-1.75, 7.5), (-1.75, -0.35), (1.05, -0.35), (1.05, -6.8), (-0.35, -6.8), (-0.35, -3.45), (-1.05, -3.45), (-1.05, -7.5), (1.75, -7.5), (1.75, 0.35)]

        points = [(0.95, 7.5), (0.95, 5.45), (0.25, 5.45), (0.25, 6.8), (-1.05, 6.8), (-1.05, 0.35), (1.75, 0.35), (1.75, -7.5), (-0.95, -7.5), (-0.95, -5.45), (-0.25, -5.45), (-0.25, -6.8), (1.05, -6.8), (1.05, -0.35), (-1.75, -0.35), (-1.75, 7.5), (0.95, 7.5)]

        for i in range(0, len(points)):
            r = self.mid_diameter/2
            s.Line(point1=(points[i-1][0]+r, points[i-1][1]+7.5), point2=(points[i][0]+r, points[i][1]+7.5))
        g = s.geometry
        s.ConstructionLine(point1=(0.0, -10*self.pitch), point2=(0.0, 10*self.pitch))
        part = model.Part(name=self.name, dimensionality=THREE_D, type=DEFORMABLE_BODY)
        p = model.parts[self.name]

        p.BaseSolidRevolve(sketch=s, angle=(self.length/self.pitch)*360.0, flipRevolveDirection=self.revolve_direction, pitch=self.pitch, flipPitchDirection = OFF, moveSketchNormalToPath=ON)
        p.setValues(geometryRefinement=EXTRA_FINE)
        # for k in g.keys():
        #     s.delete((g[k],))
        
        print(f"Geometria {self.name} criada. \n")

        # Gera malha
        p.seedPart(size=numero_elementos)
        p.setElementType(regions=(p.cells[0],), elemTypes=(mesh.ElemType(elemCode=C3D8R),))
        p.generateMesh()
        print(f"Malha {self.name} criada. \n")

        try:
            material = model.materials[self.name_material]
        except:
            material = model.Material(name=f'{self.name_material}')
            material.Elastic(table=((self.young_modulus, self.poisson),))
            material.Density(table=((self.density,),))

            print(f"Material {material.name} criado.\n")

        # Cria seção e associa material
        model.HomogeneousSolidSection(name=f'Section_{self.name_material}', material=self.name_material, thickness=None)
        region = p.Set(cells=p.cells, name=f'Set-{self.name}')
        p.SectionAssignment(region=region, sectionName=f'Section_{self.name_material}', offset=0.0, offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_SECTION)

        # Cria o assembly e as cópias
        assembly = model.rootAssembly
        instance_name = f'{self.name}-1'
        assembly.Instance(name=instance_name, part=part, dependent=ON)
        # assembly.translate(instanceList=(instance_name,), vector=(0,self.length,0))
        # num_instances = 2
        # angle_step = 360.0 / num_instances

        # for j in range(0, num_instances):
        #     rotation_angle = j * angle_step
        #     instance_rotated_name = f'{self.name}-{j+1}'
        #     assembly.Instance(name=instance_rotated_name, part=part, dependent=ON)
        #     assembly.rotate(instanceList=(instance_rotated_name,), axisPoint=(0, 0, 0), axisDirection=(0, 1, 0), angle=rotation_angle)
        #     assembly.translate(instanceList=(instance_rotated_name,), vector=(0,(factor+1)/2*self.length, 0))
        
        # for k in g.keys():
        #     s.delete((g[k],))

        # assembly.suppressFeatures((instance_name,))
        print(f"Assembly {self.name} criado. \n")

class pressure_armour:
    def __init__(self, name, pitch, lay_angle, n, mid_diameter, thickness, name_material, young_modulus, poisson, density, revolve_direction = ON, plastic_table = None, length = 0, **kwargs):
        self.name = name
        self.lay_angle = lay_angle
        self.n = n
        self.mid_diameter = mid_diameter
        self.thickness = thickness
        self.pitch = pitch
        self.length = pitch
        self.radius = mid_diameter/2
        self.name_material = name_material
        self.young_modulus = young_modulus
        self.poisson = poisson
        self.density = density
        self.plastic_table = plastic_table
        self.revolve_direction = revolve_direction
    def create_geometry(self, model):
        # Gera geometria
        self.pitch = pitch(self.mid_diameter/2-self.thickness/2, self.lay_angle)
        print("========================================================")
        s = model.ConstrainedSketch(name='pressure_section', sheetSize=200.0)
        points = [(-3.1, 2.155), (-3.1, -7.095), (-0.95, -7.095), (-0.95, -2.155), (3.1, -2.155), (3.1, 7.095), (0.95, 7.095), (0.95, 2.155), (-3.1, 2.155)]

        for i in range(0, len(points)):
            r = self.mid_diameter/2
            s.Line(point1=(points[i-1][0]+r, points[i-1][1]), point2=(points[i][0]+r, points[i][1]))

        g = s.geometry
        s.ConstructionLine(point1=(0.0, -10*self.pitch), point2=(0.0, 10*self.pitch))
        part = model.Part(name=self.name, dimensionality=THREE_D, type=DEFORMABLE_BODY)
        p = model.parts[self.name]
        factor = 1

        p.BaseSolidRevolve(sketch=s, angle=factor*self.length/self.pitch*360.0, flipRevolveDirection=self.revolve_direction, pitch=self.pitch, flipPitchDirection = OFF, moveSketchNormalToPath=ON)
        p.setValues(geometryRefinement=EXTRA_FINE)

        print(f"Geometria {self.name} criada. \n")

        # Gera malha
        p.seedPart(size=numero_elementos)
        p.setElementType(regions=(p.cells[0],), elemTypes=(mesh.ElemType(elemCode=C3D8R),))
        p.generateMesh()
        print(f"Malha {self.name} criada. \n")

        try:
            material = model.materials[self.name_material]
        except:
            material = model.Material(name=f'{self.name_material}')
            material.Elastic(table=((self.young_modulus, self.poisson),))
            material.Density(table=((self.density,),))

            print(f"Material {material.name} criado.\n")

        # Cria seção e associa material
        model.HomogeneousSolidSection(name=f'Section_{self.name_material}', material=self.name_material, thickness=None)
        region = p.Set(cells=p.cells, name=f'Set-{self.name}')
        p.SectionAssignment(region=region, sectionName=f'Section_{self.name_material}', offset=0.0, offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_SECTION)

        # Cria o assembly e as cópias
        assembly = model.rootAssembly
        instance_name = f'{self.name}-1'
        assembly.Instance(name=instance_name, part=part, dependent=ON)
        assembly.translate(instanceList=(instance_name,), vector=(0,self.length,0))
        num_instances = 2
        angle_step = 360.0 / num_instances

        for j in range(0, num_instances):
            rotation_angle = j * angle_step
            instance_rotated_name = f'{self.name}-{j+1}'
            assembly.Instance(name=instance_rotated_name, part=part, dependent=ON)
            assembly.rotate(instanceList=(instance_rotated_name,), axisPoint=(0, 0, 0), axisDirection=(0, 1, 0), angle=rotation_angle)
            assembly.translate(instanceList=(instance_rotated_name,), vector=(0,(factor+1)/2*self.length,0))
        
        for k in g.keys():
            s.delete((g[k],))

        print(f"Assembly {self.name} criado. \n")