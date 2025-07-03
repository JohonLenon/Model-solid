from caeModules import mesh
from abaqusConstants import * # type: ignore
from numpy import pi, sin, cos, sign

class cilindros:
    def __init__(self, name, pitch, internal_diameter, thickness, name_material, young_modulus, poisson, density, plastic_table = None):
        self.name = name
        self.pitch = pitch
        self.internal_diameter = internal_diameter
        self.radius = internal_diameter/2
        self.thickness = thickness
        self.name_material = name_material
        self.young_modulus = young_modulus
        self.poisson = poisson
        self.density = density
        self.plastic_table = plastic_table
    def create_geometry(self, model, s):
        # Cria geometria
        s.CircleByCenterPerimeter(center=(0,0), point1=(0, self.radius))
        part = model.Part(name=self.name, dimensionality=THREE_D, type=DEFORMABLE_BODY)
        shell_part = model.parts[self.name]
        shell_part.BaseShellExtrude(sketch=s, depth= self.pitch)
        shell_part.setValues(geometryRefinement=EXTRA_FINE)
        print(f"Geometria {self.name} criada. \n")

        # Cria malha
        shell_part.seedPart(size=5)
        shell_part.setElementType(regions=((shell_part.faces,)), elemTypes=(mesh.ElemType(elemCode=S4),))
        shell_part.generateMesh()
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
        region = shell_part.Set(faces=shell_part.faces, name=f'Set-{self.name}')
        shell_part.SectionAssignment(region=region, sectionName=f'Section_{self.name}', offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_SECTION)

        # Cria assembly e posiciona
        assembly = model.rootAssembly
        assembly.Instance(name=self.name, part=part, dependent=ON)
        assembly.rotate(instanceList=[self.name], axisPoint = (0,0,0), axisDirection =(1, 0, 0), angle = -90)
        g = s.geometry
        for k in g.keys():
            s.delete((g[k],))
        print(f"Assembly {self.name} criado. \n")

class tension_armour:
    def __init__(self, name, pitch, lay_angle, number_strips, internal_diameter, height, width, points, name_material, young_modulus, poisson, density, plastic_table = None, length = 0):
        self.name = name
        self.pitch = pitch
        self.length = max(pitch, length)
        self.lay_angle = lay_angle
        self.number_strips = number_strips
        self.internal_diameter = internal_diameter
        self.radius = internal_diameter/2
        self.height = height
        self.width = width
        self.points = points
        self.name_material = name_material
        self.young_modulus = young_modulus
        self.poisson = poisson
        self.density = density
        self.plastic_table = plastic_table
    def create_geometry(self, model):
        pontos = []
        for i in range(self.points + 1):
            position_ratio = i / self.points
            t = 2 * pi * position_ratio * self.length / self.pitch * sign(self.lay_angle)
            x = self.radius * cos(t)
            z = self.radius * sin(t)
            y = self.length * position_ratio
            if y > self.length:
                break
            pontos.append((x, y, z))

        pontos = tuple(pontos)
        part = model.Part(name=self.name, dimensionality=THREE_D, type=DEFORMABLE_BODY)
        p = model.parts[self.name]
        p.WireSpline(points=pontos)

        print(f"Geometria {self.name} criada.\n")

        # Define a orientação da viga
        region = p.Set(edges=p.edges, name=f'Set-{self.name}')
        part.assignBeamSectionOrientation(region=region, method=N1_COSINES, n1=(0.0, 1.0, 0.0))
        print(f"Orientação da viga {self.name} definida. \n")

        # Gera malha
        p.seedEdgeByNumber(edges=p.edges, number=self.points)
        p.setElementType(regions=(p.edges,), elemTypes=(mesh.ElemType(elemCode=B31),))
        p.generateMesh()
        print(f"Malha {self.name} criada. \n")

        # Cria perfil, seção e associa material
        model.RectangularProfile(name= self.name+'_Profile', a=self.width, b=self.height)
        model.BeamSection(name= self.name+'_Section', integration=DURING_ANALYSIS, profile= self.name+'_Profile', material='METALLICMaterial')
        p.SectionAssignment(region=region, sectionName= self.name+'_Section', thicknessAssignment=FROM_SECTION)
        print(f"Perfil, seção e materiais para {self.name} atribuídos. \n")

        # Cria o assembly e as cópias
        assembly = model.rootAssembly
        instance_name = f'{self.name}-1'
        assembly.Instance(name=instance_name, part=part, dependent=ON)
        angle_step = 360.0 / self.number_strips
        for i in range(1, self.number_strips):
            rotation_angle = i * angle_step
            instance_rotated_name = f'{self.name}-{i+1}'
            assembly.Instance(name=instance_rotated_name, part=part, dependent=ON)
            assembly.rotate(instanceList=(instance_rotated_name,), axisPoint=(0, 0, 0), axisDirection=(0, 1, 0), angle=rotation_angle)
        print(f"Assembly {self.name} criado.\n")