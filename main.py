import os
import sys
sys.path.append(os.getcwd())

from create_material import create_material
from create_tension_armour_beam import create_tension_armour_beam
from create_cilindros import create_cilindros
from create_contact import create_contact
from create_analysis import create_analysis

create_material()
create_tension_armour_beam()
create_cilindros()
create_contact()
create_analysis()