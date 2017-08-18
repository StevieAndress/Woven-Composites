from part import *
from material import *
from section import *
from assembly import *
from step import *
from interaction import *
from load import *
from mesh import *
from optimization import *
from job import *
from sketch import *
from visualization import *
from connectorBehavior import *
import math
import csv
import regionToolset

m = 3#number of warp fibers
n = 3#number of weft fibers
f1 = .25 #space between each warp fiber
f2 = .25 #space between each weft fiber
r1 = .0275#radius of each warp
r2 = .0275#radius of each weft
C1 = 431650000.0 #c10 parameter
C2 = 431650000.0
D1 = 5.5e-10 #D10 parameter
D2 = 5.5e-10
MS1 = .02 #mesh size
MS2 = .02
fric = 0.3

#calculate the length of each fiber
l1 = n * f2
l2 = m *f1
disz = l1/3 #displacement boundary condition in z direction
disx = 0 #displacement boundary condition in x direction

weave = mdb.models['Model-1']

#Create Geometry------------------------------------------------------

#Create warp geometry
weave.ConstrainedSketch(name='__profile__', sheetSize=l1)
weave.sketches['__profile__'].CircleByCenterPerimeter(center=(
    0.0, 0.0), point1=(0.0, r1))
warp = weave.Part(dimensionality=THREE_D, name='warp', type=
    DEFORMABLE_BODY)
warp.BaseSolidExtrude(depth=l1, sketch=weave.sketches['__profile__'])
del weave.sketches['__profile__']

#partition warp horizontaly
warp.PartitionCellByPlaneThreePoints(cells=
    warp.cells.findAt((0,0,0),), point1=(0,0,0), point2=(0,0,l1),
    point3=(r1,0,l1))

#partition warp into n sections
for i in range (1,n):
  warp.PartitionCellByPlaneThreePoints(cells=
    warp.cells.getSequenceFromMask(('[#3 ]', ), ),
    point1=(0,0,i*f2), point2=(0,r1,i*f2),point3=(r1,0,i*f2))

#partion warp vertically
warp.PartitionCellByPlaneThreePoints(cells=warp.cells,
    point1=(0,0,0), point2=(0,0,l1),point3=(0,r1,l1))

#Create weft geometry
weave.ConstrainedSketch(name='__profile__', sheetSize=l1)
weave.sketches['__profile__'].CircleByCenterPerimeter(center=(
    0.0, 0.0), point1=(0.0, r2))
weft = weave.Part(dimensionality=THREE_D, name='weft', type=
    DEFORMABLE_BODY)
weft.BaseSolidExtrude(depth=l2, sketch=weave.sketches['__profile__'])
del weave.sketches['__profile__']

#partition weft horizontaly
weft.PartitionCellByPlaneThreePoints(cells=
    weft.cells.findAt((0,0,0),), point1=(0,0,0), point2=(0,0,l2),
    point3=(r2,0,l2))

#partition weft into m sections
for i in range (1,m):
  weft.PartitionCellByPlaneThreePoints(cells=
    weft.cells.getSequenceFromMask(('[#3 ]', ), ),
    point1=(0,0,i*f1), point2=(0,r2,i*f1),point3=(r2,0,i*f1))
  
#partion weft vertically
weft.PartitionCellByPlaneThreePoints(cells=weft.cells,
    point1=(0,0,0), point2=(0,0,l2),point3=(0,r2,l2))

#define materials--------------------------------------------------------

warpcells=[]
for i in range (0, n):
  warpcells.append(warp.cells.findAt(((r1/2,r1/2,i*f2+f2/2),),((r1/2,-r1/2,i*f2+f2/2),),
	((-r1/2,r1/2,i*f2+f2/2),),((-r1/2,-r1/2,i*f2+f2/2),)))
tuple(warpcells)

weftcells=[]
for i in range (0, m):
  weftcells.append(weft.cells.findAt(((r2/2,r2/2,i*f1+f1/2),),((r2/2,-r2/2,i*f1+f1/2),),
	((-r2/2,r2/2,i*f1+f1/2),),((-r2/2,-r2/2,i*f1+f1/2),)))
tuple(weftcells)

#Define warp material
mat1 = weave.Material(name='Warp Material')
mat1.Hyperelastic(materialType=ISOTROPIC, table=((C1, D1), ),
    testData=OFF, type=NEO_HOOKE, volumetricResponse=VOLUMETRIC_DATA)

#create & assign section to warp
weave.HomogeneousSolidSection(material='Warp Material', name=
    'Section-1', thickness=None)
warp.Set(cells=warpcells, name='Set-1')
warp.SectionAssignment(offset=0.0, 
    offsetField='', offsetType=MIDDLE_SURFACE, region=warp.sets['Set-1'],
    sectionName='Section-1', thicknessAssignment=FROM_SECTION)


#Define weft material
mat2 = weave.Material(name='Weft Material')
mat2.Hyperelastic(materialType=ISOTROPIC, table=((C2, D2), ),
    testData=OFF, type=NEO_HOOKE, volumetricResponse=VOLUMETRIC_DATA)

#create & assign section to weft
weave.HomogeneousSolidSection(material='Weft Material', name=
    'Section-2', thickness=None)
weft.Set(cells=weftcells, name='Set-1')
weft.SectionAssignment(offset=0.0, 
    offsetField='', offsetType=MIDDLE_SURFACE, region=weft.sets['Set-1'],
    sectionName='Section-2', thicknessAssignment=FROM_SECTION)

#create assembly---------------------------------------------------------

weave.rootAssembly.DatumCsysByDefault(CARTESIAN)

#insert warp instances
for i in range (0,m):
  weave.rootAssembly.Instance(dependent=ON, name='warp ' +str(i+1), 
    part=warp)
  weave.rootAssembly.instances['warp '+str(i+1)].translate(vector=(
   (i+.5)*f1 , 0.0,0.0 ))
  
#insert weft instances
for i in range (0,n):
  weave.rootAssembly.Instance(dependent=ON, name='weft ' +str(i+1), 
    part=weft)
  weave.rootAssembly.rotate(angle=90, axisDirection=(0, 1, 0),
    axisPoint=(0.0, 0.0, 0.0), instanceList = ('weft '+str(i+1),))
  weave.rootAssembly.instances['weft '+str(i+1)].translate(vector=(
    0,0,(i+.5)*f2 ))  
  
#Apply Mesh--------------------------------------------------------------

warp.seedPart(deviationFactor=0.1, minSizeFactor=0.1, size=MS1)
warp.setElementType(elemTypes=(ElemType(elemCode=C3D20, elemLibrary=STANDARD),
    ElemType(elemCode=C3D15, elemLibrary=STANDARD),ElemType(elemCode=C3D10,
    elemLibrary=STANDARD)), regions=warpcells)
warp.generateMesh()

weft.seedPart(deviationFactor=0.1, minSizeFactor=0.1, size=MS2)
weft.setElementType(elemTypes=(ElemType(elemCode=C3D20, elemLibrary=STANDARD),
    ElemType(elemCode=C3D15, elemLibrary=STANDARD), ElemType(elemCode=C3D10, 
    elemLibrary=STANDARD)), regions=weftcells)
weft.generateMesh()
    
weave.rootAssembly.regenerate()
  
#Step 1, displace warp----------------------------------------------

weave.StaticStep(initialInc=1, maxInc=1.0, name='Step-1', 
    previous='Initial')
weave.steps['Step-1'].setValues(nlgeom=ON)

#field and history output request
weave.FieldOutputRequest(createStepName='Step-1', name=
    'F-Output-1', variables=PRESELECT)
weave.fieldOutputRequests['F-Output-1'].setValues(variables=(
    'S', 'PE', 'PEEQ', 'PEMAG', 'LE', 'U', 'RF', 'CF', 'CSTATUS','COORD'))

weave.HistoryOutputRequest(createStepName='Step-1', name=
    'H-Output-1', variables=PRESELECT)

#set boundary conditions

#constrain weft
wefts=[]

for i in range (0,n):
  wefts.append(weave.rootAssembly.sets['weft '+str(i+1)+'.Set-1'])
  
tuple(wefts)
weave.rootAssembly.SetByBoolean(sets=wefts, name='wefts')

weave.EncastreBC(createStepName='Step-1', localCsys=None, name=
    'BC-1', region=weave.rootAssembly.sets['wefts'])

#displace warp
#create set containing alternating fibers

over=[]
under=[]

for i in range (0,m):
  if i % 2 == 0:
    over.append(weave.rootAssembly.sets['warp '+str(i+1)+'.Set-1'])

  else:
    under.append(weave.rootAssembly.sets['warp '+str(i+1)+'.Set-1'])


tuple(over)
weave.rootAssembly.SetByBoolean(sets=over, name='over')

tuple(under)
weave.rootAssembly.SetByBoolean(sets=under, name='under')

 
#define weave pattern
mdb.models['Model-1'].ExpressionField(description='', expression=
    'sin( Z * pi / f2)', localCsys=None, name='AnalyticalField-1')

#apply displacement 
weave.DisplacementBC(amplitude=UNSET, createStepName='Step-1', 
    distributionType=FIELD, fieldName='AnalyticalField-1', fixed=OFF, 
    localCsys=None, name='BC-2', region=
    weave.rootAssembly.sets['over'], u1=0.0, u2=(r1+r2+.001), u3=0.0, 
    ur1=UNSET, ur2=UNSET, ur3=UNSET)
weave.DisplacementBC(amplitude=UNSET, createStepName='Step-1', 
    distributionType=FIELD, fieldName='AnalyticalField-1', fixed=OFF, 
    localCsys=None, name='BC-3', region=
    weave.rootAssembly.sets['under'], u1=0.0, u2=-(r1+r2+.001), u3=0.0, 
    ur1=UNSET, ur2=UNSET, ur3=UNSET)

#step 2, establish contact---------------------------------------------

#create step
weave.StaticStep(initialInc=.1,minInc=1e-12, maxInc=1.0, name='Step-2', 
    maxNumInc=10000, previous='Step-1')  

#create sets of alternating slave surfaces
slave=[]
for i in range (0,n):
  
  for j in range (0,m):
    if i%2==0:
      
      if j%2==0:
	slave.append(weave.rootAssembly.instances[
	  'weft ' + str(i+1)].faces.findAt(((j*f1+f1/2,r2*sin(pi/4),
		i*f2+f2/2+r2*sin(pi/4)), ),((j*f1+f1/2,r2*sin(pi/4),
		i*f2+f2/2-r2*sin(pi/4) ), ),))
      else:
	slave.append(weave.rootAssembly.instances[
	  'weft ' + str(i+1)].faces.findAt(((j*f1+f1/2,-r2*.707107,
		i*f2+f2/2+r2*sin(pi/4)), ),((j*f1+f1/2,-r2*.707107,
		i*f2+f2/2-r2*sin(pi/4)), ),))

    else:
      
      if j%2==0:
	slave.append(weave.rootAssembly.instances[
	  'weft ' + str(i+1)].faces.findAt(((j*f1+f1/2,-r2*sin(pi/4),
		i*f2+f2/2+r2*sin(pi/4)), ),((j*f1+f1/2,-r2*sin(pi/4),
		i*f2+f2/2-r2*sin(pi/4) ), ),))

      else:
	slave.append(weave.rootAssembly.instances[
	  'weft ' + str(i+1)].faces.findAt(((j*f1+f1/2,r2*.707107,
		i*f2+f2/2+r2*sin(pi/4)), ),((j*f1+f1/2,r2*.707107,
		i*f2+f2/2-r2*sin(pi/4)), ),))
	
tuple(slave)    
weave.rootAssembly.Surface(name='s_Surf-1',side1Faces=slave[0:])

#create sets of alternating master surfaces
master=[]
for i in range (0,m):
  
  centerline=[] #create an array of edges on warp centerline
  
  for j in range (0,n):
    
    centerline.append(weave.rootAssembly.instances[
      'warp ' + str(i+1)].edges.findAt(((i*f1+f1/2,0,j*f2+f2/2),),))
    
    if i%2==0:
     
      if j%2==0:
	master.append(weave.rootAssembly.instances[
	  'warp ' + str(i+1)].faces.findAt(((i*f1+f1/2+r1*sin(pi/4),
	  -r1*sin(pi/4),j*f2+f2/2),),((i*f1+f1/2-r1*sin(pi/4),
	  -r1*sin(pi/4),j*f2+f2/2),),))
      else:
	master.append(weave.rootAssembly.instances[
	  'warp ' + str(i+1)].faces.findAt(((i*f1+f1/2+r1*sin(pi/4),
	  r1*sin(pi/4),j*f2+f2/2),),((i*f1+f1/2-r1*sin(pi/4),
	  r1*sin(pi/4),j*f2+f2/2),),))
    else:
     
      if j%2==0:
	master.append(weave.rootAssembly.instances[
	  'warp ' + str(i+1)].faces.findAt(((i*f1+f1/2+r1*sin(pi/4),
	  r1*sin(pi/4),j*f2+f2/2),),((i*f1+f1/2-r1*sin(pi/4),
	  r1*sin(pi/4),j*f2+f2/2),),))
      else:
	master.append(weave.rootAssembly.instances[
	  'warp ' + str(i+1)].faces.findAt(((i*f1+f1/2+r1*sin(pi/4),
	  -r1*sin(pi/4),j*f2+f2/2),),((i*f1+f1/2-r1*sin(pi/4),
	  -r1*sin(pi/4),j*f2+f2/2),),))
	  
  tuple(centerline)
  weave.rootAssembly.Set(name = 'centerline'+str(i),edges=centerline)
 
tuple(master)    
weave.rootAssembly.Surface(name='m_Surf-1',side1Faces=master[0:])

#Define contact properties
weave.ContactProperty('IntProp-1')
weave.interactionProperties['IntProp-1'].TangentialBehavior(dependencies=0,
    directionality=ISOTROPIC, elasticSlipStiffness=None, 
    formulation=PENALTY, fraction=0.005, maximumElasticSlip=FRACTION, 
    pressureDependency=OFF, shearStressLimit=None, slipRateDependency=OFF, 
    table=((fric, ), ), temperatureDependency=OFF)
weave.interactionProperties['IntProp-1'].NormalBehavior(
    allowSeparation=OFF, constraintEnforcementMethod=DEFAULT, 
    pressureOverclosure=HARD)
weave.StdContactControl(name='ContCtrl-1')
mdb.models['Model-1'].contactControls['ContCtrl-1'].setValues(stabilizeChoice=
    AUTOMATIC)

#apply contact
weave.SurfaceToSurfaceContactStd(adjustMethod=NONE, 
    clearanceRegion=None, createStepName='Step-2', datumAxis=None, 
    initialClearance=OMIT, interactionProperty='IntProp-1', master=
    weave.rootAssembly.surfaces['m_Surf-1'], name='Int-1', 
    slave=weave.rootAssembly.surfaces['s_Surf-1'], sliding=
    FINITE, surfaceSmoothing=AUTOMATIC, thickness=ON)
mdb.models['Model-1'].interactions['Int-1'].setValues(adjustMethod=NONE, 
    bondingSet=None, contactControls='ContCtrl-1', contactTracking=TWO_CONFIG, 
    enforcement=SURFACE_TO_SURFACE, initialClearance=OMIT, sliding=FINITE, 
    thickness=ON)

#modify boundary conditions

#release displacement BC
weave.boundaryConditions['BC-2'].deactivate('Step-2')
weave.boundaryConditions['BC-3'].deactivate('Step-2')


#create rigid surface for end of warp fiber
ends=[]
refset1=[]
refset2=[]
for i in range (0,m):

  end1=weave.rootAssembly.instances['warp ' 
    + str(i+1)].faces.findAt(((i*f1+f1/2+r1/2,r1/2,0),),
    ((i*f1+f1/2-r1/2,r1/2,0),),((i*f1+f1/2+r1/2,-r1/2,0),),
    ((i*f1+f1/2-r1/2,-r1/2,0),))
  weave.rootAssembly.Set(faces = end1, name = ('warp '+str(i+1)+'-end1'))
  end2=weave.rootAssembly.instances['warp ' 
    + str(i+1)].faces.findAt(((i*f1+f1/2+r1/2,r1/2,l1),),
    ((i*f1+f1/2-r1/2,r1/2,l1),),((i*f1+f1/2+r1/2,-r1/2,l1),),
    ((i*f1+f1/2-r1/2,-r1/2,l1),))
  weave.rootAssembly.Set(faces = end2, name = ('warp '+str(i+1)+'-end2'))
  
  pt1=weave.rootAssembly.ReferencePoint(point=(i*f1+f1/2,0,0))
  pt2=weave.rootAssembly.ReferencePoint(point=(i*f1+f1/2,0,l1))
  ends.append(weave.rootAssembly.instances['warp ' 
    + str(i+1)].vertices.findAt(((i*f1+f1/2,0,l1),),))
  tuple(ends)
  weave.rootAssembly.Set(vertices = ends, name = 'ends')
  weave.RigidBody(name='Constraint1-'+str(i+1)+'-1', refPointRegion=Region(
    referencePoints=(weave.rootAssembly.referencePoints[pt1.id],
    )), tieRegion=weave.rootAssembly.sets['warp '+str(i+1)+'-end1'])
  weave.RigidBody(name='Constraint1-'+str(i+1)+'-2', refPointRegion=Region(
    referencePoints=(weave.rootAssembly.referencePoints[pt2.id],
    )), tieRegion=weave.rootAssembly.sets['warp '+str(i+1)+'-end2'])
  refset1.append(weave.rootAssembly.referencePoints[pt1.id])
  refset2.append(weave.rootAssembly.referencePoints[pt2.id])  


#constrain motion of end points    
weave.rootAssembly.Set(name='RefSet1', referencePoints=tuple(refset1))
weave.rootAssembly.Set(name='RefSet2', referencePoints=tuple(refset2))
weave.DisplacementBC(amplitude=UNSET, createStepName='Step-2', 
    distributionType=UNIFORM, fieldName='', fixed=OFF, localCsys=None, name=
    'BC-5', region=weave.rootAssembly.sets['RefSet1'], u1=0.0, 
    u2=0.0, u3=0.0, ur1=UNSET, ur2=UNSET, ur3=0.0)
weave.DisplacementBC(amplitude=UNSET, createStepName='Step-2', 
    distributionType=UNIFORM, fieldName='', fixed=OFF, localCsys=None, name=
    'BC-6', region=weave.rootAssembly.sets['ends'], u1=0.0, 
    u2=0.0, u3=0.0, ur1=UNSET, ur2=UNSET, ur3=0.0)

#step 3, release weft---------------------------------------------

mdb.models['Model-1'].StaticStep(initialInc=.1, maxInc=1, minInc=1e-12, 
   maxNumInc=1000, name='Step-3', previous='Step-2')

#deactivate interaction 1
weave.interactions['Int-1'].deactivate('Step-3')

#switch contact master and slave surface
mdb.models['Model-1'].SurfaceToSurfaceContactStd(adjustMethod=NONE, 
    clearanceRegion=None, contactControls='ContCtrl-1', createStepName='Step-3'
    , datumAxis=None, initialClearance=OMIT, interactionProperty='IntProp-1', 
    master=mdb.models['Model-1'].rootAssembly.surfaces['s_Surf-1'], name=
    'Int-2', slave=mdb.models['Model-1'].rootAssembly.surfaces['m_Surf-1'], 
    sliding=FINITE, surfaceSmoothing=AUTOMATIC, thickness=ON)

#deactivate weft BC
weave.boundaryConditions['BC-1'].deactivate('Step-3')

#modify warp BC
#mdb.models['Model-1'].boundaryConditions['BC-5'].setValuesInStep(stepName=
#    'Step-3', ur2=FREED, ur3=FREED)
#mdb.models['Model-1'].boundaryConditions['BC-6'].setValuesInStep(stepName=
#    'Step-3', ur2=FREED, ur3=FREED)

#create rigid surface for ends of weft fibers
ends=[]
refset1=[]
refset2=[]
for i in range (0,n):

  end1=weave.rootAssembly.instances['weft ' 
    + str(i+1)].faces.findAt(((0,r1/2,i*f2+f2/2+r2/2),),
    ((0,r2/2,i*f2+f2/2-r2/2),),((0,-r2/2,i*f2+f2/2+r2/2),),
    ((0,-r2/2,i*f2+f2/2-r2/2),))
  weave.rootAssembly.Set(faces = end1, name = ('weft '+str(i+1)+'-end1'))
  end2=weave.rootAssembly.instances['weft ' 
    + str(i+1)].faces.findAt(((l2,r1/2,i*f2+f2/2+r2/2),),
    ((l2,r2/2,i*f2+f2/2-r2/2),),((l2,-r2/2,i*f2+f2/2+r2/2),),
    ((l2,-r2/2,i*f2+f2/2-r2/2),))
  weave.rootAssembly.Set(faces = end2, name = ('weft '+str(i+1)+'-end2'))
  
  pt1=weave.rootAssembly.ReferencePoint(point=(0,0,i*f2+f2/2))
  pt2=weave.rootAssembly.ReferencePoint(point=(l2,0,i*f2+f2/2))
  weave.RigidBody(name='Constraint2-'+str(i+1)+'-1', refPointRegion=Region(
    referencePoints=(weave.rootAssembly.referencePoints[pt1.id],
    )), tieRegion=weave.rootAssembly.sets['weft '+str(i+1)+'-end1'])
  weave.RigidBody(name='Constraint2-'+str(i+1)+'-2', refPointRegion=Region(
    referencePoints=(weave.rootAssembly.referencePoints[pt2.id],
    )), tieRegion=weave.rootAssembly.sets['weft '+str(i+1)+'-end2'])
  refset1.append(weave.rootAssembly.referencePoints[pt1.id])
  refset2.append(weave.rootAssembly.referencePoints[pt2.id]) 

#constrain motion of weft end points  
weave.rootAssembly.Set(name='RefSet3', referencePoints=tuple(refset1))
weave.rootAssembly.Set(name='RefSet4', referencePoints=tuple(refset2))
weave.DisplacementBC(amplitude=UNSET, createStepName='Step-3', 
    distributionType=UNIFORM, fieldName='', fixed=OFF, localCsys=None, name=
    'BC-7', region=mdb.models['Model-1'].rootAssembly.sets['RefSet3'], u1=0.0, 
    u2=0.0, u3=0.0, ur1=0.0, ur2=UNSET, ur3=UNSET)
weave.DisplacementBC(amplitude=UNSET, createStepName='Step-3', 
    distributionType=UNIFORM, fieldName='', fixed=OFF, localCsys=None, name=
    'BC-8', region=mdb.models['Model-1'].rootAssembly.sets['RefSet4'], u1=0.0, 
    u2=0.0, u3=0.0, ur1=0.0, ur2=UNSET, ur3=UNSET)

#step 4, add relax weave----------------------------------------------

mdb.models['Model-1'].StaticStep(initialInc=.1, maxInc=1, minInc=1e-12, 
   maxNumInc=1000, name='Step-4', previous='Step-3')

#remove boundary conditions
mdb.models['Model-1'].boundaryConditions['BC-5'].setValuesInStep(stepName=
    'Step-4', u1=FREED)
mdb.models['Model-1'].boundaryConditions['BC-6'].setValuesInStep(stepName=
    'Step-4', u1=FREED, u3=FREED)
mdb.models['Model-1'].boundaryConditions['BC-7'].setValuesInStep(stepName=
    'Step-4', u3=FREED)
mdb.models['Model-1'].boundaryConditions['BC-8'].setValuesInStep(stepName=
    'Step-4', u1=FREED, u3=FREED)



#step 5, add load-----------------------------------------------------

mdb.models['Model-1'].StaticStep(initialInc=.1, maxInc=1, minInc=1e-12, 
   maxNumInc=1000, name='Step-5', previous='Step-4')

#modify warp constraints & add uniaxial load  
if disz != 0:
 
    mdb.models['Model-1'].boundaryConditions['BC-6'].setValuesInStep(stepName=
    'Step-5', u1=FREED, u3=disz)

else:
  
    mdb.models['Model-1'].boundaryConditions['BC-6'].setValuesInStep(stepName=
    'Step-5', u1=FREED, u3=FREED)


#modify weft constraints & add uniaxial load  

if disx != 0:
 
    mdb.models['Model-1'].boundaryConditions['BC-8'].setValuesInStep(stepName=
    'Step-5', u3=FREED, u1=disx)

else:
  
    mdb.models['Model-1'].boundaryConditions['BC-8'].setValuesInStep(stepName=
    'Step-5', u1=FREED, u3=FREED)
