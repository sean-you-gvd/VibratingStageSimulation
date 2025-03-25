# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2019 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================


import pychrono.core as chrono
#import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess

import errno
import os
import importlib.util

import os

print ("Vibrating Stage Simulation")

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('relative/path/to/data/directory/')

bowlpath=r''
nx=7
nz=7
ny=3

def AddFallingItems(sys):
    # Shared contact materials for falling objects
    bodylist=[]
    mat = chrono.ChContactMaterialSMC()
    mat.SetFriction(1e-3)
    mat.SetAdhesion(1e-3)
    # Create falling rigid bodies (spheres and boxes etc.)
    for ix in range(-7, 7):
        for iz in range(-7, 7):
            for iy in range(0,3):
                # add spheres
                mass = .1
                #radius = .05
                radius = .5
                body = chrono.ChBody()
                comp = (2.0 / 5.0) * mass * radius**2
                body.SetInertiaXX(chrono.ChVector3d(comp, comp, comp))
                body.SetMass(mass)
                #body.SetPos(chrono.ChVector3d(.1 * ix + 0.1, -2,.1 * iz))
                #body.SetPos(chrono.ChVector3d(3 * ix + 0.1, -2,3 * iz))
                body.SetPos(chrono.ChVector3d(4 * ix + 0.1, 2*iy-2,4 * iz))
                body_ct_shape = chrono.ChCollisionShapeSphere(mat, radius)
                body.AddCollisionShape(body_ct_shape)
                body.EnableCollision(True)

                sphere = chrono.ChVisualShapeSphere(radius)
                sphere.SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
                body.AddVisualShape(sphere)

                sys.AddBody(body)
                bodylist.append(body)

def AddContainerWall(body, mat, size, pos, visible=True):
    body_ct_shape = chrono.ChCollisionShapeBox(mat, size.x, size.y, size.z)
    body.AddCollisionShape(body_ct_shape, chrono.ChFramed(pos, chrono.QUNIT))
    if visible:
        box = chrono.ChVisualShapeBox(size)
        box.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
        body.AddVisualShape(box, chrono.ChFramed(pos))

def AddContainer(sys):
    # The fixed body (5 walls)
    fixedBody = chrono.ChBody()

    fixedBody.SetMass(1.0)
    fixedBody.SetFixed(True)
    fixedBody.SetPos(chrono.ChVector3d())
    fixedBody.EnableCollision(False)

    # Contact material for container
    fixed_mat = chrono.ChContactMaterialSMC()

    #AddContainerWall(fixedBody, fixed_mat, chrono.ChVector3d(20, 1, 20), chrono.ChVector3d(0, -5, 0))
    AddContainerWall(fixedBody, fixed_mat, chrono.ChVector3d(1, 10, 20.99), chrono.ChVector3d(-10, 0, 0), False)
    AddContainerWall(fixedBody, fixed_mat, chrono.ChVector3d(1, 10, 20.99), chrono.ChVector3d(10, 0, 0), False)
    AddContainerWall(fixedBody, fixed_mat, chrono.ChVector3d(20.99, 10, 1), chrono.ChVector3d(0, 0, -10), False)
    AddContainerWall(fixedBody, fixed_mat, chrono.ChVector3d(20.99, 10, 1), chrono.ChVector3d(0, 0, 10), False)

    sys.AddBody(fixedBody)

    # The rotating mixer body
    rotatingBody = chrono.ChBody()
    exported_items = chrono.ChBody()
    exported_items.SetMass(10.0)
    exported_items.SetInertiaXX(chrono.ChVector3d(50, 50, 50))

#    exported_items.SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))

    rotatingBody.SetMass(10.0)
    rotatingBody.SetInertiaXX(chrono.ChVector3d(50, 50, 50))
    rotatingBody.SetPos(chrono.ChVector3d(0, 0, 0))
    rotatingBody.EnableCollision(True)

    # Contact material for mixer body
    mixer_mat = chrono.ChContactMaterialSMC()
#    exported_items = chrono.ImportSolidWorksSystem(chrono.GetChronoDataFile('spline bowl.STEP'))

#    exported_items = chrono.ChBodyEasyMesh('hemisphere bowl.obj',  # x,y,z size
    exported_items = chrono.ChBodyEasyMesh('/bowls/spike bowl.obj',  # x,y,z size

                                        4000,      # density
                                        True,      # visualization?
                                        True,      # collision?
                                        True,
                                        mixer_mat, # contact material
                                        0.001) #swept




    rotatingBody = chrono.ChBodyEasyBox(20, 1, 20,  # x,y,z size
                                        4000,      # density
                                        True,      # visualization?
                                        True,      # collision?
                                        mixer_mat) # contact material
    rotatingBody.SetPos(chrono.ChVector3d(0, -4, 0))

    exported_items.SetPos(chrono.ChVector3d(0, 0, 0))
#    bowlimg=exported_items.GetVisualModel()
#    bowlimg.SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))

    #sys.Add(rotatingBody)
    sys.Add(exported_items)


    # A motor between the two
    #my_motor = chrono.ChLinkMotorRotationSpeed()
    my_motor=chrono.ChLinkMotorLinearSpeed()
    my_motor.Initialize(
                        exported_items,
                        #rotatingBody,
                        fixedBody,
                        chrono.ChFramed(chrono.ChVector3d(0, 0, 0), 
                                        chrono.QuatFromAngleAxis(chrono.CH_PI_2, chrono.VECT_X)))
    mfun=chrono.ChFunctionSine(10,10,2)

    #mfun = chrono.ChFunctionConst(chrono.CH_PI / 2.0)  # speed w=90°/s
    my_motor.SetSpeedFunction(mfun)

    sys.AddLink(my_motor)

    return rotatingBody

# ---------------------------------------------------------------------
#
#  Create the simulation sys and add items
#

sys = chrono.ChSystemSMC()
#sys = chrono.ChSystemNSC()


sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
#sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_MULTICORE)



# Simulation and rendering time-step
time_step = 1e-4
out_step  = 1.0 / 20

# Add fixed and moving bodies
mixer = AddContainer(sys)
AddFallingItems(sys)

# Create the Irrlicht visualization
#vis = chronoirr.ChVisualSystemIrrlicht()
#vis.AttachSystem(sys)
#vis.SetWindowSize(1024,768)
#vis.SetWindowTitle('Collisions between objects')
#vis.Initialize()
#vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
#vis.AddSkyBox()
#vis.AddCamera(chrono.ChVector3d(0, 18, -20))
#vis.AddCamera(chrono.ChVector3d(0, 80, -40))
#vis.AddTypicalLights()
#vis.AddLight(chrono.ChVector3d(400,2000,0),2000,chrono.ChColor(1,1,1))
#vis.AddLight(chrono.ChVector3d(0,-2000,0),2000,chrono.ChColor(1,1,1))
#pov_exporter = postprocess.ChPovRay(sys)
#pov_exporter.SetTemplateFile(chrono.GetChronoDataFile("POVRay_chrono_template.pov"))
#pov_exporter.SetBasePath("povray1")

pov_exporter = postprocess.ChBlender(sys)
#pov_exporter.SetTemplateFile(chrono.GetChronoDataFile("POVRay_chrono_template.pov"))
pov_exporter.SetBasePath("blender")

pov_exporter.AddAll()
pov_exporter.ExportScript()



# Simulation loop
time     = 0.0
out_time = 0.0
endtime = 300.0
while time<endtime:
    sys.DoStepDynamics(time_step)
    time = sys.GetChTime()
    if (time >= out_time and time < endtime):
#        vis.BeginScene() 
 #       vis.Render()
#        vis.EndScene()
        pov_exporter.ExportData()
        print ('time=', sys.GetChTime() )
        out_time += out_step
   
        # print out contact force and torque
        # frc = mixer.GetAppliedForce()
        # trq = mixer.GetAppliedTorque()
        # print(sys.GetChTime())
        # print("force: ", frc)
        # print("torque: ", trq)



