import adsk.core
import adsk.fusion
import traceback
#import os.path
import xml.etree.ElementTree as ET
import math
import xml.dom.minidom as DOM
import os
global position
global ui
import tkinter
from tkinter.filedialog import askdirectory

ui = None
app = adsk.core.Application.get()
ui  = app.userInterface

position = 0
## @package SDFusion
# This is an exporter for Autodesk Fusion 360 models to SDFormat.
#
# This can be loaded as an addon in Fusion 360.
# It exports all rigid groups of the robot model as links
# to STL and creates nodes in the SDF file for them.
# It creates SDF nodes for all joints of the robot model.
# Supported joint types are: "fixed", "revolute", and "ball".

## Global variable to make the Fusion 360 design object accessible
# for every function.
design = None

## Global variable to make the output file directory accessible for
# every function.
ui.messageBox('Please insert a destination address')        
root = tkinter.Tk()
root.withdraw()
root.update()
fileDir = askdirectory()
root.destroy()
print(fileDir)

#fileDir = "C:/Users/Usuario/Desktop/Legs"
#if not os.path.exists(fileDir):
#    os.makedirs(fileDir)

## Global variable to make the robot model name accessible for
# every function.
modelName = "Legs"

## Global variable to make the root occurrence accessible for
# every function.
rootOcc = None

## Transforms a matrix from Fusion 360 to Gazebo.
#
# This transforms a matrix given in the Fusion 360 coordinate system
# to one in the Gazebo cooridnate system.
#
# @param self a matrix given wrt the Fusion 360 coordinate system
# @return the matrix wrt the Gazebo coordinate system
def gazeboMatrix(self):
    matrix = adsk.core.Matrix3D.create()
    matrix.setCell(1, 1, 0)
    matrix.setCell(1, 2, -1)
    matrix.setCell(2, 1, 1)
    matrix.setCell(2, 2, 0)
    self.transformBy(matrix)
    return self

## Converts three double values to string.
#
# This function converts three double values to a string separated by spaces.
#
# @param x the first double value
# @param y the second double value
# @param z the third double value
# @return the string of these values
def vectorToString(x, y, z):
    string = str(x) + " " + str(y) + " " + str(z)
    return string

## Builds SDF pose node from vector.
#
# This function builds the SDF pose node for every joint.
#
# @param vector the vector pointing to the origin of the joint.
# @return the SDF pose node
def sdfPoseVector(vector):
    pose = ET.Element("pose")  
    # convert from cm (Fusion 360) to m (SI)
    x = 0.01 * vector.x
    y = 0.01 * vector.y
    z = 0.01 * vector.z
    pos = vectorToString(x, y, z)
    rot = vectorToString(0, 0, 0)
    pose.text = pos + " " + rot
    return pose
    
## Builds SDF pose node from matrix.
#
# This function builds the SDF pose node for every link.
#
# @param matrix the transformation matrix of the link
# @return the SDF pose node
def sdfPoseMatrix(matrix):
    pose = ET.Element("pose")
    # convert from cm (Fusion 360) to m (SI)
    trans = matrix.translation
    x = 0.01 * trans.x
    y = 0.01 * trans.y
    z = 0.01 * trans.z
    pos = vectorToString(x, y, z)
    # calculate roll pitch yaw from transformation matrix
    r11 = matrix.getCell(0, 0)
    r21 = matrix.getCell(1, 0)
    r31 = matrix.getCell(2, 0)
    r32 = matrix.getCell(2, 1)
    r33 = matrix.getCell(2, 2)
    pitch = math.atan2(-r31, math.sqrt(math.pow(r11, 2) + math.pow(r21, 2)))
    cp = math.cos(pitch)
    yaw = math.atan2(r21 / cp, r11 / cp)
    roll = math.atan2(r32 / cp, r33 / cp)
    rot = vectorToString(roll, pitch, yaw)
    pose.text = pos + " " + rot
    return pose

## Builds SDF inertial node from physical properties.
#
# This function builds the SDF inertial node for every link.
#
# @param physics the physical properties of a link
# @return the SDF inertial node
def sdfInertial(physics):
    inertial = ET.Element("inertial")
    # build pose node of COM
    com = physics.centerOfMass
    pose = sdfPoseVector(com)
    inertial.append(pose)
    # build mass node
    mass = ET.Element("mass")
    mass.text = str(physics.mass)
    inertial.append(mass)
    # build inertia node
    inertia = sdfInertia(physics)
    inertial.append(inertia)
    return inertial

## Builds SDF node for one moment of inertia.
#
# This helper function builds the SDF node for one moment of inertia.
#
# @param tag the tag of the XML node
# @param value the text of the XML node
# @return the SDF moment of inertia node
def sdfMom(tag, value):
    node = ET.Element(tag)
    # convert from kg/cm^2 (Fusion 360) to kg/m^2 (SI)
    node.text = str(0.0001 * value)
    return node

## Builds SDF inertia node from physical properties.
#
# This function builds the SDF inertia node for every link.
#
# @param physics the physical properties of a link
# @return the SDF inertia node
def sdfInertia(physics):
    inertia = ET.Element("inertia")
    (returnValue, xx, yy, zz, xy, yz, xz) = physics.getXYZMomentsOfInertia()
    inertia.append(sdfMom("ixx", xx))
    inertia.append(sdfMom("ixy", xy))
    inertia.append(sdfMom("ixz", xz))
    inertia.append(sdfMom("iyy", yy))
    inertia.append(sdfMom("iyz", yz))
    inertia.append(sdfMom("izz", zz))
    return inertia
    
## Builds SDF link node.
#
# This function builds the SDF link node for every link.
#
# @param lin the link to be exported
# @return the SDF link node
def linkSDF(lin):
    global position
    linkName = lin.component.name
    link = ET.Element("link", name=linkName)
    # build pose node
    matrix = gazeboMatrix(lin.transform)
    pose = sdfPoseMatrix(matrix)    
    #if position == 0:
    #    pose = sdfPoseMatrix(matrix)
    #elif position != 0:
    #    pose = position    
    link.append(pose)
    # get physical properties of occurrence
    physics = lin.physicalProperties
    # build inertial node
    inertial = sdfInertial(physics)
    link.append(inertial)
    # build collision node
    collision = ET.Element("collision", name = linkName + "_collision")
    link.append(collision)
    # build geometry node
    geometry = ET.Element("geometry")
    collision.append(geometry)
    # build mesh node
    mesh = ET.Element("mesh")
    geometry.append(mesh)
    # build uri node
    uri = ET.Element("uri")
    global modelName
    uri.text = "model://" + modelName + "/meshes/" + linkName + ".stl"
    mesh.append(uri)
    # scale the mesh from mm to m
    scale = ET.Element("scale")
    scale.text = "0.001 0.001 0.001"
    mesh.append(scale)
    # build visual node (equal to collision node)
    visual = ET.Element("visual", name = linkName + "_visual")
    visual.append(geometry)
    link.append(visual)
    return link
    
## Builds SDF joint node.
#
# This function builds the SDF joint node for every joint type.
#
# @param joi the joint 
# @param name_parent the name of the parent link
# @param name_child the name of the child link
# @return the SDF joint node
def jointSDF(joi, name_parent, name_child):
    global position
    jointInfo = []
    jointType = ""
    jType = joi.jointMotion.jointType
    if jType == 0:
        jointType = "fixed"
    elif jType == 1:
        jointInfo = revoluteJoint(joi)
        jointType = "revolute"
    elif jType == 2:
        # not implemented
        jointType = ""
    elif jType == 3:
        # not implemented
        jointType = ""
    elif jType == 4:
        # not implemented
        jointType = ""
    elif jType == 5:
        # not implemented
        jointType = ""
    elif jType == 6:
        # SDFormat does not implement ball joint limits
        jointType = "ball"
    name = joi.name
    joint = ET.Element("joint", name=name, type=jointType)
    # build parent node
    parent = ET.Element("parent")
    parent.text = name_parent
    joint.append(parent)
    # build child node
    child = ET.Element("child")
    child.text = name_child
    joint.append(child)
    # build pose node
    pose = sdfPoseVector(joi.geometryOrOriginOne.origin)
    position =  pose
    joint.append(pose)
    joint.extend(jointInfo)
    return joint

## Builds SDF axis node for revolute joints.
#
# This function builds the SDF axis node for revolute joint.
#
# @param joi one revolute joint object
# @return a list of information nodes (here one axis node)
# for the revolute joint
def revoluteJoint(joi):
    info = []
    # build axis node
    axis = ET.Element("axis")
    xyz = ET.Element("xyz")    
    vector = joi.jointMotion.rotationAxisVector
    xyz.text = vectorToString(vector.x, vector.y, vector.z)
    axis.append(xyz)
    # build limit node
    mini = joi.jointMotion.rotationLimits.minimumValue
    maxi = joi.jointMotion.rotationLimits.maximumValue
    limit = ET.Element("limit")
    axis.append(limit)
    # Lower and upper limit have to be switched and inverted,
    # because Fusion 360 moves the parent link wrt to the
    # child link and Gazebo moves the child link wrt to the
    # parent link.
    lower = ET.Element("lower")
    lower.text = str(-maxi)
    limit.append(lower)
    upper = ET.Element("upper")
    upper.text = str(-mini)
    limit.append(upper)
    info.append(axis)
    return info
    
## Plain STL export.
##
# @param occ the occurrence to be exported
# @param linkName the name of the created STL file
def exportToSTL(occ, linkName):
    global design
    global fileDir
    meshFolder = fileDir + "/meshes/"
    if not os.path.exists(meshFolder):
        os.makedirs(meshFolder)    
    fileName = meshFolder + linkName
    desExp = design.exportManager
    stlExportOptions = desExp.createSTLExportOptions(occ, fileName)
    desExp.execute(stlExportOptions)

## Exports a rigid group to STL.
## Transforms a matrix from Fusion 360 to Gazebo.
#
# This exports a rigid group as one STL file.
# For this all components of the rigidGroup are copied to a new component.
#
# @param rig the rigid group to be exported
# @return a new occurrence which is used to export the
# relevant information to SDFormat
def rigidGroupToSTL(rig):
    global rootOcc
    linkName = rig.name
    # create new occurrence
    linkOcc = rootOcc.addNewComponent(adsk.core.Matrix3D.create())
    linkOcc.component.name = linkName
    # copy all bodies of the rigid group to the new occurrence
    allOcc = rig.occurrences
    for occ in allOcc:
        allBod = occ.bRepBodies
        for bod in allBod:
            bod.copyToComponent(linkOcc)
    # export new occurrence to STL
    exportToSTL(linkOcc, linkName)
    return linkOcc
    
## Exports an single occurrence to STL.
#
# This exports a single Fusion occurence as an STL file.
#
# @param occ the occurrence that needs to be exported.
# @return a new occurrence which is used to export the
# relevant information to SDFormat
def occurrenceToSTL(occ):
    global rootOcc
    linkName = clearName(occ.name)
    # create new occurrence
    linkOcc = rootOcc.addNewComponent(adsk.core.Matrix3D.create())
    linkOcc.component.name = linkName
    # copy all bodies of the occurrence to the new occurrence
    allBod = occ.bRepBodies
    for bod in allBod:
        bod.copyToComponent(linkOcc)
    # export new occurrence to STL
    exportToSTL(linkOcc, linkName)
    return linkOcc

## Clear filenames of unwanted characters
#
# This function replaces all ':' with underscores and deletes spaces in filenames.
# to one in the Gazebo cooridnate system.
#
# @param name a filename
# @return the filename without ':' and spaces
def clearName(name):
    name = name.replace(":", "_")
    name = name.replace(" ", "")
    return name


def export_next(parent_name, terminate):
#    design = adsk.fusion.Design.cast(product)
    # get root component in this design
 #   rootComp = design.rootComponent
    global model
    global allRigidGroups
    global allComponents
    global position

    compareJoint = 'empty'
    if compareJoint not in jointsList:
        for com in allComponents:
            if com is not None:
                allJoints = com.joints
                #export child joint and link 
                for joi in allJoints:
                    if joi is not None:
                        if joi.name not in jointsList: 
                            one = joi.occurrenceOne
                            two = joi.occurrenceTwo
                            #joint_parent = clearName(one.name)
                            #joint_child = clearName(two.name)           
                            #missing_link = True
                            
#                            print("one")
#                            print(one.name)
#                            print("two")
#                            print(two.name)
                            
                            link_parent = None
                            link_child = None

                            for rig in allRigidGroups:

                                value_parent = rig.occurrences.itemByName(one.name)
                                value_child = rig.occurrences.itemByName(two.name)
 
                                if value_parent is not None:
                                    link_parent = rig.name
                                    print('possible parent: ', link_parent)
                            
                                if value_child is not None:
                                    link_child = rig.name
                                    #missing_link = False
                                    print('possible parent:', link_child)
 
                            if link_parent is None or link_child is None:
                                ui.messageBox('Error: Please include all objects in rigid groups!')
                                terminate = 1
                            print('Parent component is: ', parent_name)
                            print("export joint: ", joi.name)
                            joint = jointSDF(joi, link_child, link_parent)
                            print("pos: ", position.text)
                            model.append(joint)
                            jointsList.append(joi.name)
                            compareJoint = joi.name
                            if link_child != parent_name and link_parent != parent_name:
                                print('there was a bifurcation')
                            
                            for rig in allRigidGroups:   
                                if rig.name == link_child and rig.name != parent_name :
                                    linkOcc = rigidGroupToSTL(rig)              
                                    link = linkSDF(linkOcc)
                                    model.append(link)                          
                                    #delete the temporary new occurrence
                                    linkOcc.deleteMe()
                                    #Call doEvents to give Fusion a chance to react.
                                    adsk.doEvents()
                                    print('export_next was called with link_child = parent')
                                    export_next(link_child, terminate)


                                elif rig.name == link_parent and rig.name != parent_name:
                                   # jointsList.append(link_child)
                                    #export_next(link_child, linksList)
                                
                                    ## MOVE RIGID GROUP
                                
                                    # ROTATE RIGID GROUP
                                
                                
                                    linkOcc = rigidGroupToSTL(rig)    
                                    link = linkSDF(linkOcc)
                                    model.append(link)                          
                                    #delete the temporary new occurrence
                                    linkOcc.deleteMe()
                                    #Call doEvents to give Fusion a chance to react.
                                    adsk.doEvents()
                                    print('export_next was called with link_parent = parent')                                    
                                    print("exported link ", rig.name)
                                    export_next(link_parent, terminate)
    if terminate == 1:
        return 0
                                    
## Exports a robot model from Fusion 360 to SDFormat.
def run(context):
    try:
        # get active design   
        global product
        product = app.activeProduct
        global design
        design = adsk.fusion.Design.cast(product)
        # get root component in this design
        rootComp = design.rootComponent
        # get all occurrences within the root component
        global rootOcc
        rootOcc = rootComp.occurrences      
# build sdf root node
        root = ET.Element("sdf", version="1.6")             
        global model        
        model = ET.Element("model", name=modelName)         
        root.append(model)                                  
### 1)///// get root component
        # Select an occurrence.

        occSel = ui.selectEntity('Select root link', 'Occurrences')
        if occSel:
            occ = adsk.fusion.Occurrence.cast(occSel.entity)
            
            result = ''
            for rigidGroup in occ.rigidGroups:
                result += rigidGroup.name
                    
            if result == '':
                ui.messageBox('No rigid groups for the selected occurrence.')
            else:
                ui.messageBox('The rigid groups below are on the selected occurrence:' + result)
        
        linkOcc = rigidGroupToSTL(rigidGroup)              
        link = linkSDF(linkOcc)
        model.append(link)                          
        #delete the temporary new occurrence
        linkOcc.deleteMe()
        #Call doEvents to give Fusion a chance to react.
        adsk.doEvents()
        
        global allRigidGroups
        allRigidGroups = rootComp.allRigidGroups
        global allComponents
        allComponents = design.allComponents     
#2) search for parent component
        parent_name = result 
        global jointsList 
        jointsList = []
        global terminate
        global position
        position = 0
        terminate = 0
        export_next(parent_name, terminate)
        
        # get all construction points that serve as viaPoints
        filename = fileDir + "/model.sdf"
        domxml = DOM.parseString(ET.tostring(root))
        pretty = domxml.toprettyxml()
        file = open(filename, "w")
        file.write(pretty)
        file.close()
        ui.messageBox("SDF file of model " + modelName + " written to '" + fileDir + "'.")
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc())) 