import adsk.core, adsk.fusion, traceback, math
def createNewComponent(rootComp):
    allOccs = rootComp.occurrences
    newOcc = allOccs.addNewComponent(adsk.core.Matrix3D.create())
    return newOcc.component

def translateToNewOrigin(design, newOrigin):
    root = adsk.fusion.Component.cast(design.rootComponent)
    # Iterate through the occurrences in the root component.
    for occurrence in root.occurrences:
        # Get the current transform of the occurrence.
        trans = adsk.core.Matrix3D.cast(occurrence.transform)
        # Change the translation portion of the matrix to account for the new origin.
        trans.translation = newOrigin.asVector()
        # Set the transform of the occurrence.
        occurrence.transform = trans
    # Snapshot the changes.
    root.parentDesign.snapshots.add()

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        rootComp = design.rootComponent
        features = rootComp.features
        
        #Translate root component to a new origin        
        newOrigin = adsk.core.Point3D.create(0,0,0)
        translateToNewOrigin(design, newOrigin)

        #Rotate selected body in x around origin
        body = adsk.fusion.BRepBody.cast(ui.selectEntity('Select a body', 'Bodies').entity)
        if body:
            trans = adsk.core.Matrix3D.create()
            rotX = adsk.core.Matrix3D.create()
            rotX.setToRotation(math.pi/4, adsk.core.Vector3D.create(1.57,0,0), adsk.core.Point3D.create(0,0,0))
            trans.transformBy(rotX)
            des = adsk.fusion.Design.cast(app.activeProduct)
            root = des.rootComponent
            ents = adsk.core.ObjectCollection.create()
            ents.add(body)
            moveInput = root.features.moveFeatures.createInput(ents, trans)
            root.features.moveFeatures.add(moveInput)
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
