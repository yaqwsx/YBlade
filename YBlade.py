#Author-Jan Mrázek
#Description-

import adsk.core, adsk.fusion, adsk.cam, traceback
from adsk.core import Point3D, Point2D, Vector3D, Vector2D, Matrix3D
import math
import sys 
from copy import deepcopy

handlers = []
sys.stderr = sys.stdout
sys.stdout.flush()

def readProfile(profileFile):
    points = []
    for l in profileFile.readlines()[1:]:
        p = l.split()
        points.append((float(p[0]), float(p[1])))
    return points

class Struct(object): pass

def readBlade(bladeFile):
    sections = []
    for l in bladeFile.readlines()[3:]:
        x = l.split()
        s = Struct()
        s.pos = float(x[0]) * 100 # Convert to cm
        s.len = float(x[1]) * 100 # Convert to cm 
        s.twist = float(x[2])
        s.offset = float(x[3]) * 100 # Convert to cm
        s.thread = float(x[4])
        sections.append(s)
    return sections

def findClosest(target, l):
    """
    Find value in l closest target. Return index of such a value
    """
    minVal = l[0]
    minIdx = 0
    for i, e in enumerate(l):
        if abs(target - minVal) > abs(target - e):
            minVal = e
            minIdx = i
    return minIdx

def deduceOffset(blade, profile):
    positives = list([(x, y) for x, y in profile if y > 0])
    posIdx = findClosest(blade[0].thread, [x for x, y in positives])
    negatives = list([(x, y) for x, y in profile if y < 0])
    negIdx = findClosest(blade[0].thread, [x for x, y in negatives])

    mid = (positives[posIdx][1] + negatives[negIdx][1]) / 2

    for b in blade:
        b.offset = -mid * b.len

def offsetLen(blade, offsetMm):
    newBlade = []
    for x in blade:
        y = deepcopy(x)
        y.len -= 2 * offsetMm / 10
        newBlade.append(y)
    return newBlade

def profilePoints(profileData, chordLength, twist, threadAxisOffset, zoffset):
    pointSet = adsk.core.ObjectCollection.create()
    for profilePoint in profileData:
        p = Point3D.create(profilePoint[0] * chordLength, profilePoint[1] * chordLength, 0)
        p.translateBy(Vector3D.create(-chordLength * threadAxisOffset, zoffset))
        m = Matrix3D.create()
        m.setToRotation(math.radians(twist), Vector3D.create(0, 0, 1), Point3D.create(0, 0, 0))
        p.transformBy(m)
        pointSet.add(p)
    return pointSet

def drawProfile(sketch, profileData, chordLength, twist, threadAxisOffset, zoffset):
    pointSet = profilePoints(profileData, chordLength, twist, threadAxisOffset, zoffset)
    spline = sketch.sketchCurves.sketchFittedSplines.add(pointSet)
    first, last = pointSet.item(0), pointSet.item(pointSet.count - 1)
    line = sketch.sketchCurves.sketchLines.addByTwoPoints(first, last)
    profile = adsk.core.ObjectCollection.create()
    profile.add(spline)
    profile.add(line)
    return profile

def drawProfileLines(sketch, profileData, chordLength, twist, threadAxisOffset, zoffset):
    pointSet = profilePoints(profileData, chordLength, twist, threadAxisOffset, zoffset)
    lineProfile = adsk.core.ObjectCollection.create()
    for i in range(pointSet.count):
        first, last = pointSet.item(i), pointSet.item((i + 1) % pointSet.count)
        line = sketch.sketchCurves.sketchLines.addByTwoPoints(first, last)
        line.isConstruction = True
        lineProfile.add(line)
    return lineProfile

def drawGuideLine(sketch, blade, seed):
    pointSet = adsk.core.ObjectCollection.create() 
    for s in blade:
        p = Point3D.create(seed[0] * s.len, seed[1] * s.len, s.pos)
        p.translateBy(Vector3D.create(-s.len * s.thread, s.offset))
        m = Matrix3D.create()
        m.setToRotation(math.radians(s.twist), Vector3D.create(0, 0, 1), Point3D.create(0, 0, 0))
        p.transformBy(m)
        pointSet.add(p)
    spline = sketch.sketchCurves.sketchFittedSplines.add(pointSet)
    return spline

def drawSpline(sketch, points):
    spline = sketch.sketchCurves.sketchFittedSplines.add(points)
    return adsk.fusion.Path.create(spline, adsk.fusion.ChainedCurveOptions.noChainedCurves)

def drawLinestring(sketch, points):
    line = adsk.core.ObjectCollection.create() 
    for i in range(points.count - 1):
        s, e = points.item(i), points.item((i + 1) % points.count)
        l = sketch.sketchCurves.sketchLines.addByTwoPoints(s, e)
        line.add(l)
    return line

def extrudeBlade(component, profiles, sweepLine, guideLine):
    sweepProfile = adsk.core.ObjectCollection.create()
    sweepProfile.add(profiles[0].profiles.item(0))
    sweepProfile.add(profiles[0].profiles.item(1))

    path = component.features.createPath(sweepLine)
    guide = component.features.createPath(guideLine)

    sweeps = component.features.sweepFeatures
    sweepInput = sweeps.createInput(sweepProfile, path, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    sweepInput.guideRail = guide
    sweepInput.profileScaling = adsk.fusion.SweepProfileScalingOptions.SweepProfileScaleOption
    sweeps.add(sweepInput)

def hollowBlade(component, profiles, guides):
    loftFeats = component.features.loftFeatures
    loftInput = loftFeats.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    loftSectionsObj = loftInput.loftSections
    for p in profiles:
        loftSectionsObj.add(p.profiles.item(0))
    c = loftInput.centerLineOrRails
    for g in guides:
        c.addRail(g)
    loftFeats.add(loftInput)

def hollowBladeAlt(component, profiles, guides):
    loftFeats = component.features.loftFeatures
    for i in range(len(profiles) - 1):
        op = adsk.fusion.FeatureOperations.JoinFeatureOperation
        if i == 0:
            op = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        loftInput = loftFeats.createInput(op)
        loftSectionsObj = loftInput.loftSections
        loftSectionsObj.add(profiles[i].profiles.item(0))
        loftSectionsObj.add(profiles[i + 1].profiles.item(0))
        c = loftInput.centerLineOrRails
        for g in guides:
            path = adsk.fusion.Path.create(g.item(i), adsk.fusion.ChainedCurveOptions.noChainedCurves)
            c.addRail(path)
        loftFeats.add(loftInput)

def collectLinePoints(linestring):
    res = set()
    for i in range(linestring.count):
        l = linestring.item(i)
        s, e = l.geometry.startPoint, l.geometry.endPoint
        res.add((s.x, s.y))
        res.add((e.x, e.y))
    return list(res)

def getLeftmostPoint(points):
    res = points[0]
    for p in points:
        if p[0] < res[0]:
            res = p
    return res

def getRightmostPoint(points):
    res = points[0]
    for p in points:
        if p[0] > res[0]:
            res = p
    return res

def dist(a, b):
    x = a[0] - b[0]
    y = a[1] - b[1]
    return math.sqrt(x*x + y*y)

def _vec2d_dist(p1, p2):
    return (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2

def _vec2d_sub(p1, p2):
    return (p1[0]-p2[0], p1[1]-p2[1])


def _vec2d_mult(p1, p2):
    return p1[0]*p2[0] + p1[1]*p2[1]

# Taken from https://stackoverflow.com/questions/2573997/reduce-number-of-points-in-line
def ramerdouglas(line, dist):
    if len(line) < 3:
        return line

    (begin, end) = (line[0], line[-1]) if line[0] != line[-1] else (line[0], line[-2])

    distSq = []
    for curr in line[1:-1]:
        tmp = (
            _vec2d_dist(begin, curr) - _vec2d_mult(_vec2d_sub(end, begin), _vec2d_sub(curr, begin)) ** 2 / _vec2d_dist(begin, end))
        distSq.append(tmp)

    maxdist = max(distSq)
    if maxdist < dist ** 2:
        return [begin, end]

    pos = distSq.index(maxdist)
    return (ramerdouglas(line[:pos + 2], dist) + 
            ramerdouglas(line[pos + 1:], dist)[1:])

def reduceProfile(profile, minDistance):
    return ramerdouglas(profile, minDistance)

def inputFile(ui, title):
    fileDlg = ui.createFileDialog()
    fileDlg.isMultiSelectEnabled = False
    fileDlg.title = title
    fileDlg.filter = "*.*"
    
    # Show file open dialog
    dlgResult = fileDlg.showOpen()
    if dlgResult == adsk.core.DialogResults.DialogOK:
        return fileDlg.filenames[0]
    else:
        raise RuntimeError("No file specified")      

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        commandDefinitions = ui.commandDefinitions

        design = app.activeProduct
        rootComp = design.activeComponent 

        qbladeFile = inputFile(ui, "Select QBlade specification file")
        profileFile = inputFile(ui, "Select profile file")

        class YBladeExecuteHandler(adsk.core.CommandEventHandler):
            def __init__(self):
                super().__init__()
            def notify(self, args):
                try:
                    command = args.firingEvent.sender
                    inputs = command.commandInputs
                    params = {input.id: input.value for input in inputs}
                    
                    sketches = rootComp.sketches
                    planes = rootComp.constructionPlanes
                    xyPlane = rootComp.xYConstructionPlane

                    with open(params["profileFile"]) as f:
                        profileData = readProfile(f)
                    reducedProfileData = reduceProfile(profileData, params["simplificationFactor"])

                    with open(params["bladeFile"]) as f:
                        blade = readBlade(f)
                    deduceOffset(blade, profileData)

                    profiles = []
                    prevLen = -1000
                    prevTwist = -1000
                    leftInfillRail = adsk.core.ObjectCollection.create()
                    rightInfillRail = adsk.core.ObjectCollection.create()
                    for i, b in enumerate(blade):
                        if abs(b.len - prevLen) < 1 and abs(b.twist - prevTwist) < 1 and i != len(blade) - 1:
                            continue
                        prevLen = b.len
                        prevTwist = b.twist
                        planeInput = planes.createInput()
                        offsetValue = adsk.core.ValueInput.createByReal(b.pos)
                        planeInput.setByOffset(xyPlane, offsetValue)
                        plane = planes.add(planeInput)
                        plane.name = f"profile_{i}"
                        profileSketch = sketches.add(plane)
                        profileSketch.isLightBulbOn = False
                        spline = drawProfile(profileSketch, profileData, b.len, b.twist, b.thread, b.offset)
                        lines = drawProfileLines(profileSketch, reducedProfileData, b.len, b.twist, b.thread, b.offset)
                        dirPoint = adsk.core.Point3D.create(0, 0, 0)
                        offsetCurves = profileSketch.offset(lines, dirPoint, params["thickness"])
                        profiles.append(profileSketch)

                        points = collectLinePoints(offsetCurves)
                        lp = getLeftmostPoint(points)
                        leftInfillRail.add(Point3D.create(lp[0], lp[1], b.pos))
                        rp = getRightmostPoint(points)
                        rightInfillRail.add(Point3D.create(rp[0], rp[1], b.pos))

                    guideSketch = sketches.add(xyPlane)
                    guideLine1 = drawGuideLine(guideSketch, blade, (0, 0))
                    guideLine2 = drawGuideLine(guideSketch, blade, (1, 0))
                    innerGuide1 = drawLinestring(guideSketch, leftInfillRail)
                    innerGuide2 = drawLinestring(guideSketch, rightInfillRail)
                    sweepLine = guideSketch.sketchCurves.sketchLines.addByTwoPoints(
                        Point3D.create(0, 0, blade[0].pos),
                        Point3D.create(0, 0, blade[-1].pos))

                    hollowBladeAlt(rootComp, profiles, [innerGuide1, innerGuide2])
                    extrudeBlade(rootComp, profiles, sweepLine, guideLine1)
                    
                    adsk.terminate()
                except:
                    if ui:
                        ui.messageBox("Failed:\n{}".format(traceback.format_exc()))

        class YBladeDestroyHandler(adsk.core.CommandEventHandler):
            def __init__(self):
                super().__init__()
            def notify(self, args):
                sys.stdout.close()

        class YBladeCreateHandler(adsk.core.CommandCreatedEventHandler):
            def __init__(self):
                super().__init__()        
            def notify(self, args):
                try:
                    cmd = args.command
                    onExecute = YBladeExecuteHandler()
                    cmd.execute.add(onExecute)
                    onDestroy = YBladeDestroyHandler()
                    cmd.destroy.add(onDestroy)
                    # keep the handler referenced beyond this function
                    handlers.append(onExecute)
                    handlers.append(onDestroy)

                    inputs = cmd.commandInputs
                    inputs.addStringValueInput("profileFile", "Profile file path", profileFile)
                    inputs.addStringValueInput("bladeFile", "Blade file path", qbladeFile)
                    inputs.addDistanceValueCommandInput("thickness", "Shell thickness", adsk.core.ValueInput.createByString("1mm"))
                    inputs.addValueInput("simplificationFactor", "Infill simplification factor", "", adsk.core.ValueInput.createByReal(0.005))
                except:
                    if ui:
                        ui.messageBox("Failed:\n{}".format(traceback.format_exc()))

        cmdDef = commandDefinitions.itemById("YBlade")
        if not cmdDef:
            cmdDef = commandDefinitions.addButtonDefinition("YBlade",
                    "Import QBlade",
                    "Create a blade.",
                    "./resources")
    
        onCommandCreated = YBladeCreateHandler()
        cmdDef.commandCreated.add(onCommandCreated)
        # keep the handler referenced beyond this function
        handlers.append(onCommandCreated)
        inputs = adsk.core.NamedValues.create()
        cmdDef.execute(inputs)
        adsk.autoTerminate(False)
    except:
        print("Failed:\n{}".format(traceback.format_exc()))
        if ui:
            ui.messageBox("Failed:\n{}".format(traceback.format_exc()))
