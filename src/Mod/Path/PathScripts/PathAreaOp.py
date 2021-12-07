# -*- coding: utf-8 -*-
# ***************************************************************************
# *   Copyright (c) 2017 sliptonic <shopinthewoods@gmail.com>               *
# *                                                                         *
# *   This program is free software; you can redistribute it and/or modify  *
# *   it under the terms of the GNU Lesser General Public License (LGPL)    *
# *   as published by the Free Software Foundation; either version 2 of     *
# *   the License, or (at your option) any later version.                   *
# *   for detail see the LICENCE text file.                                 *
# *                                                                         *
# *   This program is distributed in the hope that it will be useful,       *
# *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
# *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
# *   GNU Library General Public License for more details.                  *
# *                                                                         *
# *   You should have received a copy of the GNU Library General Public     *
# *   License along with this program; if not, write to the Free Software   *
# *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  *
# *   USA                                                                   *
# *                                                                         *
# ***************************************************************************

import FreeCAD
import Path
import PathScripts.PathLog as PathLog
import PathScripts.PathOp as PathOp
import PathScripts.PathUtils as PathUtils
import PathScripts.PathGeom as PathGeom
import DraftGeomUtils
import math
from PySide import QtCore

# lazily loaded modules
from lazy_loader.lazy_loader import LazyLoader
Draft = LazyLoader('Draft', globals(), 'Draft')
Part = LazyLoader('Part', globals(), 'Part')

if FreeCAD.GuiUp:
    import FreeCADGui

__title__ = "Base class for PathArea based operations."
__author__ = "sliptonic (Brad Collette)"
__url__ = "https://www.freecadweb.org"
__doc__ = "Base class and properties for Path.Area based operations."
__contributors__ = "russ4262 (Russell Johnson)"


PathLog.setLevel(PathLog.Level.INFO, PathLog.thisModule())
# PathLog.trackModule(PathLog.thisModule())


# Qt translation handling
def translate(context, text, disambig=None):
    return QtCore.QCoreApplication.translate(context, text, disambig)


class ObjectOp(PathOp.ObjectOp):
    '''Base class for all Path.Area based operations.
    Provides standard features including debugging properties AreaParams,
    PathParams and removalshape, all hidden.
    The main reason for existence is to implement the standard interface
    to Path.Area so subclasses only have to provide the shapes for the
    operations.'''

    def opFeatures(self, obj):
        '''opFeatures(obj) ... returns the base features supported by all Path.Area based operations.
        The standard feature list is OR'ed with the return value of areaOpFeatures().
        Do not overwrite, implement areaOpFeatures(obj) instead.'''
        return PathOp.FeatureTool | PathOp.FeatureDepths | PathOp.FeatureStepDown \
            | PathOp.FeatureHeights | PathOp.FeatureStartPoint \
            | self.areaOpFeatures(obj) | PathOp.FeatureCoolant

    def areaOpFeatures(self, obj):
        '''areaOpFeatures(obj) ... overwrite to add operation specific features.
        Can safely be overwritten by subclasses.'''
        # pylint: disable=unused-argument
        return 0

    def initOperation(self, obj):
        '''initOperation(obj) ... sets up standard Path.Area properties and calls initAreaOp().
        Do not overwrite, overwrite initAreaOp(obj) instead.'''
        PathLog.track()

        # Debugging
        obj.addProperty("App::PropertyString", "AreaParams", "Path")
        obj.setEditorMode('AreaParams', 2)  # hide
        obj.addProperty("App::PropertyString", "PathParams", "Path")
        obj.setEditorMode('PathParams', 2)  # hide
        obj.addProperty("Part::PropertyPartShape", "removalshape", "Path")
        obj.setEditorMode('removalshape', 2)  # hide

        self.initAreaOp(obj)

    def initAreaOp(self, obj):
        '''initAreaOp(obj) ... overwrite if the receiver class needs initialisation.
        Can safely be overwritten by subclasses.'''
        pass # pylint: disable=unnecessary-pass

    def areaOpShapeForDepths(self, obj, job):
        '''areaOpShapeForDepths(obj) ... returns the shape used to make an initial calculation for the depths being used.
        The default implementation returns the job's Base.Shape'''
        if job:
            if job.Stock:
                PathLog.debug("job=%s base=%s shape=%s" % (job, job.Stock, job.Stock.Shape))
                return job.Stock.Shape
            else:
                PathLog.warning(translate("PathAreaOp", "job %s has no Base.") % job.Label)
        else:
            PathLog.warning(translate("PathAreaOp", "no job for op %s found.") % obj.Label)
        return None

    def areaOpOnChanged(self, obj, prop):
        '''areaOpOnChanged(obj, porp) ... overwrite to process operation specific changes to properties.
        Can safely be overwritten by subclasses.'''
        pass # pylint: disable=unnecessary-pass

    def opOnChanged(self, obj, prop):
        '''opOnChanged(obj, prop) ... base implementation of the notification framework - do not overwrite.
        The base implementation takes a stab at determining Heights and Depths if the operations's Base
        changes.
        Do not overwrite, overwrite areaOpOnChanged(obj, prop) instead.'''
        # PathLog.track(obj.Label, prop)
        if prop in ['AreaParams', 'PathParams', 'removalshape']:
            obj.setEditorMode(prop, 2)

        if prop == 'Base' and len(obj.Base) == 1:
            (base, sub) = obj.Base[0]
            bb = base.Shape.BoundBox  # parent boundbox
            subobj = base.Shape.getElement(sub[0])
            fbb = subobj.BoundBox  # feature boundbox

            if hasattr(obj, 'Side'):
                if bb.XLength == fbb.XLength and bb.YLength == fbb.YLength:
                    obj.Side = "Outside"
                else:
                    obj.Side = "Inside"

        self.areaOpOnChanged(obj, prop)

    def opOnDocumentRestored(self, obj):
        for prop in ['AreaParams', 'PathParams', 'removalshape']:
            if hasattr(obj, prop):
                obj.setEditorMode(prop, 2)

        self.areaOpOnDocumentRestored(obj)

    def areaOpOnDocumentRestored(self, obj):
        '''areaOpOnDocumentRestored(obj) ... overwrite to fully restore receiver'''
        pass # pylint: disable=unnecessary-pass

    def opSetDefaultValues(self, obj, job):
        '''opSetDefaultValues(obj) ... base implementation, do not overwrite.
        The base implementation sets the depths and heights based on the
        areaOpShapeForDepths() return value.
        Do not overwrite, overwrite areaOpSetDefaultValues(obj, job) instead.'''
        PathLog.debug("opSetDefaultValues(%s, %s)" % (obj.Label, job.Label))

        if PathOp.FeatureDepths & self.opFeatures(obj):
            try:
                shape = self.areaOpShapeForDepths(obj, job)
            except Exception as ee: # pylint: disable=broad-except
                PathLog.error(ee)
                shape = None

            # Set initial start and final depths
            if shape is None:
                PathLog.debug("shape is None")
                startDepth = 1.0
                finalDepth = 0.0
            else:
                bb = job.Stock.Shape.BoundBox
                startDepth = bb.ZMax
                finalDepth = bb.ZMin

            # obj.StartDepth.Value = startDepth
            # obj.FinalDepth.Value = finalDepth
            obj.OpStartDepth.Value = startDepth
            obj.OpFinalDepth.Value = finalDepth

            PathLog.debug("Default OpDepths are Start: {}, and Final: {}".format(obj.OpStartDepth.Value, obj.OpFinalDepth.Value))
            PathLog.debug("Default Depths are Start: {}, and Final: {}".format(startDepth, finalDepth))

        self.areaOpSetDefaultValues(obj, job)

    def areaOpSetDefaultValues(self, obj, job):
        '''areaOpSetDefaultValues(obj, job) ... overwrite to set initial values of operation specific properties.
        Can safely be overwritten by subclasses.'''
        pass # pylint: disable=unnecessary-pass

    def createLeadIn(self, win, vtouch, radius, degree):
        f=Part.Face(win)
        z=f.BoundBox.ZMax
        z=0
        v1=FreeCAD.Vector(-radius, -radius, z)
        midpoint1=radius*math.cos(45*math.pi/180)
        midpoint2=-radius+midpoint1
        v2=FreeCAD.Vector(-radius, -radius, z)
        A1 = Part.LineSegment(FreeCAD.Vector(-radius, -radius, z), FreeCAD.Vector(-radius*2, -radius,z))
        A2 = Part.Arc(v1, FreeCAD.Vector(midpoint2, -midpoint1, z), FreeCAD.Vector(0,0,z))
        B1 = Part.Arc(FreeCAD.Vector(0,0,z), FreeCAD.Vector(midpoint2, midpoint1, z), FreeCAD.Vector(-radius, radius, z))
        B2 = Part.LineSegment(FreeCAD.Vector(-radius, radius, z), FreeCAD.Vector(-radius*2, radius, z))
        B3 = Part.LineSegment(FreeCAD.Vector(-radius*2, radius, z), FreeCAD.Vector(-radius*2, -radius, z))
        w1=Part.Wire([Part.Edge(B1), Part.Edge(B2), Part.Edge(B3)])
        w2=Part.Wire([Part.Edge(A1), Part.Edge(A2)])
        c1=w1.copy()
        c2=w2.copy()
        mat=FreeCAD.Matrix()
        mat.rotateZ(degree/180*math.pi)
        w1.transformShape(mat)
        w2.transformShape(mat)
        mat2=FreeCAD.Matrix()
        mat2.move(vtouch)
        w1.transformShape(mat2)
        w2.transformShape(mat2)
        insidecnt=0
        for i in w1.Vertexes+w2.Vertexes:
            if (f.isInside(FreeCAD.Vector(i.Point[0], i.Point[1], z), 0.05, True)):
                insidecnt=insidecnt+1
        if insidecnt>2:
            mat=FreeCAD.Matrix()
            mat.rotateZ((degree-180)/180*math.pi)
            c1.transformShape(mat)
            c2.transformShape(mat)
            c1.transformShape(mat2)
            c2.transformShape(mat2)
            return [c1, c2]
        return [w1, w2]
        #set tabstop=4 set shiftwidth=4
    def getAngle(self, v1, v2, mode):
        try:
            deltaX = v2.x - v1.x
            deltaY = v2.y - v1.y
            if mode ==1:
                angle = math.atan2(float(deltaY),float(deltaX))*180/math.pi        # degrees
            else:
                angle = math.atan2(float(deltaY),float(deltaX))                  # radian
            return angle
        except Exception:
            print("error...")
            return 0


    def insertLeadIn(self, wirein, ep):
        c=wirein.Vertexes[0]
        maxlen=0
        mindist=None
        maxWire=None
        degree=45
        halfpos=Part.Vertex()
        newEdges=[]
        inEdges=Part.__sortEdges__(wirein.Edges)
        for i in inEdges:
            # todo curve/circle/arc integration for now I only took care about the GeomLine
            if hasattr(i, 'Curve') and i.Curve.TypeId == 'Part::GeomLine':
                dist = ep.distanceToLine(i.Curve.Location, i.Curve.Direction)
                line = None
                intersect = []
                if (len(self.halfposlist)>2):
                    e=self.halfposlist
                    line=Part.LineSegment(e[0], e[0]+(e[1]-e[0])*(len(self.halfposlist)+5))
                    intersect=DraftGeomUtils.findIntersection(i, Part.Edge(line))


                if (len(intersect)>0 or (mindist==None or dist<mindist)):
                    mindist=dist
                    maxlen=i.Length
                    half=[]
                    if (len(intersect)>0):
                        e1=Part.Edge(Part.LineSegment(i.Vertexes[0].Point, intersect[0]))
                        e2=Part.Edge(Part.LineSegment(intersect[0], i.Vertexes[1].Point))
                        if (e1.Orientation == 'Reversed'):
                            e1.reverse()
                        if (e2.Orientation == 'Reversed'):
                            e2.reverse()
                    else:
                        e1=Part.Edge(Part.LineSegment(i.Vertexes[0].Point, (i.Vertexes[0].Point+i.Vertexes[1].Point)/2))
                        e2=Part.Edge(Part.LineSegment((i.Vertexes[0].Point+i.Vertexes[1].Point)/2, i.Vertexes[1].Point))
                    half=Part.Wire([e1,e2])
                    halfpos=half.Vertexes[1].Point
                    degree=self.getAngle(i.Vertexes[0].Point, i.Vertexes[1].Point,1)+90
                    newEdges+=half.Edges
                else:
                    newEdges.append(i)
            else:
                newEdges.append(i)
        self.halfposlist.append(halfpos)
        leadIn=self.createLeadIn(wirein, halfpos, 4, degree+180)
        return self.getPath(leadIn[1].Edges, newEdges, leadIn[0].Edges)

    def getLength(self, v1, v2):
        p2=v2.Vertexes[0]
        p3=v2.Vertexes[1]
        if(v1.Vertexes[0].Point.isEqual(v2.Vertexes[0].Point, 0.001) or v1.Vertexes[0].Point.isEqual(v2.Vertexes[1].Point, 0.001)):
            p1=v1.Vertexes[1]
        else:
            p1=v1.Vertexes[0]
        a=Part.Edge(Part.LineSegment(p1.Point,  p2.Point))
        b=Part.Edge(Part.LineSegment(p2.Point, p3.Point))
        c=Part.Edge(Part.LineSegment(p3.Point, p1.Point))
        w=Part.Wire([a,b,c])
        return Part.Face(w).Length

    def getPath(self, entry, edges, exit):
         mainList=[]
         mainList.append(entry[0])
         mainList.append(entry[1])
         c1=None
         c2=None
         for b in edges:
             if self.isConnected(entry[1], b):
                 if c1 == None:
                     c1=b
                 elif c2 == None:
                     c2=b
                     break
         cseg=None
         if (self.getLength(c1, entry[1])>self.getLength(c2, entry[1])):
             mainList.append(c1)
             edges.remove(c1)
             cseg=c1
         else:
             mainList.append(c2)
             edges.remove(c2)
             cseg=c2
         slist=Part.__sortEdges__(edges)

         if (self.isConnected(entry[1], slist[0])):
             slist.reverse()
         mainList+=slist
         mainList+=exit
         return mainList

    def _buildPathArea(self, obj, baseobject, isHole, start, getsim):
        '''_buildPathArea(obj, baseobject, isHole, start, getsim) ... internal function.'''
        # pylint: disable=unused-argument
        startVector = None
        PathLog.track()
        area = Path.Area()
        area.setPlane(PathUtils.makeWorkplane(baseobject))
        area.add(baseobject)

        areaParams = self.areaOpAreaParams(obj, isHole) # pylint: disable=assignment-from-no-return

        entryPoint=None
        if 'entryPoint' in areaParams:
            entryPoint = AreaParams['entryPoint']
            areaParams.pop('entryPoint', None)

        passes = 1
        if 'passes' in areaParams:
            passes = areaParams['passes']
            areaParams.pop('passes', None)

        stepOverPercent = 0
        if 'stepOverPercent' in areaParams:
            stepOverPercent = areaParams['stepOverPercent']
            areaParams.pop('stepOverPercent', None)

        mpClippingArea = None
        mpClippingOffset = None
        if 'mpClippingArea' in areaParams:
            mpClippingArea = areaParams['mpClippingArea']
            mpClippingOffset = areaParams['mpClippingOffset']
            areaParams.pop('mpClippingOffset', None)
            areaParams.pop('mpClippingArea', None)

        radius = 0
        if 'radius' in areaParams:
            radius = areaParams['radius']
            areaParams.pop('radius', None)

        heights = [i for i in self.depthparams]
        PathLog.debug('depths: {}'.format(heights))

        PathLog.debug("Area with params: {}".format(area.getParams()))

        sections = []
        for i in range(passes):
            if Offset in areaParams:
                areaParams['Offset'] = areaParams['Offset'] + (radius*2)*stepOverPercent/100.0
            area.setParams(**areaParams)
            obj.AreaParams = str(area.getParams())
            sections += area.makeSections(mode=0, project=self.areaOpUseProjection(obj), heights=heights)


        if mpClippingArea is not None:
            shapelist=[]
            grouplist=[]
            self.halfposlist=[]
            sl={}
            for sec in sections:
                zPos=sec.getShape().BoundBox.ZMax
                mpClippingArea.translate(FreeCAD.Vector(0,0,sec.getShape().BoundBox.ZMax-mpClippingArea.BoundBox.ZMax))
                f=Path.Area().add([mpClippingArea, sec.getShape().Wires[0]], op=2).getShape()
                if len(f.Wires) == 0:
                    continue 
                leadin=self.insertLeadIn(f.Wires[0], FreeCAD.Vector(entryPoint[0], entryPoint[1], sec.getShape().BoundBox.ZMax))
                # not sure if needed
                if (leadin[0].Orientation == 'Reversed'):
                    leadin[0].reverse()
                if (leadin[1].Orientation == 'Reversed'):
                    leadin[1].reverse()
                intro=Part.Wire([leadin[1], leadin[0]])
                edges=Part.Wire(leadin[2:len(leadin)-4])
                outro=Part.Wire(leadin[len(leadin)-4:])
                if zPos not in sl:
                    sl[zPos]=[]
                sl[zPos].append([intro, edges, outro])
                #fixme
                if (leadin[0].Vertexes[0].Point.isEqual(leadin[1].Vertexes[0].Point, 0.001) or leadin[0].Vertexes[0].Point.isEqual(leadin[1].Vertexes[1].Point, 0.001)):
                    startVector=leadin[0].Vertexes[1].Point
                else:
                    startVector=leadin[0].Vertexes[0].Point
            k=sorted(sl.keys())
            k.reverse()
            for i in k:
                sl[i].reverse()
                for b in sl[i]:
                    shapelist+=b
        else:
            shapelist = [sec.getShape() for sec in sections]

  
        PathLog.debug("sections = %s" % sections)
        shapelist = [sec.getShape() for sec in sections]
        PathLog.debug("shapelist = %s" % shapelist)

        pathParams = self.areaOpPathParams(obj, isHole) # pylint: disable=assignment-from-no-return
        pathParams['shapes'] = shapelist
        pathParams['feedrate'] = self.horizFeed
        pathParams['feedrate_v'] = self.vertFeed
        pathParams['verbose'] = True
        if passes > 1:
            pathParams['sort_mode'] = 0
        pathParams['resume_height'] = obj.SafeHeight.Value
        pathParams['retraction'] = obj.ClearanceHeight.Value
        pathParams['return_end'] = True
        # Note that emitting preambles between moves breaks some dressups and prevents path optimization on some controllers
        pathParams['preamble'] = False

        if not self.areaOpRetractTool(obj):
            pathParams['threshold'] = 2.001 * self.radius

        if self.endVector is not None:
            pathParams['start'] = self.endVector
        elif PathOp.FeatureStartPoint & self.opFeatures(obj) and obj.UseStartPoint:
            pathParams['start'] = obj.StartPoint

        obj.PathParams = str({key: value for key, value in pathParams.items() if key != 'shapes'})
        PathLog.debug("Path with params: {}".format(obj.PathParams))

        (pp, end_vector) = Path.fromShapes(**pathParams)
        PathLog.debug('pp: {}, end vector: {}'.format(pp, end_vector))
        self.endVector = end_vector # pylint: disable=attribute-defined-outside-init

        simobj = None
        if getsim:
            areaParams['Thicken'] = True
            areaParams['ToolRadius'] = self.radius - self.radius * .005
            area.setParams(**areaParams)
            sec = area.makeSections(mode=0, project=False, heights=heights)[-1].getShape()
            simobj = sec.extrude(FreeCAD.Vector(0, 0, baseobject.BoundBox.ZMax))

        return pp, simobj

    def _buildProfileOpenEdges(self, obj, edgeList, isHole, start, getsim):
        '''_buildPathArea(obj, edgeList, isHole, start, getsim) ... internal function.'''
        # pylint: disable=unused-argument
        PathLog.track()

        paths = []
        heights = [i for i in self.depthparams]
        PathLog.debug('depths: {}'.format(heights))
        for i in range(0, len(heights)):
            for baseShape in edgeList:
                hWire = Part.Wire(Part.__sortEdges__(baseShape.Edges))
                hWire.translate(FreeCAD.Vector(0, 0, heights[i] - hWire.BoundBox.ZMin))

                pathParams = {} # pylint: disable=assignment-from-no-return
                pathParams['shapes'] = [hWire]
                pathParams['feedrate'] = self.horizFeed
                pathParams['feedrate_v'] = self.vertFeed
                pathParams['verbose'] = True
                pathParams['resume_height'] = obj.SafeHeight.Value
                pathParams['retraction'] = obj.ClearanceHeight.Value
                pathParams['return_end'] = True
                # Note that emitting preambles between moves breaks some dressups and prevents path optimization on some controllers
                pathParams['preamble'] = False

                if self.endVector is None:
                    V = hWire.Wires[0].Vertexes
                    lv = len(V) - 1
                    pathParams['start'] = FreeCAD.Vector(V[0].X, V[0].Y, V[0].Z)
                    if obj.Direction == 'CCW':
                        pathParams['start'] = FreeCAD.Vector(V[lv].X, V[lv].Y, V[lv].Z)
                else:
                    pathParams['start'] = self.endVector

                obj.PathParams = str({key: value for key, value in pathParams.items() if key != 'shapes'})
                PathLog.debug("Path with params: {}".format(obj.PathParams))

                (pp, end_vector) = Path.fromShapes(**pathParams)
                paths.extend(pp.Commands)
                PathLog.debug('pp: {}, end vector: {}'.format(pp, end_vector))

        self.endVector = end_vector
        simobj = None

        return paths, simobj

    def opExecute(self, obj, getsim=False): # pylint: disable=arguments-differ
        '''opExecute(obj, getsim=False) ... implementation of Path.Area ops.
        determines the parameters for _buildPathArea().
        Do not overwrite, implement
            areaOpAreaParams(obj, isHole) ... op specific area param dictionary
            areaOpPathParams(obj, isHole) ... op specific path param dictionary
            areaOpShapes(obj)             ... the shape for path area to process
            areaOpUseProjection(obj)      ... return true if operation can use projection
        instead.'''
        PathLog.track()

        # Instantiate class variables for operation reference
        self.endVector = None # pylint: disable=attribute-defined-outside-init
        self.leadIn = 2.0  # pylint: disable=attribute-defined-outside-init

        # Initiate depthparams and calculate operation heights for operation
        self.depthparams = self._customDepthParams(obj, obj.StartDepth.Value, obj.FinalDepth.Value)

        # Set start point
        if PathOp.FeatureStartPoint & self.opFeatures(obj) and obj.UseStartPoint:
            start = obj.StartPoint
        else:
            start = None

        aOS = self.areaOpShapes(obj) # pylint: disable=assignment-from-no-return

        # Adjust tuples length received from other PathWB tools/operations
        shapes = []
        for shp in aOS:
            if len(shp) == 2:
                (fc, iH) = shp
                #     fc, iH,  sub or description
                tup = fc, iH, 'otherOp'
                shapes.append(tup)
            else:
                shapes.append(shp)

        if len(shapes) > 1:
            jobs = list()
            for s in shapes:
                if s[2] == 'OpenEdge':
                    shp = Part.makeCompound(s[0])
                else:
                    shp = s[0]
                jobs.append({
                    'x': shp.BoundBox.XMax,
                    'y': shp.BoundBox.YMax,
                    'shape': s
                })

            jobs = PathUtils.sort_jobs(jobs, ['x', 'y'])

            shapes = [j['shape'] for j in jobs]

        sims = []
        for shape, isHole, sub in shapes:
            profileEdgesIsOpen = False

            if sub == 'OpenEdge':
                profileEdgesIsOpen = True
                if PathOp.FeatureStartPoint & self.opFeatures(obj) and obj.UseStartPoint:
                    osp = obj.StartPoint
                    self.commandlist.append(Path.Command('G0', {'X': osp.x, 'Y': osp.y, 'F': self.horizRapid}))

            try:
                if profileEdgesIsOpen:
                    (pp, sim) = self._buildProfileOpenEdges(obj, shape, isHole, start, getsim)
                else:
                    (pp, sim) = self._buildPathArea(obj, shape, isHole, start, getsim)
            except Exception as e: # pylint: disable=broad-except
                FreeCAD.Console.PrintError(e)
                FreeCAD.Console.PrintError("Something unexpected happened. Check project and tool config.")
            else:
                if profileEdgesIsOpen:
                    ppCmds = pp
                else:
                    ppCmds = pp.Commands

                # Save gcode commands to object command list
                self.commandlist.extend(ppCmds)
                sims.append(sim)
            # Eif

            if self.areaOpRetractTool(obj) and self.endVector is not None and len(self.commandlist) > 1:
                self.endVector[2] = obj.ClearanceHeight.Value
                self.commandlist.append(Path.Command('G0', {'Z': obj.ClearanceHeight.Value, 'F': self.vertRapid}))

        PathLog.debug("obj.Name: " + str(obj.Name) + "\n\n")
        return sims

    def areaOpRetractTool(self, obj):
        '''areaOpRetractTool(obj) ... return False to keep the tool at current level between shapes. Default is True.'''
        # pylint: disable=unused-argument
        return True

    def areaOpAreaParams(self, obj, isHole):
        '''areaOpAreaParams(obj, isHole) ... return operation specific area parameters in a dictionary.
        Note that the resulting parameters are stored in the property AreaParams.
        Must be overwritten by subclasses.'''
        # pylint: disable=unused-argument
        pass # pylint: disable=unnecessary-pass

    def areaOpPathParams(self, obj, isHole):
        '''areaOpPathParams(obj, isHole) ... return operation specific path parameters in a dictionary.
        Note that the resulting parameters are stored in the property PathParams.
        Must be overwritten by subclasses.'''
        # pylint: disable=unused-argument
        pass # pylint: disable=unnecessary-pass

    def areaOpShapes(self, obj):
        '''areaOpShapes(obj) ... return all shapes to be processed by Path.Area for this op.
        Must be overwritten by subclasses.'''
        # pylint: disable=unused-argument
        pass # pylint: disable=unnecessary-pass

    def areaOpUseProjection(self, obj):
        '''areaOpUseProcjection(obj) ... return True if the operation can use procjection, defaults to False.
        Can safely be overwritten by subclasses.'''
        # pylint: disable=unused-argument
        return False

    # Support methods
    def _customDepthParams(self, obj, strDep, finDep):
        finish_step = obj.FinishDepth.Value if hasattr(obj, "FinishDepth") else 0.0
        cdp = PathUtils.depth_params(
            clearance_height=obj.ClearanceHeight.Value,
            safe_height=obj.SafeHeight.Value,
            start_depth=strDep,
            step_down=obj.StepDown.Value,
            z_finish_step=finish_step,
            final_depth=finDep,
            user_depths=None)
        return cdp
# Eclass

def SetupProperties():
    setup = []
    return setup
