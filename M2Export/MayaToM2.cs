using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Autodesk.Maya.OpenMaya;
using Autodesk.Maya.OpenMayaAnim;
using M2Lib.m2;
using M2Lib.types;

namespace M2Export
{
    /// <summary>
    /// Extract procedures from Maya to WoW.
    /// </summary>
    public static class MayaToM2
    {
        private const float Epsilon = 0.00001f;
        private const int MaxWeightsNumber = 4;

        //Invert and create vector
        private static C3Vector AxisInvert(float x, float y, float z)
        {
            return MGlobal.isYAxisUp ? new C3Vector(-1*x, z, y) : new C3Vector(x, y, z);
        }

        private static C3Vector AxisInvert(MFloatPoint point) => AxisInvert(point.x, point.y, point.z);
        private static C3Vector AxisInvert(MFloatVector point) => AxisInvert(point.x, point.y, point.z);
        private static C3Vector AxisInvert(MPoint point) => AxisInvert((float) point.x, (float) point.y, (float) point.z);

        public static void ExtractModel(M2 wowModel)
        {
            var initialTime = MAnimControl.currentTime;
            MAnimControl.currentTime = 0;

            MGlobal.displayInfo("Building model " + wowModel.Name);

            var wowView = new M2SkinProfile();//TODO multiple views by decimating ?
            wowModel.Views.Add(wowView);//Because later in the code it always refers to wowModel.Views[0] since it's the only one we do.

            var jointIds = ExtractJoints(wowModel);
            ExtractMeshes(wowModel, jointIds);

            //Static models requirements
            if(wowModel.Sequences.Count == 0) wowModel.Sequences.Add(new M2Sequence());//For non-animated models, basic "Stand"
            if(wowModel.Bones.Count == 0) wowModel.Bones.Add(new M2Bone());//For jointless static models
            if(wowModel.TexUnitLookup.Count == 0) wowModel.TexUnitLookup.Add(0);
            if(wowModel.UvAnimLookup.Count == 0) wowModel.UvAnimLookup.Add(-1);

            MAnimControl.currentTime = initialTime;
        }
        /// <summary>
        /// Creates the bone hierarchy.
        /// </summary>
        /// <param name="wowModel"></param>
        private static Dictionary<string, short> ExtractJoints(M2 wowModel)
        {
            var mayaData = new Dictionary<string, MayaM2Bone>();

            // Sequences
            //TODO multiple sequences
            var start = MAnimControl.animationStartTime;
            var end = MAnimControl.animationEndTime;
            var startVal = start.asUnits(MTime.Unit.kMilliseconds);
            var endVal = end.asUnits(MTime.Unit.kMilliseconds);
            MGlobal.displayInfo("Anim from "+startVal+" to "+endVal);
            var sequence = new M2Sequence {Length = (uint) (endVal - startVal)};
            sequence.Flags |= M2Sequence.SequenceFlags.Looped;
            wowModel.Sequences.Add(sequence);

            //Goal of iteration : Extract raw joint data and store it in intermediate object, MayaM2Bone
            var processedJoints = new HashSet<string>();
            for(var jointIter = new MItDag(MItDag.TraversalType.kDepthFirst, MFn.Type.kJoint); !jointIter.isDone; jointIter.next())
            {
                var jointPath = new MDagPath();
                jointIter.getPath(jointPath);
                if (processedJoints.Contains(jointPath.fullPathName)) continue;
                MGlobal.displayInfo("Extracting raw data of "+jointPath.fullPathName);
                var joint = new MFnIkJoint(jointPath);
                mayaData[jointPath.fullPathName] = new MayaM2Bone();

                // Hierarchy
                var isRoot = joint.parentCount == 0 || !joint.parent(0).hasFn(MFn.Type.kJoint);
                if (!isRoot) {
                    var parentPath = new MDagPath();
                    MDagPath.getAPathTo(joint.parent(0), parentPath);
                    if(!mayaData.ContainsKey(parentPath.fullPathName))
                        MGlobal.displayError("\tParent is not referenced. Crash incoming. Path : "+parentPath.fullPathName);
                    mayaData[jointPath.fullPathName].Parent = mayaData[parentPath.fullPathName];
                }
                //Note : M2Bone.submesh_id is wrong in the wiki. Do not try to compute it.
                // Label
                mayaData[jointPath.fullPathName].Type = MGlobal.executeCommandStringResult("getAttr -asString " + joint.fullPathName + ".type");
                mayaData[jointPath.fullPathName].OtherType = MGlobal.executeCommandStringResult("getAttr -asString " + joint.fullPathName + ".otherType");
                mayaData[jointPath.fullPathName].Side = MGlobal.executeCommandStringResult("getAttr -asString " + joint.fullPathName + ".side");

                // Base translation is used to compute position
                MAnimControl.currentTime = 0;
                mayaData[jointPath.fullPathName].BaseTranslation = joint.getTranslation(MSpace.Space.kTransform);

                var transData = new List<Tuple<uint, MVector>>();
                var rotData = new List<Tuple<uint, MQuaternion>>();
                var scaleData = new List<Tuple<uint, MVector>>();
                for (var i = startVal; i < endVal; i+=33)//TODO FIXME What if not multiple of 33 ?
                {
                    //Get data for this joint for this frame
                    MAnimControl.currentTime = new MTime(i, MTime.Unit.kMilliseconds);

                    var translation = joint.getTranslation(MSpace.Space.kTransform);
                    var rotation = new MQuaternion();
                    joint.getRotation(rotation, MSpace.Space.kTransform);
                    var scaleArray = new double[3];
                    joint.getScale(scaleArray);
                    var scale = new MVector(scaleArray);

                    if (!translation.isEquivalent(MVector.zero, Epsilon))
                    {
                        var previousIsTheSame = transData.Count > 0 && transData.Last().Item2.isEquivalent(translation, Epsilon);
                        if (!previousIsTheSame)
                            transData.Add(new Tuple<uint, MVector>((uint) (i - startVal), translation));
                    }
                    if (!rotation.isEquivalent(MQuaternion.identity, Epsilon))
                    {
                        var previousIsTheSame = rotData.Count > 0 && rotData.Last().Item2.isEquivalent(rotation, Epsilon);
                        if (!previousIsTheSame)
                            rotData.Add(new Tuple<uint, MQuaternion>((uint) (i - startVal), rotation));
                    }
                    if (!scale.isEquivalent(MVector.one, Epsilon))
                    {
                        var previousIsTheSame = scaleData.Count > 0 && scaleData.Last().Item2.isEquivalent(scale, Epsilon);
                        if (!previousIsTheSame)
                            scaleData.Add(new Tuple<uint, MVector>((uint) (i - startVal), scale));
                    }
                }
                if(transData.Count > 0) mayaData[joint.fullPathName].Translation.Add(transData);
                if(rotData.Count > 0) mayaData[joint.fullPathName].Rotation.Add(rotData);
                if(scaleData.Count > 0) mayaData[joint.fullPathName].Scale.Add(scaleData);
                processedJoints.Add(jointPath.fullPathName);
            }

            //Goal of iteration : apply transformations to joint data & their children
            processedJoints.Clear();
            for (var jointIter = new MItDag(MItDag.TraversalType.kBreadthFirst, MFn.Type.kJoint);
                !jointIter.isDone;
                jointIter.next())
            {
                var jointPath = new MDagPath();
                jointIter.getPath(jointPath);
                if (processedJoints.Contains(jointPath.fullPathName)) continue;
                MGlobal.displayInfo("Applying joint orient of "+jointPath.fullPathName);
                var joint = new MFnIkJoint(jointPath);
                var jointOrient = new MQuaternion();
                joint.getOrientation(jointOrient);

                for (uint i = 0; i < joint.childCount; i++)
                {
                    if (!joint.child(i).hasFn(MFn.Type.kJoint)) continue;
                    var childFn = new MFnIkJoint(joint.child(i));
                    MGlobal.displayInfo("\tto "+ childFn.fullPathName+";");
                    mayaData[childFn.fullPathName].RotateTranslation(jointOrient);
                }

                processedJoints.Add(jointPath.fullPathName);
            }

            //Goal of iteration : Add to WoW model the joints data, converted to appropriate M2 objects with MayaM2Bone.Bone()
            processedJoints.Clear();
            for(var jointIter = new MItDag(MItDag.TraversalType.kDepthFirst, MFn.Type.kJoint); !jointIter.isDone; jointIter.next())
            {
                var jointPath = new MDagPath();
                jointIter.getPath(jointPath);

                if (processedJoints.Contains(jointPath.fullPathName)) continue;
                mayaData[jointPath.fullPathName].Index = wowModel.Bones.Count;
                wowModel.Bones.Add(mayaData[jointPath.fullPathName].ToBone());
            }

            var jointNameIds = new Dictionary<string, short>();//Maps a joint name to the WoW bone position in list.
            foreach (var entry in mayaData)
                jointNameIds[entry.Key] = (short) entry.Value.Index;
            return jointNameIds;
        }

        /// <summary>
        /// Faces and vertices UVs, positions and normals.
        /// </summary>
        /// <param name="meshPath"></param>
        /// <param name="wowMesh"></param>
        /// <param name="wowModel"></param>
        /// <param name="jointIds"></param>
        private static void ExtractGeometryMesh(MDagPath meshPath, M2SkinSection wowMesh, M2 wowModel, IReadOnlyDictionary<string, short> jointIds)
        {
            var wowView = wowModel.Views[0];
            var globalVertices = wowModel.GlobalVertexList;
            var vertexSum = new MFloatPoint();//for CenterMass later
            var vertexCounter = 0;

            //BEGIN Extract mesh data tables.

            // UV Sets
            var uvsets = new MStringArray();
            var meshFunctions = new MFnMesh(meshPath);
            meshFunctions.getUVSetNames(uvsets);

            //Bone Weights
            List<MDoubleArray> vertexWeights;
            MDagPathArray influenceObjects;
            GetMeshWeightData(out vertexWeights, out influenceObjects, meshPath);

            //Positions
            var positions = new MFloatPointArray();
            meshFunctions.getPoints(positions, MSpace.Space.kWorld);

            //Normals
            var normals = new MFloatVectorArray();
            meshFunctions.getVertexNormals(false, normals, MSpace.Space.kWorld);

            //END of extracting data tables

            var wowMeshVertices = new List<M2Vertex>();

            wowMesh.StartVertex = (ushort) wowView.Indices.Count;//TODO Check level for big models, like character models with lots of triangles
            wowMesh.StartTriangle = (ushort) wowView.Triangles.Count;//TODO Check level for big models

            var polygonIter = new MItMeshPolygon(meshPath);
            while (!polygonIter.isDone)
            {
                Debug.Assert(polygonIter.polygonVertexCount() == 3, 
                    "Format only handles Triangle faces. A face with "+polygonIter.polygonVertexCount()+" points has been found.\n");
                //for each vertex in the face.
                for (var i = 0; i < polygonIter.polygonVertexCount(); i++)//i = faceRelativeIndex
                {
                    var meshRelativeIndex = (int) polygonIter.vertexIndex(i);
                    var jointsWeightsForThisVertex = vertexWeights[meshRelativeIndex];
                    var wowVertex = new M2Vertex();
                    var indicesAndWeights = new List<Tuple<int, double>>();

                    //Bone weights
                    for (var b = 0; b < influenceObjects.length; b++)//for each joint
                    {
                        var kJointPath = influenceObjects[b];
                        if (!kJointPath.hasFn(MFn.Type.kJoint) || meshRelativeIndex >= vertexWeights.Count) continue;
                        Debug.Assert(b < jointsWeightsForThisVertex.Count, "vertexWeights size : " + vertexWeights.Count + " " +
                                                             "\njointWeights for this vertex : " + jointsWeightsForThisVertex.Count);
                        //Here are a joint&weight for this meshRelativeIndex vertex
                        var boneIndex = jointIds[kJointPath.fullPathName];
                        if (jointsWeightsForThisVertex[b] > Epsilon)
                        {
                            indicesAndWeights.Add(new Tuple<int, double>(boneIndex, jointsWeightsForThisVertex[b]));
                        }
                    }
                    if (indicesAndWeights.Count == 0)
                    {
                        MGlobal.displayWarning("No bone influence for vertex " + meshRelativeIndex);
                        // There is always at least 1 rootbone, even for static models.
                        wowVertex.BoneIndices[0] = 0;
                        wowVertex.BoneWeights[0] = byte.MaxValue;
                    }
                    else
                    {
                        if(indicesAndWeights.Count > MaxWeightsNumber)
                            MGlobal.displayWarning("This vertex is connected to more than "+MaxWeightsNumber+ " joints. Only the " + MaxWeightsNumber + " biggest will be exported.");
                        indicesAndWeights = indicesAndWeights.OrderByDescending(p => p.Item2).Take(MaxWeightsNumber).ToList();
                        Debug.Assert(indicesAndWeights.Count <= MaxWeightsNumber);
                        var weightSum = indicesAndWeights.Sum(p => p.Item2);
                        var availableWeight = byte.MaxValue;
                        for(var j = 0; j < indicesAndWeights.Count; j++)
                        {
                            wowVertex.BoneIndices[j] = (byte) indicesAndWeights[j].Item1;
                            //Stored weight is the minimum between actual weight and remaining weight, to keep the sum == 255
                            var byteWeight = (byte) (indicesAndWeights[j].Item2 / weightSum * byte.MaxValue);
                            wowVertex.BoneWeights[j] = byteWeight > availableWeight ? availableWeight : byteWeight;

                            availableWeight -= wowVertex.BoneWeights[j];
                        }
                        if (availableWeight > 0)// Remains
                        {
                            wowVertex.BoneWeights[indicesAndWeights.Count - 1] += availableWeight;
                        }

                        var totalWeight = wowVertex.BoneWeights.Sum(w => w);
                        Debug.Assert(totalWeight == byte.MaxValue, "Total sum of weights is not 255 but "+totalWeight);

                        wowMesh.BoneInfluences = (ushort) Math.Max(wowMesh.BoneInfluences, indicesAndWeights.Count);
                    }

                    //Pos&Normal. Notice the axis changing between Maya and WoW
                    var position = positions[meshRelativeIndex];
                    vertexSum.x = vertexSum.x + position.x;//To compute CenterMass
                    vertexSum.y = vertexSum.y + position.y;
                    vertexSum.z = vertexSum.z + position.z;
                    vertexCounter++;
                    wowVertex.Position = AxisInvert(position);
                    var normal = normals[meshRelativeIndex];
                    wowVertex.Normal = AxisInvert(normal);

                    //UV coordinates
                    if (uvsets.length <= 0 || meshFunctions.numUVs(uvsets[0]) <= 0) continue;
                    for (var j = 0; j < uvsets.length && j < 2; j++) //i < 2 as WoW M2Vertex support limits
                    {
                        var uvCoords = new float[2];
                        polygonIter.getUV(i, uvCoords, uvsets[j]);
                        // Notice the change with the v coordinate
                        wowVertex.TexCoords[j] = new C2Vector(uvCoords[0], 1 - uvCoords[1]);
                    }

                    var index = wowMeshVertices.IndexOf(wowVertex);//wowmesh relative index
                    if (index == -1)
                    {
                        wowView.Indices.Add((ushort) globalVertices.Count);
                        globalVertices.Add(wowVertex);
                        wowMesh.NVertices++;
                        index = wowMeshVertices.Count;
                        wowMeshVertices.Add(wowVertex);
                    }

                    wowView.Triangles.Add((ushort)(wowMesh.StartVertex + index));
                    wowMesh.NTriangles++;
                }
                polygonIter.next();
            }
            var boundingBox = meshFunctions.boundingBox;
            wowMesh.CenterBoundingBox = AxisInvert(boundingBox.center);
            wowMesh.Radius = (float) Math.Max(boundingBox.depth/2, boundingBox.width/2);
            var centerMass = new MFloatPoint
            {
                x = vertexSum.x/vertexCounter,
                y = vertexSum.y/vertexCounter,
                z = vertexSum.z/vertexCounter
            };
            wowMesh.CenterMass = AxisInvert(centerMass);
        }

        /// <summary>
        /// Get for each vertex the weights for all influence objects, including zero weights.
        /// </summary>
        /// <param name="vertexWeights"></param>
        /// <param name="influenceObjects"></param>
        /// <param name="meshPath"></param>
        private static void GetMeshWeightData(out List<MDoubleArray> vertexWeights, out MDagPathArray influenceObjects, MDagPath meshPath)
        {
            var fnMesh = new MFnMesh(meshPath);
            vertexWeights = new List<MDoubleArray>();
            influenceObjects = new MDagPathArray();

            // Get any attached skin cluster
            var hasSkinCluster = false;

            // Search the skin cluster affecting this geometry
            var kDepNodeIt = new MItDependencyNodes(MFn.Type.kSkinClusterFilter);

            // Go through each skin cluster in the scene until we find the one connected to this mesh
            while (!kDepNodeIt.isDone && !hasSkinCluster)
            {
                MGlobal.displayInfo("Processing skin cluster...");
                var kObject = kDepNodeIt.thisNode;
                var kSkinClusterFn = new MFnSkinCluster(kObject);
                var uiNumGeometries = kSkinClusterFn.numOutputConnections;
                kSkinClusterFn.influenceObjects(influenceObjects);
                MGlobal.displayInfo("\t uiNumGeometries : " + uiNumGeometries);
                MGlobal.displayInfo("\t influenceOBjects number : " + influenceObjects.Count);

                // Go through each connection on the skin cluster until we get the one connecting to this mesh
                MGlobal.displayInfo("Mesh we are looking for : " + fnMesh.fullPathName);
                for(uint uiGeometry = 0; uiGeometry < uiNumGeometries && !hasSkinCluster; uiGeometry++)
                {
                    var uiIndex = kSkinClusterFn.indexForOutputConnection(uiGeometry);
                    var kInputObject = kSkinClusterFn.inputShapeAtIndex(uiIndex);
                    var kOutputObject = kSkinClusterFn.outputShapeAtIndex(uiIndex);
                    if (!kOutputObject.hasFn(MFn.Type.kMesh)) continue;
                    var fnOutput = new MFnMesh(MDagPath.getAPathTo(kOutputObject));
                    MGlobal.displayInfo("Output object : " + fnOutput.fullPathName);

                    if (fnOutput.fullPathName != fnMesh.fullPathName) continue;

                    hasSkinCluster = true;
                    MGlobal.displayInfo("\t==> A connected skin cluster has been found.");

                    // Go through each vertex (== each component) and save the weights for each one
                    var kGeometryIt = new MItGeometry(kInputObject);
                    while (!kGeometryIt.isDone)
                    {
                        var kComponent = kGeometryIt.currentItem;
                        var kWeightArray = new MDoubleArray();
                        uint uiNumInfluences = 0;
                        kSkinClusterFn.getWeights(meshPath, kComponent, kWeightArray, ref uiNumInfluences);
                        vertexWeights.Add(kWeightArray);

                        kGeometryIt.next();
                    }
                }
                kDepNodeIt.next();
            }
        }

        /// <summary>
        /// Completes BoneLookup and View with the bones called by the mesh's vertices.
        /// Vertices of the mesh must have been previously calculated.
        /// </summary>
        /// <param name="wowMesh"></param>
        /// <param name="wowModel"></param>
        private static void WoWMeshBoneMapping(M2SkinSection wowMesh, M2 wowModel)
        {
            var wowView = wowModel.Views[0];
            var boneRealToLookup = new Dictionary<int, int>();//Reverse lookup

            // Bone Lookup
            for (int i = wowMesh.StartVertex; i < wowMesh.StartVertex + wowMesh.NVertices; i++)
            {
                var vert = wowModel.GlobalVertexList[wowView.Indices[i]];
                var realIndices = vert.BoneIndices;
                var weights = vert.BoneWeights;
                for (var j = 0; j < realIndices.Length; j++)// Map bones used in the Mesh to values in the boneLookup.
                {
                    if (weights[j] == 0) continue;
                    var boneIndex = realIndices[j];//Some index into the M2Bone list.
                    if (boneRealToLookup.ContainsKey(boneIndex)) continue;
                    wowModel.BoneLookup.Add(boneIndex);
                    boneRealToLookup[boneIndex] = wowModel.BoneLookup.Count - 1;
                    wowMesh.NBones++;
                }
                var lookupIndices = new byte[MaxWeightsNumber];
                for (var j = 0; j < realIndices.Length; j++)// Create a vertex property which is vertex bone indices but with lookup values.
                {
                    if (weights[j] == 0) continue;
                    var boneIndex = realIndices[j];//Some index into the M2Bone list.
                    lookupIndices[j] = (byte) boneRealToLookup[boneIndex];
                }
                wowView.Properties.Add(new VertexProperty(lookupIndices));
            }
        }

        /// <summary>
        /// Extract all the geometry from Maya to WoW.
        /// </summary>
        /// <param name="wowModel"></param>
        /// <param name="jointIds">Sometimes we get paths to Joints and we need to link them to their matching M2Bone.</param>
        private static void ExtractMeshes(M2 wowModel, IReadOnlyDictionary<string, short> jointIds)
        {
            var wowView = wowModel.Views[0];
            var meshIter = new MItDag(MItDag.TraversalType.kDepthFirst, MFn.Type.kMesh);
            var collisionFound = false;
            var totalBoundingBox = new MBoundingBox();
            while (!meshIter.isDone)
            {
                var meshPath = new MDagPath();
                meshIter.getPath(meshPath);

                var meshFn = new MFnMesh(meshPath);
                // only want non-history items
                if (!meshFn.isIntermediateObject)//TODO Check if already processed
                {
                    var name = meshFn.name;
                    MGlobal.displayInfo("Mesh name : "+ name);
                    if (name == "Collision") //TODO use custom attribute
                    {
                        if (collisionFound)
                            throw new Exception("More than one collision box has been found. One supported.");
                        MGlobal.displayInfo("\t Collision mesh detected.");
                        ExtractCollisionMesh(meshPath, wowModel);
                        collisionFound = true;
                    }
                    else
                    {

                        var wowMesh = new M2SkinSection {SubmeshId = 0};
                        //TODO give a unique ID maybe ?
                        ExtractGeometryMesh(meshPath, wowMesh, wowModel, jointIds);
                        ExtractMeshShaders(meshPath, wowModel, (ushort) wowView.Submeshes.Count);
                        wowView.Submeshes.Add(wowMesh);
                        WoWMeshBoneMapping(wowMesh, wowModel);
                        totalBoundingBox.expand(meshFn.boundingBox);
                    }
                }
                meshIter.next();
            }
            wowModel.BoundingBox = new CAaBox(AxisInvert(totalBoundingBox.min),
                AxisInvert(totalBoundingBox.max));
            wowModel.BoundingSphereRadius =
                (float) Math.Max(totalBoundingBox.depth/2, totalBoundingBox.width/2);
        }

        /// <summary>
        /// Extract the vertices, normals and triangles of a mesh into the M2 collision data fields.
        /// </summary>
        /// <param name="meshPath"></param>
        /// <param name="wowModel"></param>
        private static void ExtractCollisionMesh(MDagPath meshPath, M2 wowModel)
        {
            var meshFn = new MFnMesh(meshPath);
            wowModel.CollisionBox = new CAaBox(AxisInvert(meshFn.boundingBox.min),
                AxisInvert(meshFn.boundingBox.max));
            wowModel.CollisionSphereRadius =
                (float) Math.Max(meshFn.boundingBox.depth/2, meshFn.boundingBox.width/2);

            var collisionPoints = new MFloatPointArray();
            meshFn.getPoints(collisionPoints, MSpace.Space.kWorld);
            var collisionNormals = new MFloatVectorArray();
            meshFn.getNormals(collisionNormals, MSpace.Space.kWorld);
            var collisionTriangles = new MIntArray();
            meshFn.getTriangles(new MIntArray(), collisionTriangles);
            for (var i = 0; i < collisionPoints.Count; i++)
            {
                wowModel.CollisionVertices.Add(AxisInvert(collisionPoints[i]));
                wowModel.CollisionNormals.Add(AxisInvert(collisionNormals[i]));
            }
            foreach(var vertIndex in collisionTriangles)
                wowModel.CollisionTriangles.Add((ushort) vertIndex);
        }

        /// <summary>
        /// Originally written in C++ by RobTheBloke. 
        /// See https://nccastaff.bournemouth.ac.uk/jmacey/RobTheBloke/www/research/maya/mfnmesh.htm
        /// </summary>
        /// <param name="shadingEngine"></param>
        /// <returns>The shader name.</returns>
        private static MObjectArray GetMaterials(MObject shadingEngine)
        {
            // attach a function set to the shading engine
            var fn = new MFnDependencyNode(shadingEngine);

            // get access to the surfaceShader attribute. This will be connected to
            // lambert , phong nodes etc.
            var sShader = fn.findPlug("surfaceShader");

            // will hold the connections to the surfaceShader attribute
            var materials = new MPlugArray();

            // get the material connected to the surface shader
            sShader.connectedTo(materials, true, false);
            var materialsObjects = new MObjectArray();

            if (materials.Count <= 0) return materialsObjects;
            // if we found a material
            foreach (var plug in materials)
            {
                materialsObjects.Add(plug.node);
            }
            return materialsObjects;
        }

        /// <summary>
        /// Extract a material and its linked texture, if any.
        /// </summary>
        /// <param name="material"></param>
        /// <param name="texUnit"></param>
        /// <param name="wowModel"></param>
        private static void ExtractMaterial(MObject material, M2Batch texUnit, M2 wowModel)//TODO FIXME INFINITE LOOP
        {
            var fnShader = new MFnLambertShader(material);
            // Get lambert out of the shader
            var strName = fnShader.name;
            MGlobal.displayInfo("Detected material : "+strName+" of type "+ material.apiTypeStr);

            var wowMaterial = new M2Material {BlendMode = M2Material.BlendingMode.Opaque};// kLambert
            wowModel.Materials.Add(wowMaterial);
            texUnit.RenderFlags = (ushort) (wowModel.Materials.Count - 1);

            // ReSharper disable once UnusedVariable
            var clrDiffuse = fnShader.color;
            // ReSharper disable once UnusedVariable
            var clrAmbient = fnShader.ambientColor;
            if (material.hasFn(MFn.Type.kReflect))
            {
                var fnReflectShader = new MFnReflectShader(material);
                // ReSharper disable once UnusedVariable
                var clrSpec = fnReflectShader.specularColor;
            }

            var textureCount = 0;
            var uvAnimCount = 0;//TODO uv animations
            // Look for textures at the color plug
            var colorPlug = fnShader.findPlug("color");
            var fileTextureIter = new MItDependencyGraph(colorPlug.node, MFn.Type.kFileTexture,
                MItDependencyGraph.Direction.kUpstream, MItDependencyGraph.Traversal.kDepthFirst,
                MItDependencyGraph.Level.kNodeLevel);
            while (!fileTextureIter.isDone)
            {
                var wowTexture = new M2Texture {Type = M2Texture.TextureType.MonsterSkin1}; //TODO type with name
                //TODO hardcoded textures
                var nodeFn = new MFnDependencyNode(fileTextureIter.thisNode());

                var fileNamePlug = nodeFn.findPlug("fileTextureName");
                string textureFileName;
                fileNamePlug.getValue(out textureFileName);
                MGlobal.displayInfo("\t Texture found : " + textureFileName);

                var wrapUPlug = nodeFn.findPlug("wrapU");
                var wrapU = false;
                wrapUPlug.getValue(ref wrapU);
                var wrapVPlug = nodeFn.findPlug("wrapV");
                var wrapV = false;
                wrapVPlug.getValue(ref wrapV);
                if (wrapU) wowTexture.Flags |= M2Texture.TextureFlags.WrapX;
                if (wrapV) wowTexture.Flags |= M2Texture.TextureFlags.WrapY;

                wowModel.Textures.Add(wowTexture);
                wowModel.TexLookup.Add((short) (wowModel.Textures.Count - 1)); //TODO check it
                if (textureCount == 0) texUnit.Texture = (ushort) (wowModel.TexLookup.Count - 1);
                textureCount++;
                fileTextureIter.next();//maybe now the loop is fixed
            }
            //TODO uv animations
            texUnit.OpCount = (ushort) textureCount;
            for (var i = uvAnimCount; i < textureCount; i++)
            {
                wowModel.UvAnimLookup.Add(-1);
            }
        }

        /// <summary>
        /// Extract all shaders (M2Batch) linked to a mesh.
        /// </summary>
        /// <param name="meshPath"></param>
        /// <param name="wowModel"></param>
        /// <param name="meshNumber"></param>
        private static void ExtractMeshShaders(MDagPath meshPath, M2 wowModel, ushort meshNumber)
        {
            MGlobal.displayInfo("Looking for shaders in mesh "+meshPath.fullPathName);
            var wowView = wowModel.Views[0];
            var meshFn = new MFnMesh(meshPath);

            // get the number of instances
            var numInstances = meshFn.parentCount;
            MGlobal.displayInfo("\t"+numInstances+" instances.");

            // loop through each instance of the mesh
            for (uint i = 0; i < numInstances; ++i)
            {
                // attach a function set to this instances parent transform
                //var fn = new MFnDependencyNode(fnMesh.parent(i));

                // this will hold references to the shaders used on the meshes
                var shaderEngines = new MObjectArray();

                // this is used to hold indices to the materials returned in the object array
                var faceIndices = new MIntArray();

                // get the shaders used by the i'th mesh instance
                meshFn.getConnectedShaders(i, shaderEngines, faceIndices);

                switch (shaderEngines.length)
                {
                    // if no shader applied to the mesh instance
                    case 0:
                        break;
                    // if all faces use the same material
                    case 1:
                        var materials = GetMaterials(shaderEngines[0]);
                        MGlobal.displayInfo("\t\tIn shaderEngine[0], found "+materials.length+" materials.");
                        var texUnit = new M2Batch();
                        texUnit.Flags = 16;
                        texUnit.SubmeshIndex = meshNumber;

                        //TODO Extract Material data
                        ExtractMaterial(materials[0], texUnit, wowModel);

                        //TODO Extract Transparency data
                        var transparency = new M2TextureWeight();
                        transparency.Weight.Timestamps.Add(new M2Array<uint> {0});
                        transparency.Weight.Values.Add(new M2Array<FixedPoint_0_15> {new FixedPoint_0_15(32767)});
                        wowModel.Transparencies.Add(transparency);
                        texUnit.Transparency = (ushort) (wowModel.Transparencies.Count - 1);
                        wowModel.TransLookup.Add((short) (wowModel.Transparencies.Count - 1));

                        wowView.TextureUnits.Add(texUnit);
                        //TODO Shader
                        break;
                    //Multiple materials, each applied only on some faces. 
                    default:
                        throw new NotImplementedException("Cannot handle more than one shaderEngine per mesh.");
                }
            }
        }
    }
}