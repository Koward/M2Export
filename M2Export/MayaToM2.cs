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

        //Axis inversion Maya => WoW
        private static C3Vector AxisInvert(float x, float y, float z) => new C3Vector(-1*x, z, y);
        private static C3Vector AxisInvert(MFloatPoint point) => new C3Vector(-1*point.x, point.z, point.y);
        private static C3Vector AxisInvert(MFloatVector point) => new C3Vector(-1*point.x, point.z, point.y);
        private static C3Vector AxisInvert(MPoint point) => new C3Vector((float) (-1*point.x), (float) point.z, (float) point.y);
        //private static C3Vector AxisInvert(MVector point) => new C3Vector((float) (-1*point.x), (float) point.z, (float) point.y);

        public static void ExtractModel(M2 wowModel)
        {
            MGlobal.displayInfo("Building model " + wowModel.Name);

            var jointIds = ExtractJoints(wowModel.Bones);

            var wowView = new M2SkinProfile();//TODO multiple views by decimating ?
            wowModel.Views.Add(wowView);//Because later in the code it always refers to wowModel.Views[0] since it's the only one we do.
            ExtractMeshes(wowModel, jointIds);

            //Static models requirements
            if(wowModel.Sequences.Count == 0) wowModel.Sequences.Add(new M2Sequence());//For non-animated models, basic "Stand"
            if(wowModel.Bones.Count == 0) wowModel.Bones.Add(new M2Bone());//For jointless static models
            if(wowModel.TexUnitLookup.Count == 0) wowModel.TexUnitLookup.Add(0);
            if(wowModel.UvAnimLookup.Count == 0) wowModel.UvAnimLookup.Add(-1);
        }
        /// <summary>
        /// Creates the bone hierarchy.
        /// </summary>
        /// <param name="bones"></param>
        private static Dictionary<string, short> ExtractJoints(IList<M2Bone> bones)
        {
            var boneRefs = new Dictionary<string, M2Bone>();//Maps a joint name to the WoW bone instance.
            var boneIds = new Dictionary<M2Bone, short>();//Maps a bone to its index
            //Depth First => parent always processed before child
            var jointIter = new MItDag(MItDag.TraversalType.kDepthFirst, MFn.Type.kJoint);
            while (!jointIter.isDone)//Create an M2Bone for all Joints.
            {
                var wowBone = new M2Bone();
                boneIds[wowBone] = (short) bones.Count;
                boneRefs[jointIter.fullPathName] = wowBone;

                var jointPath = new MDagPath();
                jointIter.getPath(jointPath);
                var joint = new MFnIkJoint(jointPath);

                var isRoot = false;
                if (joint.parentCount == 0) isRoot = true;
                else if (!joint.parent(0).hasFn(MFn.Type.kJoint)) isRoot = true;
                if (isRoot)
                {
                    wowBone.ParentBone = -1;
                    wowBone.KeyBoneId = M2Bone.KeyBone.Root;
                }
                else
                {
                    var parentPath = new MFnIkJoint(joint.parent(0)).fullPathName;
                    wowBone.ParentBone = boneIds[boneRefs[parentPath]];
                    wowBone.KeyBoneId = GetBoneType(joint);
                    //TODO wowBone.KeyBoneId. Maybe with a special name the user sets ?
                    //Note : there is an existing thing in Maya
                    //Note : M2Bone.submesh_id is wrong in the wiki. Do not try to compute it.
                }
                var jointPosition = new MDoubleArray();
                //This MEL has no C++/C# equivalent, it seems.
                MGlobal.executeCommand("joint -q -position " + jointPath.fullPathName, jointPosition);
                wowBone.Pivot = AxisInvert(
                    (float) jointPosition[0],
                    (float) jointPosition[1],
                    (float) jointPosition[2]);

                bones.Add(wowBone);
                jointIter.next();
            }

            var jointNameToBoneIndex = new Dictionary<string, short>();
            foreach (var pair in boneRefs) jointNameToBoneIndex[pair.Key] = boneIds[pair.Value];
            return jointNameToBoneIndex;
        }

        /// <summary>
        /// Get bone type from joint labelling.
        /// </summary>
        /// <param name="joint">Joint to get labelling attributes from.</param>
        /// <returns>A WoW bone label.</returns>
        //TODO finish cases.
        // ReSharper disable once SuggestBaseTypeForParameter
        private static M2Bone.KeyBone GetBoneType(MFnIkJoint joint)
        {
            var jointName = joint.fullPathName;
            var type = MGlobal.executeCommandStringResult("getAttr -asString "+jointName+".type");
            var side = MGlobal.executeCommandStringResult("getAttr -asString "+jointName+".side");
            switch (side)
            {
                case "Left":
                    switch (type)
                    {
                        case "Shoulder": return M2Bone.KeyBone.ShoulderL;
                        case "Index Finger": return M2Bone.KeyBone.IndexFingerL;
                        case "Middle Finger": return M2Bone.KeyBone.MiddleFingerL;
                        case "Ring Finger": return M2Bone.KeyBone.RingFingerL;
                        case "Pinky Finger": return M2Bone.KeyBone.PinkyFingerL;
                        case "Thumb": return M2Bone.KeyBone.ThumbL;
                        case "Other":
                            var otherType = MGlobal.executeCommandStringResult("getAttr -asString "+jointName+".otherType");
                            if(otherType == "Arm") return M2Bone.KeyBone.ArmL;
                            break;
                    }
                    break;
                case "Right":
                    switch (type)
                    {
                        case "Shoulder": return M2Bone.KeyBone.ShoulderR;
                        case "Index Finger": return M2Bone.KeyBone.IndexFingerR;
                        case "Middle Finger": return M2Bone.KeyBone.MiddleFingerR;
                        case "Ring Finger": return M2Bone.KeyBone.RingFingerR;
                        case "Pinky Finger": return M2Bone.KeyBone.PinkyFingerR;
                        case "Thumb": return M2Bone.KeyBone.ThumbR;
                        case "Other":
                            var otherType = MGlobal.executeCommandStringResult("getAttr -asString " + jointName + ".otherType");
                            if (otherType == "Arm") return M2Bone.KeyBone.ArmR;
                            break;
                    }
                    break;
            }
            switch (type)
            {
                case "Root": return M2Bone.KeyBone.Root;
                case "Hip": return M2Bone.KeyBone.Waist;
                case "Spine": return M2Bone.KeyBone.SpineLow;
                case "Head": return M2Bone.KeyBone.Head;
                case "Other":
                    //TODO cases
                    var otherType = MGlobal.executeCommandStringResult("getAttr -asString " + jointName + ".otherType");
                    break;
            }
            return M2Bone.KeyBone.Other;
        }

        /// <summary>
        /// Faces and vertices UVs, positions and normals.
        /// </summary>
        /// <param name="meshPath"></param>
        /// <param name="wowMesh"></param>
        /// <param name="wowModel"></param>
        /// <param name="jointIds"></param>
        private static void ExtractMeshData(MDagPath meshPath, M2SkinSection wowMesh, M2 wowModel, IReadOnlyDictionary<string, short> jointIds)
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
                    var vertexDoubleWeights = new List<double>();
                    var vertexBoneIndices = new List<int>();

                    //Bone weights
                    for (var b = 0; b < influenceObjects.length; b++)//for each joint
                    {
                        var kJointPath = influenceObjects[b];
                        if (!kJointPath.hasFn(MFn.Type.kJoint) || meshRelativeIndex >= vertexWeights.Count) continue;
                        var kJoint = new MFnDagNode(kJointPath);
                        Debug.Assert(b < jointsWeightsForThisVertex.Count, "vertexWeights size : " + vertexWeights.Count + " " +
                                                             "\njointWeights for this vertex : " + jointsWeightsForThisVertex.Count);
                        //Here are a joint&weight for the meshRelativeIndex vertex
                        var boneIndex = jointIds[kJoint.fullPathName];
                        if (jointsWeightsForThisVertex[b] > Epsilon)
                        {
                            vertexBoneIndices.Add((byte) boneIndex);
                            vertexDoubleWeights.Add(jointsWeightsForThisVertex[b]);
                        }
                    }
                    if(vertexDoubleWeights.Count > MaxWeightsNumber)
                        MGlobal.displayWarning("This vertex is connected to more than "+MaxWeightsNumber+ " joints. Only the " + MaxWeightsNumber + " biggest will be exported.");
                    vertexDoubleWeights = vertexDoubleWeights.OrderByDescending(w => w).ToList();
                    double weightSum = 0;
                    for (var j = 0; j < vertexDoubleWeights.Count && j < MaxWeightsNumber; j++) weightSum += vertexDoubleWeights[j];
                    for(var j = 0; j < vertexDoubleWeights.Count && j < MaxWeightsNumber; j++)
                    {
                        wowVertex.BoneIndices[j] = (byte) vertexBoneIndices[j];
                        wowVertex.BoneWeights[j] = (byte) (vertexDoubleWeights[j] / weightSum * byte.MaxValue);
                    }
                    if (vertexDoubleWeights.Count == 0)
                    {
                        MGlobal.displayInfo("No bone influence for vertex " + meshRelativeIndex);
                        // There is always at least 1 rootbone, even for static models.
                        wowVertex.BoneIndices[0] = 0;
                        wowVertex.BoneWeights[0] = byte.MaxValue;
                    }
                    if (vertexDoubleWeights.Count > wowMesh.BoneInfluences)
                        wowMesh.BoneInfluences = (ushort) Math.Max(MaxWeightsNumber, vertexDoubleWeights.Count);

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
            //var meshIter = new MItDag(MItDag.TraversalType.kDepthFirst, MFn.Type.kMesh);
            var meshIter = new MItDependencyNodes(MFn.Type.kMesh);
            var collisionFound = false;
            var totalBoundingBox = new MBoundingBox();
            while (!meshIter.isDone)
            {
                var meshPath = new MDagPath();//TODO FIXME use DagPath
                //MDagPath.getAPathTo(meshIter.currentItem(), meshPath);
                MDagPath.getAPathTo(meshIter.thisNode, meshPath);

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
                        ExtractMeshData(meshPath, wowMesh, wowModel, jointIds);
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
        private static void ExtractMaterial(MObject material, M2Batch texUnit, M2 wowModel)
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

            // Look for textures at the color plug
            var colorPlug = fnShader.findPlug("color");
            var fileTextureIter = new MItDependencyGraph(colorPlug.node, MFn.Type.kFileTexture,
                MItDependencyGraph.Direction.kUpstream, MItDependencyGraph.Traversal.kDepthFirst,
                MItDependencyGraph.Level.kNodeLevel);
            if (fileTextureIter.isDone) return;
            var wowTexture = new M2Texture {Type = M2Texture.TextureType.MonsterSkin1};//TODO type with name
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
            wowModel.TexLookup.Add((short)(wowModel.Textures.Count - 1));//TODO check it
            texUnit.Texture = (ushort) (wowModel.TexLookup.Count - 1);
        }

        /// <summary>
        /// Extract all shaders (M2Batch) linked to a mesh.
        /// </summary>
        /// <param name="meshPath"></param>
        /// <param name="wowModel"></param>
        /// <param name="meshNumber"></param>
        private static void ExtractMeshShaders(MDagPath meshPath, M2 wowModel, ushort meshNumber)
        {
            var wowView = wowModel.Views[0];
            var meshFn = new MFnMesh(meshPath);

            // get the number of instances
            var numInstances = meshFn.parentCount;

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