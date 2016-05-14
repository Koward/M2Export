using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Autodesk.Maya.OpenMaya;
using Autodesk.Maya.OpenMayaAnim;
using M2Lib.m2;
using M2Lib.types;

namespace MayaM2
{
    /// <summary>
    /// Extract procedures from Maya to WoW.
    /// </summary>
    public static class MayaToM2
    {
        //As a rule about axis I took here (a,b,c) -> (a,-1*c,b). Hope it's right.
        private const float Epsilon = 0.00001f;

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

                MVector pivot;
                var isRoot = false;
                if (joint.parentCount == 0) isRoot = true;
                else if (!joint.parent(0).hasFn(MFn.Type.kJoint)) isRoot = true;
                if (isRoot)
                {
                    wowBone.ParentBone = -1;
                    wowBone.KeyBoneId = M2Bone.KeyBone.Root;
                    pivot = joint.getTranslation(MSpace.Space.kObject);
                }
                else
                {
                    var parentPath = new MFnIkJoint(joint.parent(0)).fullPathName;
                    wowBone.ParentBone = boneIds[boneRefs[parentPath]];
                    wowBone.KeyBoneId = M2Bone.KeyBone.Other;
                    //TODO wowBone.KeyBoneId. Maybe with a special name the user sets ?
                    //Note : M2Bone.submesh_id is wrong in the wiki. Do not try to compute it.
                    var parentTranslation = new MFnIkJoint(joint.parent(0)).getTranslation(MSpace.Space.kObject);
                    var parentPivot = bones[wowBone.ParentBone].Pivot;
                    //TODO check if axis inversion needed
                    pivot = new MVector
                    {
                        x = parentTranslation.x + parentPivot.X,
                        y = parentTranslation.y + parentPivot.Y,
                        z = parentTranslation.z + parentPivot.Z
                    };
                }
                // Axis inversion here
                wowBone.Pivot = new C3Vector((float) pivot.x, (float) (-1 * pivot.z), (float) pivot.y);

                bones.Add(wowBone);
                jointIter.next();
            }
            var jointNameToBoneIndex = new Dictionary<string, short>();
            foreach (var pair in boneRefs) jointNameToBoneIndex[pair.Key] = boneIds[pair.Value];
            return jointNameToBoneIndex;
        }

        /// <summary>
        /// Faces and vertices UVs, positions and normals.
        /// </summary>
        /// <param name="meshNode"></param>
        /// <param name="wowMesh"></param>
        /// <param name="wowModel"></param>
        /// <param name="jointIds"></param>
        private static void ExtractMeshData(MObject meshNode, M2SkinSection wowMesh, M2 wowModel, IReadOnlyDictionary<string, short> jointIds)
        {
            var wowView = wowModel.Views[0];
            var globalVertices = wowModel.GlobalVertexList;

            //BEGIN Extract mesh data tables.

            // UV Sets
            var uvsets = new MStringArray();
            var meshFunctions = new MFnMesh(meshNode);
            meshFunctions.getUVSetNames(uvsets);

            //Bone Weights
            List<MDoubleArray> vertexWeights;
            MDagPathArray influenceObjects;
            GetMeshWeightData(out vertexWeights, out influenceObjects, MDagPath.getAPathTo(meshNode));

            //Positions
            var positions = new MFloatPointArray();
            meshFunctions.getPoints(positions);

            //Normals
            var normals = new MFloatVectorArray();
            meshFunctions.getVertexNormals(false, normals);

            //END of extracting data tables

            var wowMeshVertices = new List<M2Vertex>();

            wowMesh.StartVertex = (ushort) wowView.Indices.Count;//TODO Check level for big models, like character models with lots of triangles
            wowMesh.StartTriangle = (ushort) wowView.Triangles.Count;//TODO Check level for big models

            var polygonIter = new MItMeshPolygon(meshNode);
            while (!polygonIter.isDone)
            {
                Debug.Assert(polygonIter.polygonVertexCount() == 3, 
                    "Format only handles Triangle faces. A face with "+polygonIter.polygonVertexCount()+" points has been found.\n");
                //for each vertex in the face.
                for (var i = 0; i < polygonIter.polygonVertexCount(); i++)//i = faceRelativeIndex
                {
                    var meshRelativeIndex = (int) polygonIter.vertexIndex(i);
                    var wowVertex = new M2Vertex();
                    var boneCounter = 0;
                    var vertexDoubleWeights = new double[4];

                    //Bone weights
                    for (var b = 0; b < influenceObjects.length; b++)//for each joint
                    {
                        var kJointPath = influenceObjects[b];
                        if (!kJointPath.hasFn(MFn.Type.kJoint) || meshRelativeIndex >= vertexWeights.Count) continue;
                        var kJoint = new MFnDagNode(kJointPath);
                        var jointWeights = vertexWeights[meshRelativeIndex];
                        Debug.Assert(b < jointWeights.Count, "vertexWeights size : " + vertexWeights.Count + " " +
                                                             "\njointWeights for this vertex : " + jointWeights.Count);
                        //Here are a joint&weight for the meshRelativeIndex vertex
                        var boneIndex = jointIds[kJoint.fullPathName];
                        if (jointWeights[b] > Epsilon)
                        {
                            Debug.Assert(boneCounter <= 4, "This vertex is connected to "+boneCounter+" joints. A maximum of 4 is allowed.");
                            wowVertex.BoneIndices[boneCounter] = (byte) boneIndex;
                            vertexDoubleWeights[boneCounter] = jointWeights[b];
                            boneCounter++;
                        }
                    }
                    var weightSum = vertexDoubleWeights.Sum();
                    for(var j = 0; j < vertexDoubleWeights.Length; j++)
                    {
                        wowVertex.BoneWeights[j] = (byte) (vertexDoubleWeights[j] / weightSum * byte.MaxValue);
                    }
                    if (boneCounter == 0)
                    {
                        MGlobal.displayInfo("No bone influence for vertex " + meshRelativeIndex);
                        // There is always at least 1 rootbone, even for static models.
                        wowVertex.BoneIndices[0] = 0;
                        wowVertex.BoneWeights[0] = byte.MaxValue;
                    }
                    if (boneCounter > wowMesh.BoneInfluences) wowMesh.BoneInfluences = (ushort) boneCounter;

                    //Pos&Normal. Notice the axis changing between Maya and WoW
                    var position = positions[meshRelativeIndex];
                    wowVertex.Position = new C3Vector(position.x, -1*position.z, position.y);
                    var normal = normals[meshRelativeIndex];
                    wowVertex.Normal = new C3Vector(normal[0], -1*normal[2], normal[1]);

                    //TODO Generate CenterMass, CenterBoundingBox, Radius

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
        }

        /// <summary>
        /// Get for each vertex the weights for all influence objects.
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
                    var fnOutput = new MFnMesh(kOutputObject);
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
            wowMesh.StartBones = (ushort) wowModel.BoneLookup.Count;
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
                var lookupIndices = new byte[4];
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
            var meshIter = new MItDependencyNodes(MFn.Type.kMesh);
            while (!meshIter.isDone)
            {
                var meshFn = new MFnMesh(meshIter.thisNode);
                // only want non-history items
                if (!meshFn.isIntermediateObject)
                {
                    var name = meshFn.name;
                    MGlobal.displayInfo("Mesh name : "+name);
                    if (name == "Collision")
                    {
                        MGlobal.displayInfo("\t Collision mesh detected.");
                        //TODO collision stuff.
                    }
                    var wowMesh = new M2SkinSection {SubmeshId = 0};
                    //TODO give a unique ID maybe ?
                    ExtractMeshData(meshIter.thisNode, wowMesh, wowModel, jointIds);
                    ExtractMeshShaders(meshIter.thisNode, wowModel, (ushort) (wowView.Submeshes.Count));
                    wowView.Submeshes.Add(wowMesh);
                    WoWMeshBoneMapping(wowMesh, wowModel);
                }
                meshIter.next();
            }
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

            var clrDiffuse = fnShader.color;
            var clrAmbient = fnShader.ambientColor;
            if (material.hasFn(MFn.Type.kReflect))
            {
                var fnReflectShader = new MFnReflectShader(material);
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
        /// <param name="mesh"></param>
        /// <param name="wowModel"></param>
        /// <param name="meshNumber"></param>
        private static void ExtractMeshShaders(MObject mesh, M2 wowModel, ushort meshNumber)
        {
            var wowView = wowModel.Views[0];
            var fnMesh = new MFnMesh(mesh);

            // get the number of instances
            var numInstances = fnMesh.parentCount;

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
                fnMesh.getConnectedShaders(i, shaderEngines, faceIndices);

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