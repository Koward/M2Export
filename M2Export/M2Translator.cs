using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using Autodesk.Maya.OpenMaya;
using Autodesk.Maya.OpenMayaAnim;
using M2Lib.m2;
using M2Lib.types;

[assembly: MPxFileTranslatorClass(typeof(M2Export.M2Translator), "World of Warcraft M2", null, null, null)]

namespace M2Export
{
    public class M2Translator : MPxFileTranslator
    {
        protected const string FExtension = "m2";
        protected const M2.Format WoWVersion = M2.Format.LichKing;
        protected const float Epsilon = 0.00001f;

        public override string defaultExtension()
        {
            return FExtension;
        }

        public override bool haveReadMethod()
        {
            return false;
        }

        public override bool haveWriteMethod()
        {
            return true;
        }

        /// <summary>
        /// Maya calls this method to find out if this translator is capable of
        /// handling the given file.
        /// </summary>
        /// <param name="file"></param>
        /// <param name="buffer"></param>
        /// <param name="bufferLen"></param>
        /// <returns></returns>
        public override MFileKind identifyFile(MFileObject file, string buffer, short bufferLen)
        {
            //Based on extension
            //TODO Check magic number if bufferLen >= 4 and the first characters are M,D,2,0
            var fileName = file.name;
            var fileNameLen = fileName.Length;
            var startOfExtension = fileName.IndexOf('.') + 1;

            if ((startOfExtension > 0)
            && (startOfExtension < fileNameLen)
            && (fileName.Substring(startOfExtension, fileNameLen) == FExtension))
            {
                return MFileKind.kIsMyFileType;
            }
            return MFileKind.kNotMyFileType;
        }

        /// <summary>
        /// Maya calls this method to have the translator write out a file.
        /// </summary>
        /// <param name="file"></param>
        /// <param name="optionsString"></param>
        /// <param name="mode"></param>
        public override void writer(MFileObject file, string optionsString, FileAccessMode mode)
        {
            MGlobal.displayInfo("Exporting to M2..");

            var wowModel = new M2 {Name = file.rawName.Substring(0, file.rawName.Length - 3)};
            // Name is fileName without .m2 extension

            MGlobal.displayInfo("Building model " + wowModel.Name);

            var jointIds = ExtractJoints(wowModel.Bones);
            ExtractTextures(wowModel);

            var wowView = new M2SkinProfile();//TODO multiple views by decimating ?
            wowModel.Views.Add(wowView);//Because later in the code it always refers to wowModel.Views[0] since it's the only one we do.
            ExtractMeshes(wowModel, jointIds);

            //Static models requirements
            if(wowModel.Sequences.Count == 0) wowModel.Sequences.Add(new M2Sequence());//For non-animated models, basic "Stand"
            if(wowModel.Bones.Count == 0) wowModel.Bones.Add(new M2Bone());//For jointless static models
            if(wowModel.TexUnitLookup.Count == 0) wowModel.TexUnitLookup.Add(0);
            if(wowModel.UvAnimLookup.Count == 0) wowModel.UvAnimLookup.Add(-1);

            using (var writer = new BinaryWriter(new FileStream(file.expandedFullName, FileMode.Create, FileAccess.Write)))
            {
                wowModel.Save(writer, WoWVersion); //TODO Choose version at output ?
            }
            MGlobal.displayInfo("Done.");
        }

        /// <summary>
        /// Creates the bone hierarchy.
        /// </summary>
        /// <param name="bones"></param>
        private static Dictionary<string, short> ExtractJoints(List<M2Bone> bones)
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

                MVector wowPivot;
                var isRoot = false;
                if (joint.parentCount == 0) isRoot = true;
                else if (!joint.parent(0).hasFn(MFn.Type.kJoint)) isRoot = true;
                if (isRoot)
                {
                    wowBone.ParentBone = -1;
                    wowBone.KeyBoneId = M2Bone.KeyBone.Root;
                    wowPivot = joint.getTranslation(MSpace.Space.kObject);
                }
                else
                {
                    var parentPath = new MFnIkJoint(joint.parent(0)).fullPathName;
                    wowBone.ParentBone = boneIds[boneRefs[parentPath]];
                    wowBone.KeyBoneId = M2Bone.KeyBone.Other;
                    //TODO wowBone.KeyBoneId. Maybe with a special name the user sets ?
                    //Note : M2Bone.submesh_id is wrong in the wiki. Do not try to compute it.
                    var parentTranslation = new MFnIkJoint(joint.parent(0)).getTranslation(MSpace.Space.kObject);
                    var wowParentPivot = bones[wowBone.ParentBone].Pivot;
                    //TODO check if axis inversion needed
                    wowPivot = new MVector
                    {
                        x = parentTranslation.x + wowParentPivot.X,
                        y = parentTranslation.y + wowParentPivot.Y,
                        z = parentTranslation.z + wowParentPivot.Z
                    };
                }
                wowBone.Pivot = new C3Vector((float) wowPivot.x, (float) wowPivot.y, (float) wowPivot.z);

                bones.Add(wowBone);
                jointIter.next();
            }
            var jointNameToBoneIndex = new Dictionary<string, short>();
            foreach (var pair in boneRefs) jointNameToBoneIndex[pair.Key] = boneIds[pair.Value];
            return jointNameToBoneIndex;
        }

        private static void ExtractTextures(M2 wowModel)
        {
            var wowTextures = wowModel.Textures;
            var texIter = new MItDependencyNodes(MFn.Type.kFileTexture);
            while (!texIter.isDone)
            {
                var dependencyTexNode = new MFnDependencyNode(texIter.thisNode);

                //Get attributes
                var fileNamePlug = dependencyTexNode.findPlug("fileTextureName");
                string mayaFilename;
                fileNamePlug.getValue(out mayaFilename);
                var wrapUPlug = dependencyTexNode.findPlug("wrapU");
                var wrapU = false;
                wrapUPlug.getValue(ref wrapU);
                var wrapVPlug = dependencyTexNode.findPlug("wrapV");
                var wrapV = false;
                wrapVPlug.getValue(ref wrapV);
                

                var wowTexture = new M2Texture();
                wowTexture.Type = M2Texture.TextureType.MonsterSkin1;
                //TODO hardcoded textures
                //if()
                //wowTexture.Name = mayaFilename;//TODO replace extension by BLP
                //else

                if(wrapU) wowTexture.Flags |= M2Texture.TextureFlags.WrapX;
                if(wrapV) wowTexture.Flags |= M2Texture.TextureFlags.WrapY;
                //TODO Type, identification with name ?
                wowTextures.Add(wowTexture);
                wowModel.TexLookup.Add((short) (wowTextures.Count - 1));//TODO check it
                texIter.next();
            }
        }

        /// <summary>
        /// Faces and vertices UVs, positions and normals.
        /// </summary>
        /// <param name="meshNode"></param>
        /// <param name="wowMesh"></param>
        /// <param name="wowView"></param>
        /// <param name="globalVertices"></param>
        /// <param name="jointIds"></param>
        private static void ExtractMeshData(MObject meshNode, M2SkinSection wowMesh, M2SkinProfile wowView, List<M2Vertex> globalVertices, Dictionary<string, short> jointIds)
        {
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

        private static void GetMeshWeightData(out List<MDoubleArray> vertexWeights, out MDagPathArray influenceObjects, MDagPath meshPath)
        {
            var fnMesh = new MFnMesh(meshPath);
            vertexWeights = new List<MDoubleArray>();
            influenceObjects = new MDagPathArray();

            /*
            var shaderArray = new MObjectArray();//TODO maybe use that somewhere
            var shaderIntArray = new MIntArray();

            // Get connected shaders
            mesh.getConnectedShaders(0, shaderArray, shaderIntArray);
            */

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
                    //TODO fixme this is were everything is wrong
                    MGlobal.displayInfo("Output object : " + fnOutput.fullPathName);

                    if (fnOutput.fullPathName != fnMesh.fullPathName) continue;

                    hasSkinCluster = true;
                    MGlobal.displayInfo("\t==>A connected skin cluster has been found.");

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

        private static void ExtractMeshes(M2 wowModel, Dictionary<string, short> jointIds)
        {
            var wowView = wowModel.Views[0];
            var globalVertices = wowModel.GlobalVertexList;
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
                    var wowMesh = new M2SkinSection();
                    wowMesh.SubmeshId = 0;//TODO give a unique ID maybe ?
                    ExtractMeshData(meshIter.thisNode, wowMesh, wowView, globalVertices, jointIds);
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

        private static string GetMaterialData(MObject material)
        {
            var fnShader = new MFnLambertShader(material);
            // Get lambert out of the shader
            var strName = fnShader.name;
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
            var nodeFn = new MFnDependencyNode(fileTextureIter.thisNode());
            var plug = nodeFn.findPlug("fileTextureName");
            string textureFileName;
            plug.getValue(out textureFileName);
            //TODO something with texture FileName
            return textureFileName;
        }

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
                        var shaderName = new MFnDependencyNode(materials[0]).name;//example : lambert1. TODO find more names
                        MGlobal.displayInfo("Detected material : "+shaderName+" of type "+ materials[0].apiTypeStr);//TODO remove once handled
                        MGlobal.displayInfo("First material texture : "+ GetMaterialData(materials[0]));//TODO remove once handled
                        var texUnit = new M2Batch();
                        texUnit.Flags = 16;
                        texUnit.SubmeshIndex = meshNumber;

                        //TODO Extract Material data
                        var material = new M2Material();
                        material.BlendMode = M2Material.BlendingMode.Opaque;//lambert1
                        wowModel.Materials.Add(material);
                        texUnit.RenderFlags = (ushort) (wowModel.Materials.Count - 1);

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
                        // i'm going to sort the face indicies into groups based on
                        // the applied material
                        /*
                        var facesByMatId = new List<List<int>>((int) shaders.length);

                        // put face index into correct array
                        for (var j = 0; j < faceIndices.length; ++j)
                        {
                            facesByMatId[faceIndices[j]].Add(j);
                        }

                        // now write each material and the face indices that use them
                        for (var j = 0; j < shaders.length; ++j)
                        {
                            var thisShaderName = GetShaderName(shaders[j]);
                            MGlobal.displayInfo(thisShaderName + " " + facesByMatId[j].Count);
                        }
                        */

                }
            }
        }
    }
}