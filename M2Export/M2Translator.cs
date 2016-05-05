using System;
using System.Collections.Generic;
using System.IO;
using Autodesk.Maya.OpenMaya;
using Autodesk.Maya.OpenMayaAnim;
using M2Lib.m2;
using M2Lib.types;

[assembly: MPxFileTranslatorClass(typeof(M2Export.M2Translator), "World of Warcraft M2", null, null, null)]

namespace M2Export
{
    public class M2Translator : MPxFileTranslator
    {
        protected static string FExtension = "m2";

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

            var wowModel = new M2();
            wowModel.Name = file.rawName.Substring(0, file.rawName.Length - 3);// Name is fileName without .m2 extension

            MGlobal.displayInfo("Building model " + wowModel.Name);

            ExtractJoints(wowModel.Bones);
            ExtractTextures(wowModel);

            var wowView = new M2SkinProfile();//TODO multiple views by decimating ?
            wowModel.Views.Add(wowView);//Because later in the code it always refers to wowModel.Views[0] since it's the only one we do.
            ExtractMeshes(wowModel);

            //Static models requirements
            if(wowModel.Materials.Count == 0) wowModel.Materials.Add(new M2Material());
            if(wowModel.Sequences.Count == 0) wowModel.Sequences.Add(new M2Sequence());//For non-animated models, basic "Stand"
            if(wowModel.Bones.Count == 0) wowModel.Bones.Add(new M2Bone());//For jointless static models

            using (var writer = new BinaryWriter(new FileStream(file.expandedFullName, FileMode.Create, FileAccess.Write)))
            {
                wowModel.Save(writer, M2.Format.LichKing); //TODO Choose version at output ?
            }
            MGlobal.displayInfo("Done.");
        }

        private static void ExtractJoints(ICollection<M2Bone> bones)
        {
            var boneRefs = new Dictionary<string, M2Bone>();//Maps a joint name to the WoW bone instance.
            var boneIds = new Dictionary<M2Bone, short>();//Maps a bone to its index
            var jointIter = new MItDag(MItDag.TraversalType.kDepthFirst, MFn.Type.kJoint);
            while (!jointIter.isDone)//Create an M2Bone for all Joints.
            {
                var wowBone = new M2Bone();
                boneIds[wowBone] = (short) bones.Count;
                boneRefs[jointIter.fullPathName] = wowBone;
                bones.Add(wowBone);
                jointIter.next();
            }

            jointIter = new MItDag(MItDag.TraversalType.kDepthFirst, MFn.Type.kJoint);//Reset the iterator
            while(!jointIter.isDone)
            {
                var jointPath = new MDagPath();
                jointIter.getPath(jointPath);
                var joint = new MFnIkJoint(jointPath);
                var wowBone = boneRefs[jointIter.fullPathName];
                var mayaPivot = joint.rotatePivot(MSpace.Space.kTransform);
                wowBone.Pivot = new C3Vector((float) mayaPivot.x, (float) mayaPivot.y, (float) mayaPivot.z);

                //Bone parenting
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
                    if(!boneRefs.ContainsKey(parentPath)) {
                        foreach (var path in boneRefs.Keys)
                            MGlobal.displayInfo("\t"+path+"\n");
                    }
                    wowBone.ParentBone = boneIds[boneRefs[parentPath]];
                    wowBone.KeyBoneId = M2Bone.KeyBone.Other;
                    //TODO wowBone.KeyBoneId. Maybe with a special name the user sets ?
                    //Note : M2Bone.submesh_id is wrong in the wiki. Do not try to compute it.
                }
                jointIter.next();
            }
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
                string filename;
                fileNamePlug.getValue(out filename);
                var wrapUPlug = dependencyTexNode.findPlug("wrapU");
                var wrapU = false;
                wrapUPlug.getValue(ref wrapU);
                var wrapVPlug = dependencyTexNode.findPlug("wrapV");
                var wrapV = false;
                wrapVPlug.getValue(ref wrapV);
                

                var wowTexture = new M2Texture();
                wowTexture.Name = filename;//TODO replace extension by BLP
                if(wrapU) wowTexture.Flags |= M2Texture.TextureFlags.WrapX;
                if(wrapV) wowTexture.Flags |= M2Texture.TextureFlags.WrapY;
                wowTexture.Type = M2Texture.TextureType.Skin;
                //TODO Type, identification with name ?
                wowTextures.Add(wowTexture);
                wowModel.TexLookup.Add((short) (wowTextures.Count - 1));
                texIter.next();
            }
        }

        private static void ExtractMeshPolygons(MObject meshNode, M2SkinSection wowMesh, M2SkinProfile wowView)
        {
            wowMesh.StartTriangle = (ushort)wowView.Triangles.Count;//TODO Check level for big models

            var polygonIter = new MItMeshPolygon(meshNode);
            while (!polygonIter.isDone)
            {
                var points = new MPointArray();// Unused
                var vertexList = new MIntArray();
                polygonIter.getTriangles(points, vertexList);
                foreach (var index in vertexList)
                {
                    wowView.Triangles.Add((ushort)(wowMesh.StartVertex + index));//Added StartVertex so the index become View relative and no Mesh relative. TODO level here too ?
                    wowMesh.NTriangles++;
                }

                polygonIter.next();
            }
        }
        // ReSharper disable once SuggestBaseTypeForParameter
        private static ushort ExtractMeshWeights(out float[][] vertexBoneWeights, out uint[][] vertexBoneIndices, MFnMesh mesh)
        {
            ushort boneInfluences = 0;
            vertexBoneWeights = null;
            vertexBoneIndices = null;
            var inMeshPlug = mesh.findPlug("inMesh");
            if (!inMeshPlug.isConnected) return boneInfluences;
            var dgIt = new MItDependencyGraph(inMeshPlug);
            while (!dgIt.isDone)
            {
                var thisNode = dgIt.thisNode();
                if (thisNode.hasFn(MFn.Type.kSkinClusterFilter))
                {
                    var skinCluster = new MFnSkinCluster(thisNode);
                    var weightListPlug = skinCluster.findPlug("weightList");
                    vertexBoneWeights = new float[weightListPlug.numElements][];
                    vertexBoneIndices = new uint[weightListPlug.numElements][];
                    for (uint i = 0; i < weightListPlug.numElements; i++)//for each vertex
                    {
                        var weightPlug = weightListPlug.elementByPhysicalIndex(i).child(0);
                        if(weightPlug.numElements > 4) throw new Exception("A vertex is influenced by "+weightPlug.numElements+". The maximum is 4.");
                        if (weightPlug.numElements > boneInfluences)
                            boneInfluences = (ushort) weightPlug.numElements;//M2SkinSection.BoneInfluences
                        vertexBoneWeights[i] = new float[4];
                        vertexBoneIndices[i] = new uint[4];
                        for (uint j = 0; j < weightPlug.numElements; j++)//for each weight
                        {
                            weightPlug.elementByPhysicalIndex(j).getValue(vertexBoneWeights[i][j]);
                            vertexBoneIndices[i][j] = weightPlug.elementByPhysicalIndex(j).logicalIndex;
                        }
                    }

                }
                dgIt.next();
            }
            return boneInfluences;
        }

        private static void ExtractMeshVertices(MObject meshNode, M2SkinSection wowMesh, M2SkinProfile wowView,
            ICollection<M2Vertex> globalVertices)
        {
            // nStartVertex + MayaIndex = ViewIndex
            var mesh = new MFnMesh(meshNode);

            var vertexPositions = new MFloatPointArray();// Each vertex will be refered as its index into these lists.
            var vertexNormals = new MFloatVectorArray();
            var vertexUs = new MFloatArray();
            var vertexVs = new MFloatArray();
            float[][] vertexBoneWeights;
            uint[][] vertexBoneIndices;
            mesh.getPoints(vertexPositions);
            mesh.getVertexNormals(false, vertexNormals);
            mesh.getUVs(vertexUs, vertexVs);
            wowMesh.BoneInfluences = ExtractMeshWeights(out vertexBoneWeights, out vertexBoneIndices, mesh);
            //TODO Generate CenterMass, CenterBoundingBox, Radius

            wowMesh.StartVertex = (ushort)wowView.Indices.Count;//TODO Check level for big models, like character models with lots of triangles
            wowMesh.NVertices = (ushort) mesh.numVertices;
            for (var i = 0; i < mesh.numVertices; i++)
            {
                //There is a fix with the axis for positions, normals (z/y inversion) and uv.
                var wowVertex = new M2Vertex();

                var mayaVertexPosition = vertexPositions[i];
                wowVertex.Position = new C3Vector(mayaVertexPosition.x, mayaVertexPosition.z, mayaVertexPosition.y);
                var mayaVertexNormal = vertexNormals[i];
                wowVertex.Normal = new C3Vector(mayaVertexNormal.x, mayaVertexNormal.z, mayaVertexNormal.y);
                wowVertex.TexCoords[0] = new C2Vector(vertexUs[i], 1.0F - vertexVs[i]);
                //TODO multiple textures per mesh. Maybe with getUVSets which gives multiple arrays of u and v you can index with vertex id ?
                if (vertexBoneIndices != null)
                {
                    for (var j = 0; j < vertexBoneIndices[i].Length; j++)
                        wowVertex.BoneIndices[j] = (byte) vertexBoneIndices[i][j];
                }
                if (vertexBoneWeights != null)
                {
                    for (var j = 0; j < vertexBoneWeights[i].Length; j++)
                        wowVertex.BoneWeights[j] = (byte) (vertexBoneWeights[i][j] * byte.MaxValue);
                }

                wowView.Indices.Add((ushort)globalVertices.Count);//All of this to do the wow Vertex mapping mesh->view->global
                globalVertices.Add(wowVertex);
            }
        }

        private static void ExtractMeshes(M2 wowModel)
        {
            var wowView = wowModel.Views[0];
            var globalVertices = wowModel.GlobalVertexList;
            var meshIter = new MItDependencyNodes(MFn.Type.kMesh);
            while (!meshIter.isDone)
            {
                var wowMesh = new M2SkinSection();
                wowMesh.SubmeshId = 0;//TODO give a unique ID maybe ?
                ExtractMeshVertices(meshIter.thisNode, wowMesh, wowView, globalVertices);
                ExtractMeshPolygons(meshIter.thisNode, wowMesh, wowView);

                // BONE LOOKUP LINKING
                wowMesh.StartBones = (ushort) wowModel.BoneLookup.Count;
                for (int i = wowMesh.StartVertex; i < wowMesh.StartVertex + wowMesh.NVertices; i++)
                {
                    var vert = globalVertices[wowView.Indices[i]];
                    var realIndices = vert.BoneIndices;
                    var boneRealToLookup = new Dictionary<int, int>();//Reverse lookup
                    for (var j = 0; j < realIndices.Length && realIndices[j] != 0; j++)
                    {
                        var refBoneIndex = realIndices[j];//Some index into the M2Bone list.
                        boneRealToLookup[refBoneIndex] = wowModel.BoneLookup.Count;
                        wowModel.BoneLookup.Add(refBoneIndex);
                        wowMesh.NBones++;
                    }
                    var lookupIndices = new byte[]
                    {
                        (byte) boneRealToLookup[realIndices[0]],
                        (byte) boneRealToLookup[realIndices[1]],
                        (byte) boneRealToLookup[realIndices[2]],
                        (byte) boneRealToLookup[realIndices[3]]
                    };
                    wowView.Properties.Add(new VertexProperty(lookupIndices));
                }

                wowView.Submeshes.Add(wowMesh);
                ExtractMeshShaders(meshIter.thisNode, wowModel);//TODO Debug printing of shaders
                meshIter.next();
            }
        }


        /// <summary>
        /// Originally written in C++ by RobTheBloke. 
        /// See https://nccastaff.bournemouth.ac.uk/jmacey/RobTheBloke/www/research/maya/mfnmesh.htm
        /// </summary>
        /// <param name="shadingEngine"></param>
        /// <returns>The shader name.</returns>
        private static string GetShaderName(MObject shadingEngine)
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

            if (materials.Count <= 0) return "none";
            // if we found a material
            var fnMat = new MFnDependencyNode(materials[0].node);
            return fnMat.name;
        }


        private static void ExtractMeshShaders(MObject mesh, M2 wowModel)
        {
            var wowView = wowModel.Views[0];
            var fnMesh = new MFnMesh(mesh);

            // get the number of instances
            var numInstances = fnMesh.parentCount;

            // loop through each instance of the mesh
            for (uint i = 0; i < numInstances; ++i)
            {
                // attach a function set to this instances parent transform
                var fn = new MFnDependencyNode(fnMesh.parent(i));

                // this will hold references to the shaders used on the meshes
                var shaders = new MObjectArray();

                // this is used to hold indices to the materials returned in the object array
                var faceIndices = new MIntArray();

                // get the shaders used by the i'th mesh instance
                fnMesh.getConnectedShaders(i, shaders, faceIndices);

                switch (shaders.length)
                {
                    // if no shader applied to the mesh instance
                    case 0:
                        break;
                    // if all faces use the same material
                    case 1:
                        var shaderName = GetShaderName(shaders[0]);//example : lambert1
                        var texUnit = new M2Batch();
                        texUnit.Flags = 16;

                        var material = new M2Material();
                        wowModel.Materials.Add(material);
                        texUnit.RenderFlags = (ushort) (wowModel.Materials.Count - 1);

                        var transparency = new M2TextureWeight();
                        transparency.Weight.Timestamps.Add(new M2Array<uint> {0});
                        transparency.Weight.Values.Add(new M2Array<FixedPoint_0_15> {new FixedPoint_0_15(32767)});
                        wowModel.Transparencies.Add(transparency);
                        texUnit.Transparency = (ushort) (wowModel.Transparencies.Count - 1);
                        wowModel.TransLookup.Add((short) (wowModel.Transparencies.Count - 1));

                        wowView.TextureUnits.Add(texUnit);
                        //TODO Shader
                        break;
                    default:
                        // i'm going to sort the face indicies into groups based on
                        // the applied material - might as well... ;)
                        // also, set to same size as num of shaders
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
                        throw new NotImplementedException("Cannot handle more than one shader per mesh.");
                }
            }
        }

    }
}