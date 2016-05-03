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
            var wowView = new M2SkinProfile();

            MGlobal.displayInfo("Building model " + wowModel.Name);

            var boneIds = new Dictionary<MDagPath, int>();//Maps a joint MDagPath to the WoW bone index in the global bone list.

            // JOINTS
            var jointIter = new MItDependencyNodes(MFn.Type.kJoint);
            while(!jointIter.isDone)
            {
                var mayaObject = jointIter.thisNode;
                var joint = new MFnIkJoint(mayaObject);
                var wowBone = new M2Bone();
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
                    wowBone.ParentBone = (short) boneIds[new MFnDagNode(joint.parent(0)).dagPath];
                    wowBone.KeyBoneId = M2Bone.KeyBone.Other;
                    //TODO wowBone.KeyBoneId. Maybe with a special name the user sets ?
                    //Note : M2Bone.submesh_id is wrong in the wiki. Do not try to compute it.
                }
                boneIds[joint.dagPath] = wowModel.Bones.Count;
                wowModel.Bones.Add(wowBone);

                jointIter.next();
            }
            //Temporary TODO proper boneLookup
            for(short i = 0; i < wowModel.Bones.Count; i++) wowModel.BoneLookup.Add(i);

            var meshIter = new MItDependencyNodes(MFn.Type.kMesh);
            while (!meshIter.isDone)
            {
                var wowMesh = new M2SkinSection();
                wowMesh.SubmeshId = 0;//TODO give a unique ID maybe ?
                ExtractMesh(meshIter.thisNode, wowMesh, wowView, wowModel.GlobalVertexList);

                // BONE LINKING TODO
                for (int i = wowMesh.StartVertex; i < wowMesh.StartVertex + wowMesh.NVertices; i++)
                {
                    var vert = wowModel.GlobalVertexList[wowView.Indices[i]];
                    wowView.Properties.Add(new VertexProperty(vert.BoneIndices));//Assumes index in bones == index in boneLookup TODO change it
                }

                wowView.Submeshes.Add(wowMesh);
                meshIter.next();
            }

            // TEXTURES
            var texNodesIter = new MItDependencyNodes(MFn.Type.kFileTexture);
            while (!texNodesIter.isDone)
            {
                var texNode = texNodesIter.thisNode;
                ExtractTexture(texNode, wowModel.Textures);
                texNodesIter.next();
            }

            wowModel.Views.Add(wowView);

            // OTHER

            if(wowModel.Materials.Count == 0) wowModel.Materials.Add(new M2Material());
            if(wowModel.Sequences.Count == 0) wowModel.Sequences.Add(new M2Sequence());
            if(wowModel.Bones.Count == 0) wowModel.Bones.Add(new M2Bone());

            using (var writer = new BinaryWriter(new FileStream(file.expandedFullName, FileMode.Create, FileAccess.Write)))
            {
                wowModel.Save(writer, M2.Format.Classic); //TODO Choose version at output ?
            }
            MGlobal.displayInfo("Done.");
        }

        private static void ExtractTexture(MObject texNode, M2Array<M2Texture> wowTextures)
        {
            var dependencyTexNode = new MFnDependencyNode(texNode);

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
                if (!thisNode.hasFn(MFn.Type.kSkinClusterFilter))
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

        private static void ExtractMesh(MObject meshNode, M2SkinSection wowMesh, M2SkinProfile wowView, ICollection<M2Vertex> globalVertices)
        {
            // nStartVertex + MayaIndex = ViewIndex
            // VERTICES
            ExtractMeshVertices(meshNode, wowMesh, wowView, globalVertices);
            ExtractMeshPolygons(meshNode, wowMesh, wowView);
        }
    }
}