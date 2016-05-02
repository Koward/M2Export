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
        protected static string FPluginName = "M2Export";
        protected static string FTranslatorName = "World of Warcraft M2";

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

        public static void SetPluginName(string name)
        {
            FPluginName = name;
        }

        public static string TranslatorName()
        {
            return FTranslatorName;
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
                wowModel.Bones.Add(wowBone);
                boneIds[joint.dagPath] = wowModel.Bones.Count - 1;

                /*
                //Iterate through childs to find transforms and then subchilds to find meshes, then extract them.
                for (uint i = 0; i < joint.childCount; i++)
                {
                    var child = joint.child(i);
                    if (!child.hasFn(MFn.Type.kTransform)) continue;
                    var childTransform = new MFnTransform(child);
                    for (uint j = 0; j < childTransform.childCount; j++)
                    {
                        var subChild = childTransform.child(i);
                        if (!subChild.hasFn(MFn.Type.kMesh)) continue;
                        //var subChildMesh = new MFnMesh(subChild);
                        ExtractMesh(subChild, wowView, wowModel.GlobalVertexList);
                    }
                }
                */
                jointIter.next();
            }

            var meshIter = new MItDependencyNodes(MFn.Type.kMesh);
            while (!meshIter.isDone)
            {
                ExtractMesh(meshIter.thisNode, wowView, wowModel.GlobalVertexList);
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
            if(wowModel.Bones.Count == 0) wowModel.Bones.Add(new M2Bone());//May become useless since there is always a transform at top in Maya.

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
            wowTexture.Name = filename;
            if(wrapU) wowTexture.Flags |= M2Texture.TextureFlags.WrapX;
            if(wrapV) wowTexture.Flags |= M2Texture.TextureFlags.WrapY;
            wowTexture.Type = M2Texture.TextureType.Skin;
            //TODO Type, identification with name ?
            wowTextures.Add(wowTexture);
        }

        private static void ExtractMesh(MObject meshNode, M2SkinProfile wowView, ICollection<M2Vertex> globalVertices)
        {
            var mesh = new MFnMesh(meshNode);// Always ok when iterating on meshes only
            var wowMesh = new M2SkinSection();
            //nStartVertex + MayaIndex = ViewIndex

            // VERTICES
            var vertexPositions = new MFloatPointArray();// Each vertex will be refered as its index into these lists.
            var vertexNormals = new MFloatVectorArray();
            mesh.getPoints(vertexPositions);
            mesh.getVertexNormals(false, vertexNormals);
            //TODO Generate CenterMass, CenterBoundingBox, Radius

            wowMesh.StartVertex = (ushort)wowView.Indices.Count;//TODO Check level for big models, like character models with lots of triangles
            var vertexIter = new MItMeshVertex(meshNode);
            while (!vertexIter.isDone)
            {
                var mayaVertexPosition = vertexPositions[vertexIter.index()];
                var mayaVertexNormal = vertexNormals[vertexIter.index()];
                var mayaVertexUv = new float[2];
                vertexIter.getUV(mayaVertexUv);
                //TODO multiple textures per mesh. Maybe with getUVSets which gives multiple arrays of u and v you can index with vertex id ?

                var wowVertex = new M2Vertex();
                //Here I fix the axis
                wowVertex.Position = new C3Vector(mayaVertexPosition.x, mayaVertexPosition.z, mayaVertexPosition.y);
                wowVertex.Normal = new C3Vector(mayaVertexNormal.x, mayaVertexNormal.z, mayaVertexNormal.y);
                wowVertex.TexCoords[0] = new C2Vector(mayaVertexUv[0], mayaVertexUv[1]);
                //TODO things in vertex : boneWeights, boneIndices. Maybe in maya skinClusters ?

                wowMesh.NVertices++;
                wowView.Indices.Add((ushort)globalVertices.Count);//All of this to do the wow Vertex mapping mesh->view->global
                globalVertices.Add(wowVertex);

                vertexIter.next();
            }

            // POLYGONS
            wowMesh.StartTriangle = (ushort)wowView.Triangles.Count;//TODO Check level for big models

            var polygonIter = new MItMeshPolygon(meshNode);
            while (!polygonIter.isDone)
            {
                var points = new MPointArray();// Unused
                var vertexList = new MIntArray();
                polygonIter.getTriangles(points, vertexList);
                foreach (var index in vertexList)
                {
                    wowView.Triangles.Add((ushort)(index + wowMesh.StartVertex));//Added StartVertex so the index become View relative and no Mesh relative. TODO level here too ?
                    wowMesh.NTriangles++;
                }

                polygonIter.next();
            }

            wowView.Submeshes.Add(wowMesh);
        }
    }
}