using Autodesk.Maya.OpenMaya;
using M2Export;

[assembly: MPxNodeClass(typeof(M2EditorNode), "m2Editor", 0x00000FFF)]

namespace M2Export
{
    public class M2EditorNode : MPxNode
    {
        // Animation clips
        public static MObject AnimClipName;
        public static MObject AnimClipStart;
        public static MObject AnimClipEnd;
        public static MObject ExportAnimClip;

        public static MObject AnimClips;

        [MPxNodeInitializer]
        public static bool Initialize()
        {
            var tAttr = new MFnTypedAttribute();
            var nAttr = new MFnNumericAttribute();
            var cAttr = new MFnCompoundAttribute();

            // Animation clips
            AnimClipName = tAttr.create("animClipName", "acn", MFnData.Type.kString);
            AnimClipStart = nAttr.create("animClipStart", "acs", MFnNumericData.Type.kInt);
            AnimClipEnd = nAttr.create("animClipEnd", "ace", MFnNumericData.Type.kInt);
            ExportAnimClip = nAttr.create("exportAnimClip", "eac", MFnNumericData.Type.kBoolean, 1);
            addAttribute(AnimClipName);
            addAttribute(AnimClipStart);
            addAttribute(AnimClipEnd);
            addAttribute(ExportAnimClip);

            AnimClips = cAttr.create("animClips", "clips");
            cAttr.addChild(AnimClipName);
            cAttr.addChild(AnimClipStart);
            cAttr.addChild(AnimClipEnd);
            cAttr.addChild(ExportAnimClip);
            cAttr.isArray = true;
            addAttribute(AnimClips);

            return true;
        }
    }
}