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

        //TODO type enum
        public static MObject LoopingFlag;
        public static MObject LowPriorityFlag;
        public static MObject Repetitions;
        public static MObject BlendingFlag;
        public static MObject BlendTime;
        public static MObject Probability;
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
            nAttr.setMin(0);
            AnimClipEnd = nAttr.create("animClipEnd", "ace", MFnNumericData.Type.kInt);
            nAttr.setMin(0);
            LoopingFlag = nAttr.create("animClipLooping", "acl", MFnNumericData.Type.kBoolean);
            nAttr.setDefault(true);
            LowPriorityFlag = nAttr.create("animClipLowPriority", "aclp", MFnNumericData.Type.kBoolean);
            nAttr.setDefault(true);
            Repetitions = nAttr.create("animClipRep", "acRep", MFnNumericData.Type.k2Float);
            nAttr.setDefault((float) 0, 0);
            nAttr.setMin(0, 0);
            BlendingFlag = nAttr.create("animClipBlending", "acb", MFnNumericData.Type.kBoolean);
            nAttr.setDefault(true);
            BlendTime = nAttr.create("animClipBlendTime", "acbt", MFnNumericData.Type.k2Int);
            nAttr.setDefault(150, 150);
            nAttr.setMin(0, 0);
            Probability = nAttr.create("animClipProbability", "acp", MFnNumericData.Type.kFloat);
            nAttr.setDefault((float) 100);
            nAttr.setMin(0);
            nAttr.setMax(100);
            ExportAnimClip = nAttr.create("exportAnimClip", "eac", MFnNumericData.Type.kBoolean);
            nAttr.setDefault(true);

            addAttribute(AnimClipName);
            addAttribute(AnimClipStart);
            addAttribute(AnimClipEnd);
            addAttribute(LoopingFlag);
            addAttribute(LowPriorityFlag);
            addAttribute(Repetitions);
            addAttribute(BlendingFlag);
            addAttribute(BlendTime);
            addAttribute(Probability);
            addAttribute(ExportAnimClip);

            AnimClips = cAttr.create("animClips", "clips");
            cAttr.addChild(AnimClipName);
            cAttr.addChild(AnimClipStart);
            cAttr.addChild(AnimClipEnd);
            cAttr.addChild(LoopingFlag);
            cAttr.addChild(LowPriorityFlag);
            cAttr.addChild(Repetitions);
            cAttr.addChild(BlendingFlag);
            cAttr.addChild(BlendTime);
            cAttr.addChild(Probability);
            cAttr.addChild(ExportAnimClip);
            cAttr.isArray = true;

            addAttribute(AnimClips);

            return true;
        }
    }
}