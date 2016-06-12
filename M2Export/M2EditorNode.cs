using Autodesk.Maya.OpenMaya;
using M2Export;

[assembly: MPxNodeClass(typeof(M2EditorNode), "m2Editor", 0x00000FFF)]

namespace M2Export
{
    public class M2EditorNode : MPxNode
    {
        // Animation clips
        //TODO type enum
        public static MObject AnimClipName;
        public static MObject AnimClipStart;
        public static MObject AnimClipEnd;
        public static MObject LoopingFlag;
        public static MObject LowPriorityFlag;
        public static MObject Repetitions;
        public static MObject BlendingFlag;
        public static MObject BlendTime;
        public static MObject Probability;
        public static MObject ExportAnimClip;

        public static MObject AnimClips;

        public static void InitializeAnimClips()
        {
            var tAttr = new MFnTypedAttribute();
            var nAttr = new MFnNumericAttribute();
            var cAttr = new MFnCompoundAttribute();

            AnimClipName = tAttr.create("animClipName", "acn", MFnData.Type.kString);
            addAttribute(AnimClipName);
            AnimClipStart = nAttr.create("animClipStart", "acs", MFnNumericData.Type.kInt);
                nAttr.setMin(0);
            addAttribute(AnimClipStart);
            AnimClipEnd = nAttr.create("animClipEnd", "ace", MFnNumericData.Type.kInt);
                nAttr.setMin(0);
            addAttribute(AnimClipEnd);
            LoopingFlag = nAttr.create("animClipLooping", "acl", MFnNumericData.Type.kBoolean);
                nAttr.setDefault(true);
            addAttribute(LoopingFlag);
            LowPriorityFlag = nAttr.create("animClipLowPriority", "aclp", MFnNumericData.Type.kBoolean);
                nAttr.setDefault(true);
            addAttribute(LowPriorityFlag);
            Repetitions = nAttr.create("animClipRep", "acRep", MFnNumericData.Type.k2Float);
                nAttr.setDefault((float) 0, 0);
                nAttr.setMin(0, 0);
            addAttribute(Repetitions);
            BlendingFlag = nAttr.create("animClipBlending", "acb", MFnNumericData.Type.kBoolean);
                nAttr.setDefault(true);
            addAttribute(BlendingFlag);
            BlendTime = nAttr.create("animClipBlendTime", "acbt", MFnNumericData.Type.k2Int);
                nAttr.setDefault(150, 150);
                nAttr.setMin(0, 0);
            addAttribute(BlendTime);
            Probability = nAttr.create("animClipProbability", "acp", MFnNumericData.Type.kFloat);
                nAttr.setDefault((float) 100);
                nAttr.setMin(0);
                nAttr.setMax(100);
            addAttribute(Probability);
            ExportAnimClip = nAttr.create("exportAnimClip", "eac", MFnNumericData.Type.kBoolean);
                nAttr.setDefault(true);
            addAttribute(ExportAnimClip);

            /*
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
            */

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
        }

        [MPxNodeInitializer]
        public static bool Initialize()
        {
            InitializeAnimClips();
            return true;
        }
    }
}