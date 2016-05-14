using System.IO;
using Autodesk.Maya.OpenMaya;
using M2ExportLK;
using M2Lib.m2;
using MayaM2;

[assembly: MPxFileTranslatorClass(typeof(M2Translator), "WoW M2 (Wrath of the Lich King)", null, null, null)]

namespace M2ExportLK
{
    public class M2Translator : MPxFileTranslator
    {
        protected const M2.Format WoWVersion = M2.Format.LichKing;
        public override string defaultExtension() => "m2";
        public override bool haveReadMethod() => false;
        public override bool haveWriteMethod() => true;
        public override bool canBeOpened() => true;

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
            return CommonM2Utils.IsRightVersion(file, WoWVersion);
        }

        /// <summary>
        /// Maya calls this method to have the translator write out a file.
        /// </summary>
        /// <param name="file"></param>
        /// <param name="optionsString"></param>
        /// <param name="mode"></param>
        public override void writer(MFileObject file, string optionsString, FileAccessMode mode)
        {
            MGlobal.displayInfo("Exporting to M2 "+WoWVersion+"..");

            var wowModel = new M2 {Name = file.rawName.Substring(0, file.rawName.Length - 3)};
            // Name is fileName without .m2 extension

            MayaToM2.ExtractModel(wowModel);

            using (var writer = new BinaryWriter(new FileStream(file.expandedFullName, FileMode.Create, FileAccess.Write)))
            {
                wowModel.Save(writer, WoWVersion);
            }
            MGlobal.displayInfo("Done.");
        }
    }
}