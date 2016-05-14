using Autodesk.Maya.OpenMaya;
using M2Lib.m2;

namespace MayaM2
{
    public static class CommonM2Utils
    {
        private const string FExtension = "m2";
        public static MPxFileTranslator.MFileKind IsRightVersion(MFileObject file, M2.Format version)
        {
            //Based on extension
            //TODO Check magic number if bufferLen >= 4 and the first characters are M,D,2,0
            //TODO Check version
            var fileName = file.name;
            var fileNameLen = fileName.Length;
            var startOfExtension = fileName.IndexOf('.') + 1;

            if ((startOfExtension > 0)
            && (startOfExtension < fileNameLen)
            && (fileName.Substring(startOfExtension, fileNameLen) == FExtension))
            {
                return MPxFileTranslator.MFileKind.kIsMyFileType;
            }
            return MPxFileTranslator.MFileKind.kNotMyFileType;
        }

    }
}