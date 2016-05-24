using Autodesk.Maya.OpenMaya;

// This line is not mandatory, but improves loading performances
[assembly: ExtensionPlugin(typeof(WoWTools.AddInterface))]

namespace WoWTools
{

    // This class is instantiated by Maya once and kept alive for the 
    // duration of the session. If you don't do any one time initialization 
    // then you should remove this class.
    // Its presence still improve load performance whilst you don't do any
    // initialization in it.
    public class AddInterface : IExtensionPlugin
    {
        public bool InitializePlugin()
        {
            // Initialize your plug-in application here
            MGlobal.executeCommand("InitWoWGUI");
            return true;
        }

        public bool UninitializePlugin()
        {
            // Do plug-in application clean up here
            MGlobal.executeCommand("DelWoWGUI");
            return true;
        }

        // Return the Maya API version number
        // The actual plug-in can return an empty string by default
        public string GetMayaDotNetSdkBuildVersion() => "";
    }
}
