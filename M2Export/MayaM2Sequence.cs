using M2Lib.m2;

namespace M2Export
{
    public class MayaM2Sequence
    {
        public int Start { get; set; }
        public int End { get; set; }
        public int Type { get; set; }
        public bool IsLooping { get; set; } = true;

        public M2Sequence ToWoW()
        {
            return null;
        }
    }
}
