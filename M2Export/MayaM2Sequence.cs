using System.Collections.Generic;
using System.Linq;
using M2Lib.m2;

namespace M2Export
{
    public class MayaM2Sequence
    {
        public int Start { get; set; }
        public int End { get; set; }
        public int Type { get; set; }
        public bool IsLoop { get; set; } = true;
        public bool IsLowPriority { get; set; } = false;
        public int MinimumRepetitions { get; set; }
        public int MaximumRepetitions { get; set; }
        public bool IsBlending { get; set; } = false;
        public int BlendTimeStart { get; set; } = 150;
        public int BlendTimeEnd { get; set; } = 150;
        public short Probability { get; set; } = short.MaxValue;

        public M2Sequence ToWoW()
        {
            var seq = new M2Sequence
            {
                AnimationId = (ushort) Type,
                Length = (uint) (End - Start),
                MinimumRepetitions = (uint) MinimumRepetitions,
                MaximumRepetitions = (uint) MaximumRepetitions,
                BlendTimeStart = (ushort) BlendTimeStart,
                BlendTimeEnd = (ushort) BlendTimeEnd,
                Probability = Probability
            };
            if(IsLoop) seq.Flags |= M2Sequence.SequenceFlags.Looped;
            if (IsLowPriority)
            {
                seq.Flags &= ~M2Sequence.SequenceFlags.Priority;
                seq.Flags &= ~M2Sequence.SequenceFlags.Stored;
            }
            if(IsBlending) seq.Flags |= M2Sequence.SequenceFlags.Blended;
            return seq;
        }

        public static void NormalizeProbability(List<MayaM2Sequence> list)
        {
            var sum = list.Sum(s => s.Probability);
            var remaining = short.MaxValue;
            foreach (var seq in list)
            {
                var fixedProba = (short) (seq.Probability/(float) sum * short.MaxValue);
                seq.Probability = fixedProba > remaining ? remaining : fixedProba;
                remaining -= seq.Probability;
            }
            if (remaining > 0)
            {
                list.Last().Probability += remaining;
            }
        }
    }
}
