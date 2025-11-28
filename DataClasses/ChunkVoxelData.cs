using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using Unity.Burst;

namespace SpaceFoundationSystem
{
    [BurstCompile]
    public struct ChunkVoxelData
    {    
        public ushort anchorId;
        public ushort distance;
        
        public static int Compare(ChunkVoxelData x, ChunkVoxelData y)
        {
            var distCompare = x.distance.CompareTo(y.distance);
            if (distCompare != 0) return distCompare;
            return x.anchorId.CompareTo(y.anchorId);
        }

        public static bool operator <(ChunkVoxelData left, ChunkVoxelData right)
            => Compare(left, right) < 0;

        public static bool operator >(ChunkVoxelData left, ChunkVoxelData right)
            => Compare(left, right) > 0;

        public static bool operator <=(ChunkVoxelData left, ChunkVoxelData right)
            => Compare(left, right) <= 0;

        public static bool operator >=(ChunkVoxelData left, ChunkVoxelData right)
            => Compare(left, right) >= 0;
    }
    
}