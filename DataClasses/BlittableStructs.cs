using System.Collections.Generic;
using System.Runtime.InteropServices;
using Unity.Burst;
using Unity.Mathematics;

namespace SpaceFoundationSystem
{
    [StructLayout(LayoutKind.Sequential)]
    [BurstCompile]
    public struct IdxVoxelData
    {
        public int idx;
        public ChunkVoxelData voxel;
    }

    [StructLayout(LayoutKind.Sequential)]
    [BurstCompile]
    public struct OffIdxVoxelData
    {
        public int3 off;
        public int idx;
        public ChunkVoxelData voxel;
    }
    
    [StructLayout(LayoutKind.Sequential)]
    [BurstCompile]
    public struct DirOffData
    {
        public int3 dir;
        public int off;
    } 
    
    [BurstCompile]
    public struct VoxelDistComparator : IComparer<IdxVoxelData>
    {
        public int Compare(IdxVoxelData a, IdxVoxelData b)
        {
            return ChunkVoxelData.Compare(a.voxel, b.voxel);
        }
    }
}