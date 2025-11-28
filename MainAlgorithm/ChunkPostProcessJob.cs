using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEditor.Graphs;
using UnityEditor.TerrainTools;
using UnityEngine;

namespace SpaceFoundationSystem
{
    [BurstCompile]
    public struct ChunkPostProcessJob : IJob
    {
        [ReadOnly] public NativeArray<ChunkVoxelData> voxelArray;
        [ReadOnly] public NativeArray<ChunkVoxelData> lftVoxelArray;
        [ReadOnly] public NativeArray<ChunkVoxelData> rgtVoxelArray;
        [ReadOnly] public NativeArray<ChunkVoxelData> botVoxelArray;
        [ReadOnly] public NativeArray<ChunkVoxelData> topVoxelArray;
        [ReadOnly] public NativeArray<ChunkVoxelData> bckVoxelArray;
        [ReadOnly] public NativeArray<ChunkVoxelData> fwdVoxelArray;

        [ReadOnly] public NativeParallelMultiHashMap<int, int> delimMHM;
        [ReadOnly] public NativeParallelMultiHashMap<int, int> lftDelimMHM;
        [ReadOnly] public NativeParallelMultiHashMap<int, int> rgtDelimMHM;
        [ReadOnly] public NativeParallelMultiHashMap<int, int> botDelimMHM;
        [ReadOnly] public NativeParallelMultiHashMap<int, int> topDelimMHM;
        [ReadOnly] public NativeParallelMultiHashMap<int, int> bckDelimMHM;
        [ReadOnly] public NativeParallelMultiHashMap<int, int> fwdDelimMHM;

        [ReadOnly] public int size;

        public NativeHashSet<int2> implicitDelimitersHS;
        public NativeHashSet<int2> explicitDelimitersHS;
        public NativeHashMap<int, ushort> anchorBorderHM;

        public NativeHashMap<ushort, int> anchorIdCntHM;
        
        private static readonly int3[] dirs = new []{ new int3(1, 0, 0), new int3(0, 1, 0), new int3(0, 0, 1) };

        
        public void Execute()
        {
            for (var z = 0; z < size; ++z)
                for (var y = 0; y < size; ++y)
                    for (var x = 0; x < size; ++x)
                    {
                        var idx = SpaceFoundation.Idx(x, y, z, size);
                        var voxel = voxelArray[idx];

                        if (voxel.anchorId == 6)
                        {
                            var xxx = 0;
                        }
                        
                        anchorIdCntHM[voxel.anchorId] = anchorIdCntHM.TryGetValue(voxel.anchorId, out var count) ? count + 1 : 1;
                        foreach (var dir in dirs)
                        {
                            if ((x == size - 1 && dir.x == 1) || 
                                (y == size - 1 && dir.y == 1) ||
                                (z == size - 1 && dir.z == 1)) continue;
                            
                            var neighborIdx = SpaceFoundation.Idx(x + dir.x, y + dir.y, z + dir.z, size);
                            var neighborVoxel = voxelArray[neighborIdx];
                            if (voxel.anchorId == neighborVoxel.anchorId) continue;
                            anchorBorderHM[idx] = voxelArray[idx].anchorId;
                            anchorBorderHM[neighborIdx] = voxelArray[neighborIdx].anchorId;
                            CollectDelimiters(voxel.anchorId, idx, delimMHM, neighborVoxel.anchorId, neighborIdx, delimMHM);
                        }
                    }
            
            //iterate over all chunkBorders and add them to the anchorBorderDict
            for (var y = 0; y < size; ++y)
                for (var x = 0; x < size; ++x)
                {
                    var lft = SpaceFoundation.Idx(0, x, y, size);
                    var rgt = SpaceFoundation.Idx(size - 1, x, y, size);
                    var bot = SpaceFoundation.Idx(x, 0, y, size);
                    var top = SpaceFoundation.Idx(x, size - 1, y, size);
                    var bck = SpaceFoundation.Idx(x, y, 0, size);
                    var fwd = SpaceFoundation.Idx(x, y, size - 1, size);
                    
                    var lftAId = voxelArray[lft].anchorId;
                    var rgtAId = voxelArray[rgt].anchorId;
                    var botAId = voxelArray[bot].anchorId;
                    var topAId = voxelArray[top].anchorId;
                    var bckAId = voxelArray[bck].anchorId;
                    var fwdAId = voxelArray[fwd].anchorId;
                    
                    anchorBorderHM[lft] = lftAId;
                    anchorBorderHM[rgt] = rgtAId;
                    anchorBorderHM[bot] = botAId;
                    anchorBorderHM[top] = topAId;
                    anchorBorderHM[bck] = bckAId;
                    anchorBorderHM[fwd] = fwdAId;
                    anchorIdCntHM[lftAId] = anchorIdCntHM.TryGetValue(lftAId, out var count) ? count + 1 : 1;
                    anchorIdCntHM[rgtAId] = anchorIdCntHM.TryGetValue(rgtAId, out count) ? count + 1 : 1;
                    anchorIdCntHM[botAId] = anchorIdCntHM.TryGetValue(botAId, out count) ? count + 1 : 1;
                    anchorIdCntHM[topAId] = anchorIdCntHM.TryGetValue(topAId, out count) ? count + 1 : 1;
                    anchorIdCntHM[bckAId] = anchorIdCntHM.TryGetValue(bckAId, out count) ? count + 1 : 1;
                    anchorIdCntHM[fwdAId] = anchorIdCntHM.TryGetValue(fwdAId, out count) ? count + 1 : 1;
                    CollectDelimiters(lftAId, lft, delimMHM, lftVoxelArray[rgt].anchorId, rgt, lftDelimMHM);
                    CollectDelimiters(rgtAId, rgt, delimMHM, rgtVoxelArray[lft].anchorId, lft, rgtDelimMHM);
                    CollectDelimiters(botAId, bot, delimMHM, botVoxelArray[top].anchorId, top, botDelimMHM);
                    CollectDelimiters(topAId, top, delimMHM, topVoxelArray[bot].anchorId, bot, topDelimMHM);
                    CollectDelimiters(bckAId, bck, delimMHM, bckVoxelArray[fwd].anchorId, fwd, bckDelimMHM);
                    CollectDelimiters(fwdAId, fwd, delimMHM, fwdVoxelArray[bck].anchorId, bck, fwdDelimMHM);
                }
            
            anchorIdCntHM.Remove(0); 
        }

        private void CollectDelimiters(int aId1, int delimIdx1, NativeParallelMultiHashMap<int, int> delimMHM1, int aId2, int delimIdx2, NativeParallelMultiHashMap<int, int> delimMHM2)
        {
            if (aId1 == aId2) return;
            
            var delim1Exists = delimMHM1.TryGetFirstValue(delimIdx1, out var dId1, out var dIt1);
            var delim2Exists = delimMHM2.TryGetFirstValue(delimIdx2, out var dId2, out var dIt2);
            var a1Exists = aId1 != 0;
            var a2Exists = aId2 != 0;

            if (delim1Exists && !delim2Exists && a2Exists)
            {
                do
                {
                    if (dId1 != 0) explicitDelimitersHS.Add(new int2(dId1, aId2));
                } while (delimMHM1.TryGetNextValue(out dId1, ref dIt1));
            }
            else if (delim2Exists && !delim1Exists && a1Exists)
            {
                do
                {
                    if (dId2 != 0) explicitDelimitersHS.Add(new int2(dId2, aId1));
                } while (delimMHM2.TryGetNextValue(out dId2, ref dIt2));
            }
            else if (!delim1Exists && !delim2Exists && a1Exists && a2Exists)
            {
                implicitDelimitersHS.Add(new int2(aId1, aId2));
            }
        }
    }
}