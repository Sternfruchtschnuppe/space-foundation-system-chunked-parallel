using System;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace SpaceFoundationSystem 
{
    [BurstCompile]
    public class Chunk : IDisposable
    {
        public int size;
        public int3 chunkPosition;
        public NativeBitArray delimArray;
        
        public NativeArray<ChunkVoxelData> voxelArray;
        public NativeHashMap<int, VoxelAnchor> chunkAnchors;
        public NativeHashMap<int, VoxelAnchor> relativeAnchors;
        
        public NativeHashMap<int, ChunkVoxelData> lftBorderHM;
        public NativeHashMap<int, ChunkVoxelData> rgtBorderHM;
        public NativeHashMap<int, ChunkVoxelData> botBorderHM;
        public NativeHashMap<int, ChunkVoxelData> topBorderHM;
        public NativeHashMap<int, ChunkVoxelData> bckBorderHM;
        public NativeHashMap<int, ChunkVoxelData> fwdBorderHM;
        
        public NativeList<OffIdxVoxelData> possiblyCutoffVoxels;
        public NativeHashMap<int, ChunkVoxelData> lftCutoffBorderHM;
        public NativeHashMap<int, ChunkVoxelData> rgtCutoffBorderHM;
        public NativeHashMap<int, ChunkVoxelData> botCutoffBorderHM;
        public NativeHashMap<int, ChunkVoxelData> topCutoffBorderHM;
        public NativeHashMap<int, ChunkVoxelData> bckCutoffBorderHM;
        public NativeHashMap<int, ChunkVoxelData> fwdCutoffBorderHM;
        
        public NativeHashSet<int2> implicitDelimitersHS;
        public NativeHashSet<int2> explicitDelimitersHS;
        public NativeHashMap<int, ushort> anchorBorderHM;
        public NativeHashMap<ushort, int> anchorIdCntHM;
        public NativeParallelMultiHashMap<int, int> delimMHM;
        
        public Chunk(int3 chunkPos, LayerMask delimitingLayerMask, List<VoxelAnchor> chunkAnchors, VoxelAnchor[] allVoxelAnchors, Dictionary<Collider, int> colliderDelimDict, int chunkSize, float voxelSize)
        {
            this.size = chunkSize; 
            this.chunkPosition = chunkPos;
            var chunkWorldPos = SpaceFoundation.ChunkToWorld(chunkPos, voxelSize);

            delimArray = new NativeBitArray(size * size * size, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            
            this.chunkAnchors = new NativeHashMap<int, VoxelAnchor>(chunkAnchors.Count * 2, Allocator.Persistent);
            foreach (var a in chunkAnchors) this.chunkAnchors.Add(a.id, a);
            delimMHM = new NativeParallelMultiHashMap<int, int>(size * size * size, Allocator.Persistent);
            
            //fill delimArray
            var buffer = new Collider[8];
            var chunkOffset = SpaceFoundation.Float3ToVec(chunkWorldPos);
            var halfExtents = SpaceFoundation.Float3ToVec(new float3(voxelSize * 0.5f));
            var idx = 0;
            for (var z = 0; z < size; z++) 
                for (var y = 0; y < size; y++) 
                    for (var x = 0; x < size; x++)
                    {
                        var center = chunkOffset + new Vector3(x, y, z) * voxelSize;
                        var cnt = Physics.OverlapBoxNonAlloc(center, halfExtents, buffer, Quaternion.identity, delimitingLayerMask);
                        delimArray.Set(idx, cnt > 0);
                        var zeroAdded = false;
                        for (var i = 0; i < cnt; ++i)
                        {
                            // id 0 means collider matches mask, but has no Delimiter Component
                            var id = colliderDelimDict.GetValueOrDefault(buffer[i], 0);
                            if (id == 0 && zeroAdded) continue;
                            delimMHM.Add(idx, id);
                            zeroAdded = zeroAdded || id == 0;
                        }
                        ++idx;
                    }
            
            //compute all anchors relative to this chunk
            relativeAnchors = new NativeHashMap<int, VoxelAnchor>(allVoxelAnchors.Length * 2, Allocator.Persistent);
            for (var i = 0; i < allVoxelAnchors.Length; i++) relativeAnchors[allVoxelAnchors[i].id] = new VoxelAnchor(allVoxelAnchors[i], chunkPosition, size);
            
            voxelArray = new NativeArray<ChunkVoxelData>(delimArray.Length, Allocator.Persistent);
            
            lftBorderHM = new NativeHashMap<int, ChunkVoxelData>(size * size * 2, Allocator.Persistent);
            rgtBorderHM = new NativeHashMap<int, ChunkVoxelData>(size * size * 2, Allocator.Persistent);
            botBorderHM = new NativeHashMap<int, ChunkVoxelData>(size * size * 2, Allocator.Persistent);
            topBorderHM = new NativeHashMap<int, ChunkVoxelData>(size * size * 2, Allocator.Persistent);
            bckBorderHM = new NativeHashMap<int, ChunkVoxelData>(size * size * 2, Allocator.Persistent);
            fwdBorderHM = new NativeHashMap<int, ChunkVoxelData>(size * size * 2, Allocator.Persistent);
            possiblyCutoffVoxels = new NativeList<OffIdxVoxelData>(allVoxelAnchors.Length, Allocator.Persistent);
            lftCutoffBorderHM = new NativeHashMap<int, ChunkVoxelData>(size * size * 2, Allocator.Persistent);
            rgtCutoffBorderHM = new NativeHashMap<int, ChunkVoxelData>(size * size * 2, Allocator.Persistent);
            botCutoffBorderHM = new NativeHashMap<int, ChunkVoxelData>(size * size * 2, Allocator.Persistent);
            topCutoffBorderHM = new NativeHashMap<int, ChunkVoxelData>(size * size * 2, Allocator.Persistent);
            bckCutoffBorderHM = new NativeHashMap<int, ChunkVoxelData>(size * size * 2, Allocator.Persistent);
            fwdCutoffBorderHM = new NativeHashMap<int, ChunkVoxelData>(size * size * 2, Allocator.Persistent);
            
            implicitDelimitersHS = new NativeHashSet<int2>(size * size, Allocator.Persistent);
            explicitDelimitersHS = new NativeHashSet<int2>(size * size, Allocator.Persistent);
            anchorBorderHM = new NativeHashMap<int, ushort>(size * size, Allocator.Persistent);
            anchorIdCntHM = new NativeHashMap<ushort, int>(allVoxelAnchors.Length * 2, Allocator.Persistent);
        }
        
        public void Dispose()
        {
            if (delimArray.IsCreated) delimArray.Dispose();
            if (voxelArray.IsCreated) voxelArray.Dispose();
            if (chunkAnchors.IsCreated) chunkAnchors.Dispose();
            if (relativeAnchors.IsCreated) relativeAnchors.Dispose();
            if (lftBorderHM.IsCreated) lftBorderHM.Dispose();
            if (rgtBorderHM.IsCreated) rgtBorderHM.Dispose();
            if (botBorderHM.IsCreated) botBorderHM.Dispose();
            if (topBorderHM.IsCreated) topBorderHM.Dispose();
            if (bckBorderHM.IsCreated) bckBorderHM.Dispose();
            if (fwdBorderHM.IsCreated) fwdBorderHM.Dispose();
            if (possiblyCutoffVoxels.IsCreated) possiblyCutoffVoxels.Dispose();
            if (lftCutoffBorderHM.IsCreated) lftCutoffBorderHM.Dispose();
            if (rgtCutoffBorderHM.IsCreated) rgtCutoffBorderHM.Dispose();
            if (botCutoffBorderHM.IsCreated) botCutoffBorderHM.Dispose();
            if (topCutoffBorderHM.IsCreated) topCutoffBorderHM.Dispose();
            if (bckCutoffBorderHM.IsCreated) bckCutoffBorderHM.Dispose();
            if (fwdCutoffBorderHM.IsCreated) fwdCutoffBorderHM.Dispose();
            if (implicitDelimitersHS.IsCreated) implicitDelimitersHS.Dispose();
            if (explicitDelimitersHS.IsCreated) explicitDelimitersHS.Dispose();
            if (anchorBorderHM.IsCreated) anchorBorderHM.Dispose(); 
            if (anchorIdCntHM.IsCreated) anchorIdCntHM.Dispose();
            if (delimMHM.IsCreated) delimMHM.Dispose();
        }
    }
}