using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace SpaceFoundationSystem
{
    [BurstCompile]
    public struct VoxelFloodFillJob : IJob
    {
        [ReadOnly] public NativeBitArray delimArray;
        
        public NativeArray<ChunkVoxelData> voxelArray;
        [ReadOnly] public NativeArray<ChunkVoxelData> lftVoxelArray;
        [ReadOnly] public NativeArray<ChunkVoxelData> rgtVoxelArray;
        [ReadOnly] public NativeArray<ChunkVoxelData> botVoxelArray;
        [ReadOnly] public NativeArray<ChunkVoxelData> topVoxelArray;
        [ReadOnly] public NativeArray<ChunkVoxelData> bckVoxelArray;
        [ReadOnly] public NativeArray<ChunkVoxelData> fwdVoxelArray;
        
        public NativeHashMap<int, VoxelAnchor> chunkAnchors;
        [ReadOnly] public NativeHashMap<int, VoxelAnchor> relativeAnchors;
        
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
        public int size;
        
        public void Execute()
        {
            var size = this.size; 
            var frontierQueue1 = new NativeQueue<IdxVoxelData>(Allocator.Temp);
            var frontierQueue2 = new NativeQueue<IdxVoxelData>(Allocator.Temp);
            var visitedArray = new NativeBitArray(voxelArray.Length, Allocator.Temp);

            var initialVoxels = new NativeArray<IdxVoxelData>(
                lftBorderHM.Count + rgtBorderHM.Count + botBorderHM.Count + topBorderHM.Count + bckBorderHM.Count +
                fwdBorderHM.Count + chunkAnchors.Count, Allocator.Temp);
            var borderVoxels = new NativeList<IdxVoxelData>(initialVoxels.Length, Allocator.Temp);
            var cutoffRemovedHS = new NativeHashSet<int>(voxelArray.Length * 2, Allocator.Temp);
            var cutoffVoxels = new NativeArray<IdxVoxelData>(lftCutoffBorderHM.Count + rgtCutoffBorderHM.Count + botCutoffBorderHM.Count + topCutoffBorderHM.Count + bckCutoffBorderHM.Count + fwdCutoffBorderHM.Count, Allocator.Temp);
            
            var i = 0;
            foreach (var tuple in lftCutoffBorderHM) cutoffVoxels[i++] = new IdxVoxelData{ idx = tuple.Key, voxel = tuple.Value };
            foreach (var tuple in rgtCutoffBorderHM) cutoffVoxels[i++] = new IdxVoxelData{ idx = tuple.Key, voxel = tuple.Value };
            foreach (var tuple in botCutoffBorderHM) cutoffVoxels[i++] = new IdxVoxelData{ idx = tuple.Key, voxel = tuple.Value };
            foreach (var tuple in topCutoffBorderHM) cutoffVoxels[i++] = new IdxVoxelData{ idx = tuple.Key, voxel = tuple.Value };
            foreach (var tuple in bckCutoffBorderHM) cutoffVoxels[i++] = new IdxVoxelData{ idx = tuple.Key, voxel = tuple.Value };
            foreach (var tuple in fwdCutoffBorderHM) cutoffVoxels[i++] = new IdxVoxelData{ idx = tuple.Key, voxel = tuple.Value };
            
            var lftBorder = new NativeList<IdxVoxelData>(size * size * 2, Allocator.Temp);
            var rgtBorder = new NativeList<IdxVoxelData>(size * size * 2, Allocator.Temp);
            var botBorder = new NativeList<IdxVoxelData>(size * size * 2, Allocator.Temp);
            var topBorder = new NativeList<IdxVoxelData>(size * size * 2, Allocator.Temp);
            var bckBorder = new NativeList<IdxVoxelData>(size * size * 2, Allocator.Temp);
            var fwdBorder = new NativeList<IdxVoxelData>(size * size * 2, Allocator.Temp);
            
            // Deal with remaining cutoff voxels
            cutoffVoxels.Sort(new VoxelDistComparator());
            foreach (var iv in cutoffVoxels)
            {
                RemoveCutoffVoxels(iv.idx, iv.voxel, frontierQueue1, frontierQueue2, cutoffRemovedHS, borderVoxels,
                    visitedArray, lftBorder, rgtBorder, botBorder, topBorder, bckBorder, fwdBorder);
            }

            // Flood fill
            if (cutoffVoxels.Length == 0)
            {
                i = 0;
                foreach (var tuple in lftBorderHM) initialVoxels[i++] = new IdxVoxelData { idx = tuple.Key, voxel = tuple.Value };
                foreach (var tuple in rgtBorderHM) initialVoxels[i++] = new IdxVoxelData { idx = tuple.Key, voxel = tuple.Value };
                foreach (var tuple in botBorderHM) initialVoxels[i++] = new IdxVoxelData { idx = tuple.Key, voxel = tuple.Value };
                foreach (var tuple in topBorderHM) initialVoxels[i++] = new IdxVoxelData { idx = tuple.Key, voxel = tuple.Value };
                foreach (var tuple in bckBorderHM) initialVoxels[i++] = new IdxVoxelData { idx = tuple.Key, voxel = tuple.Value };
                foreach (var tuple in fwdBorderHM) initialVoxels[i++] = new IdxVoxelData { idx = tuple.Key, voxel = tuple.Value };
                
                foreach (var a in chunkAnchors)
                {
                    var idx = SpaceFoundation.Idx(a.Value.localVoxelPosition, size);
                    initialVoxels[i++] = new IdxVoxelData { idx = idx, voxel = new ChunkVoxelData { anchorId = (ushort)a.Key, distance = 0 }};
                }
                
                StartExploration(frontierQueue1, frontierQueue2, initialVoxels, visitedArray, lftBorder, rgtBorder, botBorder, topBorder, bckBorder, fwdBorder);
                
                // Remove potentially created cutoff voxels
                possiblyCutoffVoxels.Clear();
                var dirs = new NativeArray<DirOffData>(6, Allocator.Temp);
                dirs[0] = new DirOffData { dir = new int3(-1, 0, 0), off = -1 };
                dirs[1] = new DirOffData { dir = new int3(1, 0, 0), off = 1 };
                dirs[2] = new DirOffData { dir = new int3(0, -1, 0), off = -size };
                dirs[3] = new DirOffData { dir = new int3(0, 1, 0), off = size };
                dirs[4] = new DirOffData { dir = new int3(0, 0, -1), off = -size * size };
                dirs[5] = new DirOffData { dir = new int3(0, 0, 1), off = size * size };
                for (var z = 0; z < size; z++)
                    for (var y = 0; y < size; y++)
                        for (var x = 0; x < size; x++)
                        {
                            var idx = SpaceFoundation.Idx(x, y, z, size);
                            var v = voxelArray[idx];
                            if (v.anchorId == 0 || v.distance == 0) continue;
                            var cutoff = true;
                            var n = 0;
                            foreach (var tuple in dirs)
                            {
                                int nx = x + tuple.dir.x, ny = y + tuple.dir.y, nz = z + tuple.dir.z;
                                if (nx < 0 || nx >= size || ny < 0 || ny >= size || nz < 0 || nz >= size) continue;
                                var neighbor = voxelArray[idx + tuple.off];
                                ++n;
                                if (neighbor.anchorId != v.anchorId ||
                                    neighbor.distance >= v.distance) continue;
                                cutoff = false;
                                break;
                            }

                            if (!cutoff) continue;
                            
                            if (n < 6)
                            {
                                foreach (var tuple in dirs)
                                    if (x + tuple.dir.x < 0 || x + tuple.dir.x >= size ||
                                        y + tuple.dir.y < 0 || y + tuple.dir.y >= size ||
                                        z + tuple.dir.z < 0 || z + tuple.dir.z >= size)
                                        possiblyCutoffVoxels.Add(new OffIdxVoxelData{ off = new int3(tuple.dir.x, tuple.dir.y, tuple.dir.z), idx = idx, voxel = v});
                            }
                            else
                            {
                                RemoveCutoffVoxels(idx, v, frontierQueue1, frontierQueue2, cutoffRemovedHS,
                                    borderVoxels, visitedArray, lftBorder, rgtBorder, botBorder, topBorder, bckBorder,
                                    fwdBorder);
                            }
                        }
                dirs.Dispose();
                
                lftBorderHM.Clear();
                rgtBorderHM.Clear();
                botBorderHM.Clear();
                topBorderHM.Clear();
                bckBorderHM.Clear();
                fwdBorderHM.Clear();
            }
            
            foreach (var iv in lftBorder) lftBorderHM.TryAdd(iv.idx, iv.voxel);
            foreach (var iv in rgtBorder) rgtBorderHM.TryAdd(iv.idx, iv.voxel);
            foreach (var iv in botBorder) botBorderHM.TryAdd(iv.idx, iv.voxel);
            foreach (var iv in topBorder) topBorderHM.TryAdd(iv.idx, iv.voxel);
            foreach (var iv in bckBorder) bckBorderHM.TryAdd(iv.idx, iv.voxel);
            foreach (var iv in fwdBorder) fwdBorderHM.TryAdd(iv.idx, iv.voxel);
            
            lftCutoffBorderHM.Clear();
            rgtCutoffBorderHM.Clear();
            botCutoffBorderHM.Clear();
            topCutoffBorderHM.Clear();
            bckCutoffBorderHM.Clear();
            fwdCutoffBorderHM.Clear();

            frontierQueue1.Dispose();
            frontierQueue2.Dispose();
            visitedArray.Dispose();
            initialVoxels.Dispose();
            cutoffVoxels.Dispose();
            cutoffRemovedHS.Dispose();
            borderVoxels.Dispose();
            lftBorder.Dispose();
            rgtBorder.Dispose();
            botBorder.Dispose();
            topBorder.Dispose();
            bckBorder.Dispose();
            fwdBorder.Dispose();
        }

        /// <summary>
        /// Performs local inner chunk flood fill
        /// </summary>
        private void StartExploration(NativeQueue<IdxVoxelData> frontierQueue1,
            NativeQueue<IdxVoxelData> frontierQueue2,
            NativeArray<IdxVoxelData> initialVoxels, NativeBitArray visitedArray, NativeList<IdxVoxelData> lftBorder,
            NativeList<IdxVoxelData> rgtBorder, NativeList<IdxVoxelData> botBorder, NativeList<IdxVoxelData> topBorder,
            NativeList<IdxVoxelData> bckBorder, NativeList<IdxVoxelData> fwdBorder)
        {
            if (initialVoxels.Length == 0) return;
            
            var initialVoxelsHead = 0;
            var currentFrontierQueue = frontierQueue1;
            var nextFrontierQueue = frontierQueue2;
            initialVoxels.Sort(new VoxelDistComparator());
            var currentDist = initialVoxels[0].voxel.distance;
            
            while (initialVoxelsHead < initialVoxels.Length && initialVoxels[initialVoxelsHead].voxel.distance == currentDist)
            {
                currentFrontierQueue.Enqueue(initialVoxels[initialVoxelsHead++]);
            }
            
            // BFS expansion
            while (!currentFrontierQueue.IsEmpty())
            {
                while (!currentFrontierQueue.IsEmpty())
                {
                    ExploreVoxel(currentFrontierQueue.Dequeue(), nextFrontierQueue, visitedArray, lftBorder, rgtBorder, botBorder, topBorder, bckBorder, fwdBorder);
                }
                
                currentDist = initialVoxelsHead < initialVoxels.Length ? initialVoxels[initialVoxelsHead].voxel.distance : ushort.MinValue;
                while ((initialVoxelsHead < initialVoxels.Length && 
                        initialVoxels[initialVoxelsHead].voxel.distance == currentDist) || 
                       !nextFrontierQueue.IsEmpty())
                {
                    var hasInitial = initialVoxelsHead < initialVoxels.Length &&
                                      initialVoxels[initialVoxelsHead].voxel.distance == currentDist;
                    var hasNext = !nextFrontierQueue.IsEmpty();

                    if (hasInitial && (!hasNext || initialVoxels[initialVoxelsHead].voxel < nextFrontierQueue.Peek().voxel))
                    {
                        currentFrontierQueue.Enqueue(initialVoxels[initialVoxelsHead]);
                        initialVoxelsHead++;
                    }
                    else
                    {
                        currentFrontierQueue.Enqueue(nextFrontierQueue.Dequeue());
                    }
                }
            }
        }


        /// <summary>
        /// Deletes and refills cutoff volume in 2 steps
        /// </summary>
        private void RemoveCutoffVoxels(int idx, ChunkVoxelData cutoffVoxel, NativeQueue<IdxVoxelData> frontierQueue1,
            NativeQueue<IdxVoxelData> frontierQueue2, NativeHashSet<int> cutoffRemovedHS,
            NativeList<IdxVoxelData> borderVoxels,
            NativeBitArray visitedArray, NativeList<IdxVoxelData> lftBorder, NativeList<IdxVoxelData> rgtBorder,
            NativeList<IdxVoxelData> botBorder, NativeList<IdxVoxelData> topBorder, NativeList<IdxVoxelData> bckBorder,
            NativeList<IdxVoxelData> fwdBorder)
        {
            var queueToCheck = new NativeQueue<int>(Allocator.Temp);
            cutoffRemovedHS.Clear();
            var emptyVoxel = new ChunkVoxelData();
            queueToCheck.Enqueue(idx);
            
            // Delete cutoff volume => remove all voxels with dist > cutoffVoxel and same anchor
            while (!queueToCheck.IsEmpty())
            {
                var cIdx = queueToCheck.Dequeue();
                var voxel = voxelArray[cIdx];
                if (voxel.anchorId != cutoffVoxel.anchorId || 
                    voxel.distance < cutoffVoxel.distance) continue;
                
                voxelArray[cIdx] = emptyVoxel;
                cutoffRemovedHS.Add(cIdx);
                visitedArray.Set(cIdx, false);
                
                var pos = SpaceFoundation.ReverseIdx(cIdx, size);
                if (pos.x > 0) queueToCheck.Enqueue(cIdx - 1);
                if (pos.x < size - 1) queueToCheck.Enqueue(cIdx + 1);
                if (pos.y > 0) queueToCheck.Enqueue(cIdx - size);
                if (pos.y < size - 1) queueToCheck.Enqueue(cIdx + size);
                if (pos.z > 0) queueToCheck.Enqueue(cIdx - size * size);
                if (pos.z < size - 1) queueToCheck.Enqueue(cIdx + size * size);
            }
            queueToCheck.Dispose();
            if (cutoffRemovedHS.Count == 0) return;
            
            // Get adjacency hashset of voxels next to the emptied volume
            borderVoxels.Clear();
            foreach (var cIdx in cutoffRemovedHS)
            {
                var pos = SpaceFoundation.ReverseIdx(cIdx, size);
                if (pos.x > 0) CollectCutoffBorderVoxel(voxelArray, relativeAnchors, cIdx, cIdx - 1, size);
                else CollectCutoffBorderVoxel(lftVoxelArray, relativeAnchors, cIdx, SpaceFoundation.Idx(size - 1, pos.y, pos.z, size), size);
                if (pos.x < size - 1) CollectCutoffBorderVoxel(voxelArray, relativeAnchors, cIdx, cIdx + 1, size);
                else CollectCutoffBorderVoxel(rgtVoxelArray, relativeAnchors, cIdx, SpaceFoundation.Idx(0, pos.y, pos.z, size), size);
                if (pos.y > 0) CollectCutoffBorderVoxel(voxelArray, relativeAnchors, cIdx, cIdx - size, size);
                else CollectCutoffBorderVoxel(botVoxelArray, relativeAnchors, cIdx, SpaceFoundation.Idx(pos.x, size - 1, pos.z, size), size);
                if (pos.y < size - 1) CollectCutoffBorderVoxel(voxelArray, relativeAnchors, cIdx, cIdx + size, size);
                else CollectCutoffBorderVoxel(topVoxelArray, relativeAnchors, cIdx, SpaceFoundation.Idx(pos.x, 0, pos.z, size), size);
                if (pos.z > 0) CollectCutoffBorderVoxel(voxelArray, relativeAnchors, cIdx, cIdx - size * size, size);
                else CollectCutoffBorderVoxel(bckVoxelArray, relativeAnchors, cIdx, SpaceFoundation.Idx(pos.x, pos.y, size - 1, size), size);
                if (pos.z < size - 1) CollectCutoffBorderVoxel(voxelArray, relativeAnchors, cIdx, cIdx + size * size, size);
                else CollectCutoffBorderVoxel(fwdVoxelArray, relativeAnchors, cIdx, SpaceFoundation.Idx(pos.x, pos.y, 0, size), size);
            }
            
            // Refill the volumes
            StartExploration(frontierQueue1, frontierQueue2, borderVoxels.AsArray(), visitedArray, lftBorder, rgtBorder, botBorder, topBorder, bckBorder, fwdBorder);
            return;
            
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            void CollectCutoffBorderVoxel(NativeArray<ChunkVoxelData> voxelArray, NativeHashMap<int, VoxelAnchor> relativeAnchors, int cIdx, int nIdx, int chunkSize)
            {
                var nv = voxelArray[nIdx];
                if (nv.anchorId == 0 || !relativeAnchors.TryGetValue(nv.anchorId, out var va) ||
                    !va.IsWithinInfluenceSphere(SpaceFoundation.ReverseIdx(cIdx, chunkSize))) return;
                nv.distance += 1;
                borderVoxels.Add(new IdxVoxelData { idx = cIdx, voxel = nv });
            }
        }

        private void ExploreVoxel(IdxVoxelData tuple, NativeQueue<IdxVoxelData> nextFrontierQueue, NativeBitArray visitedArray, 
            NativeList<IdxVoxelData> lftBorder, NativeList<IdxVoxelData> rgtBorder, NativeList<IdxVoxelData> botBorder,
            NativeList<IdxVoxelData> topBorder, NativeList<IdxVoxelData> bckBorder, NativeList<IdxVoxelData> fwdBorder)
        {
            var voxelIdx = tuple.idx;
            
            var voxel = voxelArray[voxelIdx];
            var prevVoxel = tuple.voxel;
            
            if (visitedArray.GetBits(voxelIdx) == 1 || delimArray.GetBits(voxelIdx) == 1 || 
                !relativeAnchors[prevVoxel.anchorId].IsWithinInfluenceSphere(SpaceFoundation.ReverseIdx(voxelIdx, size))) return;

            visitedArray.Set(voxelIdx, true);
            if (voxel.anchorId != 0 && voxel <= prevVoxel) return;
            
            voxelArray[voxelIdx] = prevVoxel;
            
            //Add neighbors to nextFrontierQueue
            var pos = SpaceFoundation.ReverseIdx(voxelIdx, size);
            var nextVoxel = new ChunkVoxelData
            {
                anchorId = prevVoxel.anchorId, 
                distance = (ushort)(prevVoxel.distance + 1)
            };
            if (pos.x > 0) nextFrontierQueue.Enqueue(new IdxVoxelData { idx = voxelIdx - 1, voxel = nextVoxel });
            else if(!lftBorderHM.TryGetValue(voxelIdx, out var v) || v.anchorId != voxel.anchorId) 
                lftBorder.Add(new IdxVoxelData{ idx = SpaceFoundation.Idx(size - 1, pos.y, pos.z, size), voxel = nextVoxel}); 
            if (pos.x < size - 1) nextFrontierQueue.Enqueue(new IdxVoxelData { idx = voxelIdx + 1, voxel = nextVoxel });
            else if(!rgtBorderHM.TryGetValue(voxelIdx, out var v) || v.anchorId != voxel.anchorId) 
                rgtBorder.Add(new IdxVoxelData{ idx = SpaceFoundation.Idx(0, pos.y, pos.z, size), voxel = nextVoxel}); 
            if (pos.y > 0) nextFrontierQueue.Enqueue(new IdxVoxelData { idx = voxelIdx - size, voxel = nextVoxel });
            else if(!botBorderHM.TryGetValue(voxelIdx, out var v) || v.anchorId != voxel.anchorId) 
                botBorder.Add(new IdxVoxelData{ idx = SpaceFoundation.Idx(pos.x, size - 1, pos.z, size), voxel = nextVoxel}); 
            if (pos.y < size - 1) nextFrontierQueue.Enqueue(new IdxVoxelData { idx = voxelIdx + size, voxel = nextVoxel });
            else if(!topBorderHM.TryGetValue(voxelIdx, out var v) || v.anchorId != voxel.anchorId) 
                topBorder.Add(new IdxVoxelData{ idx = SpaceFoundation.Idx(pos.x, 0, pos.z, size), voxel = nextVoxel}); 
            if (pos.z > 0) nextFrontierQueue.Enqueue(new IdxVoxelData { idx = voxelIdx - size * size, voxel = nextVoxel });
            else if(!bckBorderHM.TryGetValue(voxelIdx, out var v) || v.anchorId != voxel.anchorId) 
                bckBorder.Add(new IdxVoxelData{ idx = SpaceFoundation.Idx(pos.x, pos.y, size - 1, size), voxel = nextVoxel});
            if (pos.z < size - 1) nextFrontierQueue.Enqueue(new IdxVoxelData { idx = voxelIdx + size * size, voxel = nextVoxel });
            else if(!fwdBorderHM.TryGetValue(voxelIdx, out var v) || v.anchorId != voxel.anchorId) 
                fwdBorder.Add(new IdxVoxelData{ idx = SpaceFoundation.Idx(pos.x, pos.y, 0, size), voxel = nextVoxel});
        }
    } 
}