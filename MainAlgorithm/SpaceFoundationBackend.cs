using System;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;
using System.IO;
using UnityEditor;
using UnityEngine;
using UnityEngine.SceneManagement;
using Object = UnityEngine.Object;
using Debug = UnityEngine.Debug;
using MathNet.Numerics.Statistics;
using SpaceFoundationSystem.Util;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace SpaceFoundationSystem
{
    public static class SpaceFoundationBackend
    {
        private const bool Benchmark = false;
        private const int MaxDurationMinutes = 420;
        private const int MaxIterations = 1;

        public static void RunChunkedComputeGraph()
        {
            var spaceFoundation = Object.FindObjectOfType<SpaceFoundation>();
            spaceFoundation.Reset();
            
            // Collect Anchors, setup id mapping
            var anchors = Object.FindObjectsByType<Anchor>(FindObjectsSortMode.None);
            var anchorIntToStringIdDict = new Dictionary<ushort, string>();
            var anchorStringToIntIdDict = new Dictionary<string, ushort>();
            anchorIntToStringIdDict[0] = "_";
            anchorStringToIntIdDict["_"] = 0;
            var voxelAnchors = new VoxelAnchor[anchors.Length];
            var voxelAnchorsHM = new NativeHashMap<int, VoxelAnchor>(voxelAnchors.Length, Allocator.Persistent);
            
            for (var i = 0; i < anchors.Length; i++)
            {
                var id = spaceFoundation.GetNextAnchorUniqueIntId();
                anchors[i].correspondingSpaceFoundation = spaceFoundation;
                anchors[i].SetUniqueId(spaceFoundation.GetAnchorIdString(id));
                //id mapping +1 is for id 0 = no anchor = empty space or blocked by delimiter
                var voxelAnchor = new VoxelAnchor((ushort)(id + 1), anchors[i].transform.position, anchors[i].GetMaxDistance(), spaceFoundation.chunkSize, spaceFoundation.voxelSize);
                voxelAnchors[i] = voxelAnchor;
                voxelAnchorsHM.Add(voxelAnchor.id, voxelAnchor);
                EditorUtility.SetDirty(anchors[i]);
                anchorStringToIntIdDict[anchors[i].GetUniqueId()] = voxelAnchors[i].id;
                anchorIntToStringIdDict[voxelAnchors[i].id] = anchors[i].GetUniqueId();
            }
            
            var delimiters = Object.FindObjectsByType<Delimiter>(FindObjectsSortMode.None);
            var colliderToDelimIdHS = new Dictionary<Collider, int>();
            var delimIntToStringIdDict = new Dictionary<int, string>();
            foreach (var d in delimiters)
            {
                d.ClearDelimitedAnchors();
                var id = spaceFoundation.GetNextDelimiterUniqueIntId();
                d.SetUniqueId("d"+id);
                //increment id to make space for id 0=Collider, but no delimiter component
                ++id;
                colliderToDelimIdHS[d.GetComponent<Collider>()] = id; 
                EditorUtility.SetDirty(d);
                delimIntToStringIdDict[id] = d.GetUniqueId();
            }
            
            if (Benchmark) System.Threading.Thread.Sleep(3000);
            var stopwatch = new Stopwatch();
            double totalElapsedMilliseconds = 0;
            int iterationCount = 0;
            List<double> elapsedTimes = new List<double>();
            List<string> csvData = new List<string> { "Iteration,Milliseconds,Hours,Minutes,Seconds,Milliseconds,AlgIter,ChunkCreation" };
            
            do
            {
                // Initial Iteration
                var chunkStopwatch = new Stopwatch();
                stopwatch.Restart();
                //create chunks
                var dirtyChunks = new HashSet<Chunk>();
                var chunkDict = new Dictionary<int3, Chunk>();
                foreach (var grp in voxelAnchors.GroupBy(a => a.chunkPosition))
                {
                    if (!chunkDict.TryGetValue(grp.Key, out var chunk))
                    {
                        chunkStopwatch.Start();
                        chunk = new Chunk(grp.Key, spaceFoundation.delimitingLayerMask, grp.ToList(),
                            voxelAnchors, colliderToDelimIdHS, spaceFoundation.chunkSize, spaceFoundation.voxelSize);
                        chunkStopwatch.Stop();
                        chunkDict.Add(grp.Key, chunk);
                    }

                    dirtyChunks.Add(chunk);
                }


                var neighborDeltas = new[] { new int3(-1, 0, 0), new int3(1, 0, 0), new int3(0, -1, 0), new int3(0, 1, 0), new int3(0, 0, -1), new int3(0, 0, 1) };
                var isBorderEmpty = new bool[6];
                var emptyVoxelArray = new NativeArray<ChunkVoxelData>(
                    spaceFoundation.chunkSize * spaceFoundation.chunkSize * spaceFoundation.chunkSize,
                    Allocator.TempJob);
                var emptyDelimMHM = new NativeParallelMultiHashMap<int, int>(0, Allocator.TempJob);

                ScheduleFloodFillJobs(dirtyChunks, emptyVoxelArray);
                
                // Main Loop - Rest of Iterations
                var iter = 1;
                while (true)
                {
                    if (++iter > spaceFoundation.maxChunkIterations)
                    {
                        Debug.LogError("Max Iterations reached");
                        break;
                    }
                    
                    // detect cutoff voxels and mark them to be resolved
                    var s = spaceFoundation.chunkSize;
                    var newDirtyChunks = new HashSet<Chunk>();
                    foreach (var kv in chunkDict)
                    {
                        var chunk = kv.Value;
                        if (chunk.possiblyCutoffVoxels.Length == 0) continue;
                        var dirty = false;
                        var grpDict = new Dictionary<int, Dictionary<ushort, List<OffIdxVoxelData>>>();
                        foreach (var tuple in chunk.possiblyCutoffVoxels)
                        {
                            var idx = tuple.idx;
                            var anchorId = tuple.voxel.anchorId;

                            if (!grpDict.ContainsKey(idx)) grpDict[idx] = new Dictionary<ushort, List<OffIdxVoxelData>>();
                            if (!grpDict[idx].ContainsKey(anchorId)) grpDict[idx][anchorId] = new List<OffIdxVoxelData>();
                            grpDict[idx][anchorId].Add(tuple);
                        }

                        foreach (var idxGrp in grpDict)
                        {
                            var pos = SpaceFoundation.ReverseIdx(idxGrp.Key, s);
                            foreach (var anchorGrp in idxGrp.Value)
                            {
                                var voxelDirty = true;
                                foreach (var tuple in anchorGrp.Value)
                                {
                                    var globalPos = tuple.off + pos + s * chunk.chunkPosition;
                                    if (!TryGetVoxelGlobal(globalPos, chunkDict, out var neighbor) ||
                                        tuple.voxel.anchorId != neighbor.anchorId ||
                                        tuple.voxel.distance <= neighbor.distance) continue;
                                    voxelDirty = false;
                                    break;
                                }
                                
                                if (voxelDirty)
                                {
                                    foreach (var tuple in anchorGrp.Value)
                                    {
                                        if (tuple.off.x == -1 && chunk.lftCutoffBorderHM.TryAdd(idxGrp.Key, tuple.voxel)) break;
                                        if (tuple.off.x == 1  && chunk.rgtCutoffBorderHM.TryAdd(idxGrp.Key, tuple.voxel)) break;
                                        if (tuple.off.y == -1 && chunk.botCutoffBorderHM.TryAdd(idxGrp.Key, tuple.voxel)) break;
                                        if (tuple.off.y == 1  && chunk.topCutoffBorderHM.TryAdd(idxGrp.Key, tuple.voxel)) break;
                                        if (tuple.off.z == -1 && chunk.bckCutoffBorderHM.TryAdd(idxGrp.Key, tuple.voxel)) break;
                                        if (tuple.off.z == 1  && chunk.fwdCutoffBorderHM.TryAdd(idxGrp.Key, tuple.voxel)) break;
                                    }
                                }
                                dirty = dirty || voxelDirty;
                            }
                        }
                    
                        if (!dirty) continue;
                        new VoxelFloodFillJob
                        {
                            delimArray = chunk.delimArray,
                            voxelArray = chunk.voxelArray,
                            lftVoxelArray = chunkDict.TryGetValue(chunk.chunkPosition + new int3(-1, 0, 0), out var nc) ? nc.voxelArray : emptyVoxelArray,
                            rgtVoxelArray = chunkDict.TryGetValue(chunk.chunkPosition + new int3( 1, 0, 0), out nc) ? nc.voxelArray : emptyVoxelArray,
                            botVoxelArray = chunkDict.TryGetValue(chunk.chunkPosition + new int3( 0,-1, 0), out nc) ? nc.voxelArray : emptyVoxelArray,
                            topVoxelArray = chunkDict.TryGetValue(chunk.chunkPosition + new int3( 0, 1, 0), out nc) ? nc.voxelArray : emptyVoxelArray,
                            bckVoxelArray = chunkDict.TryGetValue(chunk.chunkPosition + new int3( 0, 0,-1), out nc) ? nc.voxelArray : emptyVoxelArray,
                            fwdVoxelArray = chunkDict.TryGetValue(chunk.chunkPosition + new int3( 0, 0, 1), out nc) ? nc.voxelArray : emptyVoxelArray,
                            chunkAnchors = chunk.chunkAnchors,
                            relativeAnchors = chunk.relativeAnchors,
                            lftBorderHM = chunk.lftBorderHM,
                            rgtBorderHM = chunk.rgtBorderHM,
                            botBorderHM = chunk.botBorderHM,
                            topBorderHM = chunk.topBorderHM,
                            bckBorderHM = chunk.bckBorderHM,
                            fwdBorderHM = chunk.fwdBorderHM,
                            possiblyCutoffVoxels = chunk.possiblyCutoffVoxels,
                            lftCutoffBorderHM = chunk.lftCutoffBorderHM,
                            rgtCutoffBorderHM = chunk.rgtCutoffBorderHM,
                            botCutoffBorderHM = chunk.botCutoffBorderHM,
                            topCutoffBorderHM = chunk.topCutoffBorderHM,
                            bckCutoffBorderHM = chunk.bckCutoffBorderHM,
                            fwdCutoffBorderHM = chunk.fwdCutoffBorderHM,
                            size = chunk.size
                        }.Run();
                    }
                    
                    //create neighbors of dirty chunks, register them for swapping border arrays
                    var swappedBordersHS = new HashSet<(Chunk c, Chunk nc)>();
                    foreach (var chunk in dirtyChunks)
                    {
                        isBorderEmpty[0] = chunk.lftBorderHM.Count > 0;
                        isBorderEmpty[1] = chunk.rgtBorderHM.Count > 0;
                        isBorderEmpty[2] = chunk.botBorderHM.Count > 0;
                        isBorderEmpty[3] = chunk.topBorderHM.Count > 0;
                        isBorderEmpty[4] = chunk.bckBorderHM.Count > 0;
                        isBorderEmpty[5] = chunk.fwdBorderHM.Count > 0;

                        for (var i = 0; i < isBorderEmpty.Length; i++)
                        {
                            if (!isBorderEmpty[i]) continue;
                            var ncp = chunk.chunkPosition + neighborDeltas[i];
                            if (!chunkDict.TryGetValue(ncp, out var neighborChunk))
                            {
                                chunkStopwatch.Start();
                                neighborChunk = new Chunk(ncp, spaceFoundation.delimitingLayerMask,
                                    new List<VoxelAnchor>(),
                                    voxelAnchors, colliderToDelimIdHS, spaceFoundation.chunkSize,
                                    spaceFoundation.voxelSize);
                                chunkStopwatch.Stop();
                                chunkDict.Add(ncp, neighborChunk);
                            }

                            if (swappedBordersHS.Contains((chunk, neighborChunk)) ||
                                swappedBordersHS.Contains((neighborChunk, chunk))) continue;
                            swappedBordersHS.Add((chunk, neighborChunk));
                        }
                    }

                    //swap borders
                    foreach (var (c, nc) in swappedBordersHS)
                    {
                        newDirtyChunks.Add(c);
                        newDirtyChunks.Add(nc);
                        var delta = nc.chunkPosition - c.chunkPosition;
                        if (delta.Equals(neighborDeltas[0]))
                            (c.lftBorderHM, nc.rgtBorderHM) = (nc.rgtBorderHM, c.lftBorderHM);
                        else if (delta.Equals(neighborDeltas[1]))
                            (c.rgtBorderHM, nc.lftBorderHM) = (nc.lftBorderHM, c.rgtBorderHM);
                        else if (delta.Equals(neighborDeltas[2]))
                            (c.botBorderHM, nc.topBorderHM) = (nc.topBorderHM, c.botBorderHM);
                        else if (delta.Equals(neighborDeltas[3]))
                            (c.topBorderHM, nc.botBorderHM) = (nc.botBorderHM, c.topBorderHM);
                        else if (delta.Equals(neighborDeltas[4]))
                            (c.bckBorderHM, nc.fwdBorderHM) = (nc.fwdBorderHM, c.bckBorderHM);
                        else if (delta.Equals(neighborDeltas[5]))
                            (c.fwdBorderHM, nc.bckBorderHM) = (nc.bckBorderHM, c.fwdBorderHM);
                    }

                    dirtyChunks = newDirtyChunks;
                    // check if done
                    if (dirtyChunks.IsEmpty()) break;

                    ScheduleFloodFillJobs(dirtyChunks, emptyVoxelArray);
                }
                
                if (!Benchmark)
                {
                    // Post processing, collecting borders and delimiters
                    var jobs = new NativeArray<JobHandle>(chunkDict.Count, Allocator.TempJob);
                    var k = 0;
                    foreach (var (cp, chunk) in chunkDict)
                    {
                        var lft = chunkDict.TryGetValue(cp + new int3(-1, 0, 0), out var lftChunk);
                        var rgt = chunkDict.TryGetValue(cp + new int3(1, 0, 0), out var rgtChunk);
                        var bot = chunkDict.TryGetValue(cp + new int3(0, -1, 0), out var botChunk);
                        var top = chunkDict.TryGetValue(cp + new int3(0, 1, 0), out var topChunk);
                        var bck = chunkDict.TryGetValue(cp + new int3(0, 0, -1), out var bckChunk);
                        var fwd = chunkDict.TryGetValue(cp + new int3(0, 0, 1), out var fwdChunk);
                        jobs[k++] = new ChunkPostProcessJob()
                        {
                            voxelArray = chunk.voxelArray,
                            delimMHM = chunk.delimMHM,
                            lftVoxelArray = lft ? lftChunk.voxelArray : emptyVoxelArray,
                            lftDelimMHM = lft ? lftChunk.delimMHM : emptyDelimMHM,
                            rgtVoxelArray = rgt ? rgtChunk.voxelArray : emptyVoxelArray,
                            rgtDelimMHM = rgt ? rgtChunk.delimMHM : emptyDelimMHM,
                            botVoxelArray = bot ? botChunk.voxelArray : emptyVoxelArray,
                            botDelimMHM = bot ? botChunk.delimMHM : emptyDelimMHM,
                            topVoxelArray = top ? topChunk.voxelArray : emptyVoxelArray,
                            topDelimMHM = top ? topChunk.delimMHM : emptyDelimMHM,
                            bckVoxelArray = bck ? bckChunk.voxelArray : emptyVoxelArray,
                            bckDelimMHM = bck ? bckChunk.delimMHM : emptyDelimMHM,
                            fwdVoxelArray = fwd ? fwdChunk.voxelArray : emptyVoxelArray,
                            fwdDelimMHM = fwd ? fwdChunk.delimMHM : emptyDelimMHM,
                            size = spaceFoundation.chunkSize,
                            implicitDelimitersHS = chunk.implicitDelimitersHS,
                            explicitDelimitersHS = chunk.explicitDelimitersHS,
                            anchorBorderHM = chunk.anchorBorderHM,
                            anchorIdCntHM = chunk.anchorIdCntHM,
                            
                        }.Schedule();
                    }

                    JobHandle.CompleteAll(jobs);
                    jobs.Dispose();
                    
                    //Perform Marching Cubes
                    var s = spaceFoundation.chunkSize;
                    var ps = s + 2;
                    Debug.Log("process chunks");
                    foreach (var c in chunkDict.Values)
                    {
                        var chunkGO =
                            new GameObject($"Chunk [{c.chunkPosition.x}, {c.chunkPosition.y}, {c.chunkPosition.z}]");
                        chunkGO.transform.parent = spaceFoundation.transform;
                        chunkGO.transform.position =
                            SpaceFoundation.ChunkToWorld(c.chunkPosition, SpaceFoundation.s_VoxelSize);

                        var paddedVoxelArray = new NativeArray<ushort>(ps * ps * ps, Allocator.TempJob,
                            NativeArrayOptions.UninitializedMemory);

                        for (var z = 0; z < s; ++z)
                        for (var y = 0; y < s; ++y)
                        for (var x = 0; x < s; ++x)
                            paddedVoxelArray[x + 1 + (y + 1) * ps + (z + 1) * ps * ps] =
                                c.voxelArray[SpaceFoundation.Idx(x, y, z, spaceFoundation.chunkSize)].anchorId;

                        var min = c.chunkPosition * spaceFoundation.chunkSize + new int3(-1);
                        for (var y = 0; y < ps; ++y)
                        for (var x = 0; x < ps; ++x)
                        {
                            FillMCBuffer(ps, new int3(0, x, y), min, chunkDict, paddedVoxelArray);
                            FillMCBuffer(ps, new int3(ps - 1, x, y), min, chunkDict, paddedVoxelArray);
                            FillMCBuffer(ps, new int3(x, 0, y), min, chunkDict, paddedVoxelArray);
                            FillMCBuffer(ps, new int3(x, ps - 1, y), min, chunkDict, paddedVoxelArray);
                            FillMCBuffer(ps, new int3(x, y, 0), min, chunkDict, paddedVoxelArray);
                            FillMCBuffer(ps, new int3(x, y, ps - 1), min, chunkDict, paddedVoxelArray);
                        }

                        var meshes = FastMC.GenerateMeshesChunk(paddedVoxelArray, c.anchorIdCntHM);
                        paddedVoxelArray.Dispose();

                        var i = 0;
                        foreach (var t in c.anchorIdCntHM)
                        {
                            var aId = t.Key;
                            var anchorIdx = Array.FindIndex(voxelAnchors, a => a.id == aId);
                            var anchor = anchors[anchorIdx];

                            //todo move somewhere consistent with current SFS implementation
                            var subspace = new GameObject($"Subspace of {anchor.name} {aId}");
                            subspace.isStatic = true;
                            subspace.transform.parent = chunkGO.transform;
                            subspace.AddComponent<MeshFilter>().mesh = meshes[i++];
                            var collider = subspace.AddComponent<MeshCollider>();
                            collider.hideFlags = HideFlags.HideInHierarchy;
                            var space = subspace.AddComponent<Space>();
                            space.anchor = anchor;
                            subspace.transform.localPosition = Vector3.zero;
                            subspace.layer = LayerMask.NameToLayer(SpaceFoundation.s_SpaceLayer);

                            anchor.SetVolume(subspace);
                        }
                    }

                    // Save Location Graph Data 
                    var data = ScriptableObject.CreateInstance<SpaceFoundationData>();
                    for (var i = 0; i < voxelAnchors.Length; i++)
                    {
                        data.anchors.Add(anchorIntToStringIdDict[voxelAnchors[i].id], anchors[i]);
                        data.anchorToVoxelPositionDict.Add(anchorIntToStringIdDict[voxelAnchors[i].id],
                            anchors[i].GetVoxelGridPosition());
                    }

                    foreach (var kv in anchorStringToIntIdDict)
                    {
                        data.anchorStringToIntIdDict.Add(kv.Key, kv.Value);
                    }
                    
                    foreach (var kv in anchorIntToStringIdDict)
                    {
                        data.anchorIntToStringIdDict.Add(kv.Key, kv.Value);
                    }

                    var allExplicitDelimiters = new Dictionary<string, HashSet<string>>();
                    var allImplicitDelimiters = new HashSet<int2>();
                    foreach (var t in chunkDict)
                    {
                        foreach (var delimAnchorIds in t.Value.explicitDelimitersHS)
                        {
                            var delimStrId = delimIntToStringIdDict[delimAnchorIds.x];
                            var anchorStrId = anchorIntToStringIdDict[(ushort)delimAnchorIds.y];
                            if (!allExplicitDelimiters.ContainsKey(delimStrId))
                            {
                                allExplicitDelimiters.Add(delimStrId, new HashSet<string>());
                            }

                            allExplicitDelimiters[delimStrId].Add(anchorStrId);
                        }

                        foreach (var anchorIds in t.Value.implicitDelimitersHS)
                        {
                            if (allImplicitDelimiters.Contains(anchorIds) ||
                                allImplicitDelimiters.Contains(anchorIds.yx))
                                continue;
                            allImplicitDelimiters.Add(anchorIds);
                        }
                    }


                    foreach (var t in allExplicitDelimiters)
                    {
                        data.explicitDelimiters.Add(t.Key, new StringArrayWrapper(t.Value.ToArray()));
                    }

                    foreach (var anchorIds in allImplicitDelimiters)
                    {
                        data.implicitDelimiters.Add(spaceFoundation.GetNextDelimiterUniqueId(),
                            new StringArrayWrapper(new[]
                                { anchorIntToStringIdDict[(ushort)anchorIds.x], anchorIntToStringIdDict[(ushort)anchorIds.y] }));
                    }

                    spaceFoundation.addressablesManager.PrepareAssetFolder(true);
                    spaceFoundation.addressablesManager.CreateSpaceFoundationAsset(data, true);

                    foreach (var t in chunkDict)
                    {
                        var chunkData = ScriptableObject.CreateInstance<SpaceFoundationChunkData>();

                        chunkData.borders = new SerializableDictionary<int, string>();
                        foreach (var t1 in t.Value.anchorBorderHM) chunkData.borders.Add(t1.Key, anchorIntToStringIdDict[t1.Value]);
                        chunkData.chunkAnchorIds = new string[t.Value.anchorIdCntHM.Count];
                        var i = 0;
                        foreach (var kv in t.Value.anchorIdCntHM) chunkData.chunkAnchorIds[i++] = anchorIntToStringIdDict[kv.Key];
                        
                        spaceFoundation.addressablesManager.CreateChunkAsset(chunkData, t.Key);
                    }

                    spaceFoundation.addressablesManager.SaveAndRegisterAddressables();
                }
                
                emptyVoxelArray.Dispose();
                emptyDelimMHM.Dispose();
                foreach (var t in chunkDict) t.Value.Dispose();

                stopwatch.Stop();
                if (Benchmark && iterationCount == 0)
                {
                    Debug.Log($"Total Number of Chunks {chunkDict.Count}, number of voxels {chunkDict.Count * (int)(math.pow(spaceFoundation.chunkSize, 3))}, iterations: {iter}");
                }
                totalElapsedMilliseconds = CollectIterationStatistics(stopwatch, totalElapsedMilliseconds, elapsedTimes, iter, csvData, ref iterationCount);
                var elapsedMilliseconds = chunkStopwatch.ElapsedTicks / (double)Stopwatch.Frequency * 1000;
                csvData[^1] += $",{elapsedMilliseconds}";
            }
            while(Benchmark && totalElapsedMilliseconds < MaxDurationMinutes * 60 * 1000 && iterationCount < MaxIterations);

            if (Benchmark)
            {
                CalculateAndSaveStatistics(spaceFoundation, true, iterationCount, totalElapsedMilliseconds, csvData, elapsedTimes);
            }
            voxelAnchorsHM.Dispose();
            
        }
        
        private static void ScheduleFloodFillJobs(HashSet<Chunk> dirtyChunks, NativeArray<ChunkVoxelData> emptyVoxelArray)
        {
            NativeArray<JobHandle> jobs;
            int jobIdx;
            jobs = new NativeArray<JobHandle>(dirtyChunks.Count, Allocator.TempJob);
            jobIdx = 0;
            foreach (var chunk in dirtyChunks)
            {
                jobs[jobIdx++] = new VoxelFloodFillJob
                {
                    delimArray = chunk.delimArray,
                    voxelArray = chunk.voxelArray,
                    lftVoxelArray = emptyVoxelArray,
                    rgtVoxelArray = emptyVoxelArray,
                    botVoxelArray = emptyVoxelArray,
                    topVoxelArray = emptyVoxelArray,
                    bckVoxelArray = emptyVoxelArray,
                    fwdVoxelArray = emptyVoxelArray,
                    chunkAnchors = chunk.chunkAnchors,
                    relativeAnchors = chunk.relativeAnchors,
                    lftBorderHM = chunk.lftBorderHM,
                    rgtBorderHM = chunk.rgtBorderHM,
                    botBorderHM = chunk.botBorderHM,
                    topBorderHM = chunk.topBorderHM,
                    bckBorderHM = chunk.bckBorderHM,
                    fwdBorderHM = chunk.fwdBorderHM,
                    possiblyCutoffVoxels = chunk.possiblyCutoffVoxels,
                    lftCutoffBorderHM = chunk.lftCutoffBorderHM,
                    rgtCutoffBorderHM = chunk.rgtCutoffBorderHM,
                    botCutoffBorderHM = chunk.botCutoffBorderHM,
                    topCutoffBorderHM = chunk.topCutoffBorderHM,
                    bckCutoffBorderHM = chunk.bckCutoffBorderHM,
                    fwdCutoffBorderHM = chunk.fwdCutoffBorderHM,
                    size = chunk.size
                }.Schedule();
            }

            JobHandle.CompleteAll(jobs);
            jobs.Dispose();
        }

        private static NativeHashSet<int3> GetCompleteBorder(ushort anchorId, Chunk chunk, Dictionary<int3, Chunk> chunkDict)
        {
            var chunks = new HashSet<Chunk>();
            var chunkQueue = new Queue<Chunk>();
            chunks.Add(chunk);
            chunkQueue.Enqueue(chunk);
            var offs = new int3[] { new(-1, 0, 0), new(1, 0, 0), new(0, -1, 0), new(0, 1, 0), new(0, 0, -1), new(0, 0, 1) };
            while (!chunkQueue.IsEmpty())
            {
                var c = chunkQueue.Dequeue();
                foreach (var off in offs)
                {
                    var p = c.chunkPosition + off;
                    if (!chunkDict.TryGetValue(p, out var neighborChunk)) continue;
                    if (neighborChunk.anchorIdCntHM.ContainsKey(anchorId) && chunks.Add(neighborChunk))
                    {
                        chunkQueue.Enqueue(neighborChunk);
                    }
                }
            }

            var borders = new NativeHashSet<int3>();
            foreach (var c in chunks)
            {
                foreach (var posAnchorId in c.anchorBorderHM)
                {
                    if (posAnchorId.Value == anchorId)
                    {
                        borders.Add(SpaceFoundation.ReverseIdx(posAnchorId.Key, SpaceFoundation.s_ChunkSize)  + c.chunkPosition * SpaceFoundation.s_ChunkSize);
                    }
                }
            }

            return borders;
        }

        private static bool TryGetVoxelGlobal(int3 globalPos, Dictionary<int3, Chunk> chunkDict, out ChunkVoxelData voxelData)
        {
            var chunkPos = SpaceFoundation.VoxelToChunk(globalPos, SpaceFoundation.s_ChunkSize);
            voxelData = new ChunkVoxelData();
            if (!chunkDict.TryGetValue(chunkPos, out var chunk)) return false;

            var localPos = globalPos - chunkPos * SpaceFoundation.s_ChunkSize;
            voxelData = chunk.voxelArray[SpaceFoundation.Idx(localPos, SpaceFoundation.s_ChunkSize)];
            return true;
        }
        private static void FillMCBuffer(int paddedSize, int3 pos, int3 min, Dictionary<int3, Chunk> chunkDict, NativeArray<ushort> mcBuffer)
        {
            mcBuffer[pos.x + pos.y * paddedSize + pos.z * paddedSize * paddedSize] =
                TryGetVoxelGlobal(min + pos, chunkDict, out var voxel) ? voxel.anchorId : ushort.MinValue;
        }
    }
}