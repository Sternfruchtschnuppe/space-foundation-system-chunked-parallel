using System;
using System.Collections.Generic;
using System.IO;
using System.Threading.Tasks;
using Unity.Mathematics;
using UnityEditor;
using UnityEditor.AddressableAssets;
using UnityEditor.AddressableAssets.Settings;
using UnityEngine;
using UnityEngine.AddressableAssets;
using UnityEngine.ResourceManagement.AsyncOperations;

namespace SpaceFoundationSystem
{
    public class AddressablesManager
    {
        private readonly string saveFolderPath = "Assets/SpaceFoundationData/";
        private readonly string benchmarkFolderPath = "Benchmarks/";
        private SpaceFoundation spaceFoundation;
        private List<(string path, string id)> unregisteredAddressables = new();
        private AddressableAssetSettings settings;
        
        private HashSet<int3> emptyLoadedChunks = new();
        private Dictionary<int3, RTChunk> loadedChunks = new();
        private Queue<(int3, AsyncOperationHandle<SpaceFoundationChunkData>)> loadedChunkPosHandles;
        
        public AddressablesManager(SpaceFoundation spaceFoundation)
        {
            this.spaceFoundation = spaceFoundation;
            loadedChunkPosHandles = new (spaceFoundation.maxLoadedChunks);
            settings = AddressableAssetSettingsDefaultObject.GetSettings(false);
            if (settings == null)
            {
                Debug.LogError("Addressables Settings not found. Please open Addressables Groups window once.");
            }
        }
        
        public async Task ClearChunksAsync()
        {
            emptyLoadedChunks.Clear();
            loadedChunks.Clear();
            foreach (var t in loadedChunkPosHandles)
            {
                Addressables.Release(t.Item2);
            }
            loadedChunkPosHandles.Clear();
            
            await AwaitableResourcesUnloadUnusedAssets();
            static Task AwaitableResourcesUnloadUnusedAssets()
            {
                var unloadOp = Resources.UnloadUnusedAssets();
                var tcs = new TaskCompletionSource<object>();

                unloadOp.completed += _ => tcs.SetResult(null);

                return tcs.Task;
            }
            await Task.Yield(); 
        }
        
        public async Task<RTChunk> LoadChunkAsync(int3 chunkPosition)
        {
            if (loadedChunks.TryGetValue(chunkPosition, out var cachedChunk)) return cachedChunk;
            if (emptyLoadedChunks.TryGetValue(chunkPosition, out var emptyChunk)) return null;

            var key = GetSpaceFoundationAssetString(true)+GetChunkAssetString(chunkPosition);

            var locationHandle = Addressables.LoadResourceLocationsAsync(key);
            await locationHandle.Task;
            
            if (locationHandle.Status != AsyncOperationStatus.Succeeded || locationHandle.Result.Count == 0)
            {
                emptyLoadedChunks.Add(chunkPosition);
                return null;
            }

            var handle = Addressables.LoadAssetAsync<SpaceFoundationChunkData>(key);
            await handle.Task;

            if (handle.Status != AsyncOperationStatus.Succeeded) return null;

            var chunk = new RTChunk
            {
                borders = handle.Result.borders.ToDictionary()
            };
            loadedChunks[chunkPosition] = chunk;
            loadedChunkPosHandles.Enqueue((chunkPosition, handle));
            if (loadedChunkPosHandles.Count > spaceFoundation.maxLoadedChunks)
            {
                UnloadChunk(loadedChunkPosHandles.Dequeue());
            }
            return chunk;
        }

        private void UnloadChunk((int3 chunkPosition,  AsyncOperationHandle<SpaceFoundationChunkData> handle) data)
        {
            if (!loadedChunks.Remove(data.chunkPosition, out _)) return;
            Addressables.Release(data.handle);
        }

        public void PrepareAssetFolder(bool chunked)
        {
            EnsureEmptyFolder(saveFolderPath + GetSpaceFoundationAssetString(chunked));
        }
        
        public void SaveBenchmark(List<string> csvData, bool chunked)
        {
            EnsureFolderExists("Assets/" + benchmarkFolderPath);
            var csvString = string.Join("\n", csvData);
            var fileName = $"{GetSpaceFoundationAssetString(chunked)}_{DateTime.Now:yyyyMMdd_HHmmss}.csv";
            var filePath = Path.Combine(Application.dataPath, benchmarkFolderPath, fileName);
            File.WriteAllText(filePath, csvString);
        }

        private void EnsureEmptyFolder(string path)
        {
            if (AssetDatabase.IsValidFolder(path))
            {
                AssetDatabase.DeleteAsset(path); 
            }

            EnsureFolderExists(path);
        }

        private static void EnsureFolderExists(string path)
        {
            var parts = path.Split('/');
            var current = parts[0];
            for (var i = 1; i < parts.Length; i++)
            {
                var next = current + "/" + parts[i];
                if (!AssetDatabase.IsValidFolder(next))
                    AssetDatabase.CreateFolder(current, parts[i]);
                current = next;
            }
        }


        public void CreateSpaceFoundationAsset(SpaceFoundationData data, bool chunked)
        {
            var path = GetSpaceFoundationAssetFullPath(chunked);
            AssetDatabase.CreateAsset(data, path);
            unregisteredAddressables.Add((path, GetSpaceFoundationAssetString(chunked)));
        }

        public void CreateChunkAsset(SpaceFoundationChunkData chunkData, int3 chunkPosition)
        {
            var path = GetChunkAssetFullPath(chunkPosition);
            AssetDatabase.CreateAsset(chunkData, path);
            unregisteredAddressables.Add((path, GetSpaceFoundationAssetString(true)+GetChunkAssetString(chunkPosition)));
        }

        public void SaveAndRegisterAddressables()
        {
            AssetDatabase.SaveAssets();
            
            var group = settings.DefaultGroup;
            foreach (var (path, id) in unregisteredAddressables)
            {
                var entry = settings.CreateOrMoveEntry(AssetDatabase.AssetPathToGUID(path), group);
                entry.address = id;
            }
        }
        
        private string GetSpaceFoundationAssetFullPath(bool chunked)
        {
            return $"{saveFolderPath}{GetSpaceFoundationAssetString(chunked)}/{GetSpaceFoundationAssetString(chunked)}.asset";
        }
        
        private string GetSpaceFoundationAssetString(bool chunked = false)
        {
            return spaceFoundation.assetName + (chunked ? "_chunked" : "");
        }
        
        private string GetChunkAssetFullPath(int3 chunkPos)
        {
            return $"{saveFolderPath}{GetSpaceFoundationAssetString(true)}/{GetChunkAssetString(chunkPos)}.asset";
        }
        
        private string GetChunkAssetString(int3 chunkPos)
        {
            return chunkPos.x + "_" + chunkPos.y + "_" + chunkPos.z;
        }
    }

}