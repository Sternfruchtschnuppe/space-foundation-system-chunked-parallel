using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading.Tasks;
using UnityEngine;
using MathNet.Numerics.Statistics;
using UnityEngine.Analytics;
using Debug = UnityEngine.Debug;

namespace SpaceFoundationSystem
{
    public class SampleBenchmarkManager : MonoBehaviour
    {
        private List<Transform> samplePositions;
        public int iterations = 1000;
        public int batchSize = 100;
        
        private async void Start()
        {
            var addressablesManager = FindObjectOfType<SpaceFoundation>().addressablesManager;
            samplePositions = FindObjectsOfType<SamplePosition>().Select(sp => sp.transform).ToList();
            
            foreach (var samplePosition in samplePositions)
            {
                var sp = samplePosition.name;
                Anchor location = samplePosition.DetermineLocation();
                Debug.Log(location);
                samplePosition.DetermineLocationChunked(anchor =>
                {
                    //debugging: ensure anchors match 
                    if (location != anchor)
                    {
                        var x = sp;
                    }
                });
            }
            
            var measurements = new List<double>();
            await BenchmarkLocationChecks(measurements);
            CalculateStatistics(measurements);
            
            var chunkMeasurements = new List<double>();
            await BenchmarkLocationChecksChunking(chunkMeasurements);
            CalculateStatistics(chunkMeasurements);
            
            // no chunk preloading
            // var chunkMeasurementsResetChunkCache = new List<double>();
            // await BenchmarkLocationChecksChunking(chunkMeasurementsResetChunkCache,  addressablesManager);
            // CalculateStatistics(chunkMeasurementsResetChunkCache);
        }

        private async Task BenchmarkLocationChecks(List<double> measurements)
        {
            var random = new System.Random();
            
            var randomSamplePositions = new List<Transform>(iterations);
            for (var i = 0; i < iterations; i++)
            {
                randomSamplePositions.Add(samplePositions[random.Next(samplePositions.Count)]);
            }

            for (var i = 0; i < iterations; i++)
            {
                randomSamplePositions[i].DetermineLocation(); 
            }
            
            for (var j = 0; j < batchSize; j++)
            {
                var stopwatch = new Stopwatch();
                stopwatch.Start();
                for (var i = 0; i < iterations; i++)
                {
                    randomSamplePositions[i].DetermineLocation(); 
                }
                stopwatch.Stop();

                var avgMs = (stopwatch.ElapsedTicks / (float)Stopwatch.Frequency * 1000) / iterations;
                
                measurements.Add(avgMs);
            }
        }
        
        private async Task BenchmarkLocationChecksChunking(List<double> measurements, AddressablesManager addressablesManager=null)
        {
            var totalTimeWatch = new Stopwatch();
            var random = new System.Random();

            var randomSamplePositions = new List<Transform>(iterations);
            for (var i = 0; i < iterations; i++)
            {
                randomSamplePositions.Add(samplePositions[random.Next(samplePositions.Count)]);
            }

            for (var i = 0; i < iterations; i++)
            {
                await randomSamplePositions[i].DetermineLocationChunkedAsync(); 
            }

            if (addressablesManager != null)
            {
                await addressablesManager.ClearChunksAsync();
            }
            
            for (var j = 0; j < batchSize; j++)
            {
                totalTimeWatch.Start();
                var stopwatch = new Stopwatch();
                stopwatch.Start();
                for (var i = 0; i < iterations; i++)
                {
                    await randomSamplePositions[i].DetermineLocationChunkedAsync(); 
                }
                stopwatch.Stop(); 
                totalTimeWatch.Stop();
                
                if (addressablesManager != null)
                {
                    await addressablesManager.ClearChunksAsync();
                }                
                var avgMs = (stopwatch.ElapsedTicks / (float)Stopwatch.Frequency * 1000) / iterations;
                measurements.Add(avgMs);
            }
        }


        private void CalculateStatistics(List<double> measurements)
        {
            var average = measurements.Average();
            var median = measurements.Median();
            var min = measurements.Min();
            var max = measurements.Max();
            var standardDeviation = measurements.StandardDeviation();
            measurements.Sort();
            var nfthPercentile = measurements[(int)(measurements.Count * 0.95f)];
           
            Debug.Log($"Statistics {iterations} iterations:\nAverage: {average} ms\nMedian: {median} ms\nMin: {min} ms\nMax: {max} ms\nStandard Deviation: {standardDeviation} ms \n95th percentile: {nfthPercentile} ms");

        }
    }
}