Benchmark fix

The benchmark previously used a 30-second cutoff, while the stable navigation run can require about 34 seconds to reach the goal. That made successful runs appear as 0%. The benchmark cutoff is now 60 seconds. The proposed pipeline is aligned with the stable dashboard configuration: A* + ANFIS-Adapted APF + Adaptive CTC.

Run the server on port 5050 and use the Benchmark Matrix button.
