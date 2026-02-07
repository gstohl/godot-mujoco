using System;
using System.IO;
using System.Text.Json;

public sealed class VecNormalizeStats
{
    public double[] ObsMean { get; private set; } = Array.Empty<double>();
    public double[] ObsVar { get; private set; } = Array.Empty<double>();
    public double ClipObs { get; private set; } = 10.0;
    public double Epsilon { get; private set; } = 1e-8;

    public static bool TryLoad(string filePath, out VecNormalizeStats stats, out string error)
    {
        stats = new VecNormalizeStats();
        error = string.Empty;

        if (!File.Exists(filePath))
        {
            error = "VecNorm file does not exist: " + filePath;
            return false;
        }

        try
        {
            string json = File.ReadAllText(filePath);
            using JsonDocument doc = JsonDocument.Parse(json);
            JsonElement root = doc.RootElement;

            stats.ObsMean = ReadDoubleArray(root, "obs_mean");
            stats.ObsVar = ReadDoubleArray(root, "obs_var");

            if ((stats.ObsMean.Length == 0 || stats.ObsVar.Length == 0) &&
                root.TryGetProperty("obs_rms", out JsonElement obsRms) &&
                obsRms.ValueKind == JsonValueKind.Object)
            {
                stats.ObsMean = ReadDoubleArray(obsRms, "mean");
                stats.ObsVar = ReadDoubleArray(obsRms, "var");
            }
            if (root.TryGetProperty("clip_obs", out JsonElement clipObs))
            {
                stats.ClipObs = clipObs.GetDouble();
            }
            if (root.TryGetProperty("epsilon", out JsonElement eps))
            {
                stats.Epsilon = eps.GetDouble();
            }

            if (stats.ObsMean.Length == 0 || stats.ObsVar.Length == 0)
            {
                error = "VecNorm stats missing obs_mean or obs_var";
                return false;
            }

            if (stats.ObsMean.Length != stats.ObsVar.Length)
            {
                error = "VecNorm length mismatch between obs_mean and obs_var";
                return false;
            }

            return true;
        }
        catch (Exception ex)
        {
            error = "VecNorm parse failed: " + ex.Message;
            return false;
        }
    }

    public void NormalizeInPlace(double[] observation)
    {
        int count = Math.Min(observation.Length, ObsMean.Length);
        for (int i = 0; i < count; i++)
        {
            double denom = Math.Sqrt(Math.Max(ObsVar[i], 0.0) + Epsilon);
            double norm = (observation[i] - ObsMean[i]) / denom;
            if (norm > ClipObs)
            {
                norm = ClipObs;
            }
            else if (norm < -ClipObs)
            {
                norm = -ClipObs;
            }
            observation[i] = norm;
        }
    }

    private static double[] ReadDoubleArray(JsonElement root, string propertyName)
    {
        if (!root.TryGetProperty(propertyName, out JsonElement arr) || arr.ValueKind != JsonValueKind.Array)
        {
            return Array.Empty<double>();
        }

        double[] outArr = new double[arr.GetArrayLength()];
        for (int i = 0; i < outArr.Length; i++)
        {
            outArr[i] = arr[i].GetDouble();
        }
        return outArr;
    }
}
