using System;
using System.IO;
using System.Text.Json;
using Godot;

public sealed class MjPolicyHotReloader : IDisposable
{
    private string _exportDirPath = string.Empty;
    private double _pollIntervalSec = 0.5;
    private double _pollAccumSec;
    private bool _useLatestFiles = true;
    private string _onnxSelector = "policy.onnx";
    private string _vecNormSelector = "*vecnorm*.json";
    private string _linearSelector = "policy_linear.json";

    private DateTime _lastVecNormWriteUtc = DateTime.MinValue;
    private DateTime _lastPolicyWriteUtc = DateTime.MinValue;
    private DateTime _lastOnnxWriteUtc = DateTime.MinValue;

    private VecNormalizeStats? _vecNorm;
    private LinearPolicy? _linearPolicy;
    private OnnxPolicy? _onnxPolicy;

    public bool HasLinearPolicy => _linearPolicy != null;
    public bool HasOnnxPolicy => _onnxPolicy != null;
    public bool HasVecNorm => _vecNorm != null;
    public string LastOnnxPath { get; private set; } = string.Empty;

    public void Configure(
        string exportDirPath,
        double pollIntervalSec,
        bool useLatestFiles,
        string onnxSelector,
        string vecNormSelector,
        string linearSelector
    )
    {
        _exportDirPath = exportDirPath;
        _pollIntervalSec = Math.Max(0.1, pollIntervalSec);
        _useLatestFiles = useLatestFiles;
        _onnxSelector = string.IsNullOrWhiteSpace(onnxSelector) ? "policy.onnx" : onnxSelector;
        _vecNormSelector = string.IsNullOrWhiteSpace(vecNormSelector) ? "*vecnorm*.json" : vecNormSelector;
        _linearSelector = string.IsNullOrWhiteSpace(linearSelector) ? "policy_linear.json" : linearSelector;
    }

    public void Update(double delta)
    {
        if (string.IsNullOrWhiteSpace(_exportDirPath))
        {
            return;
        }

        _pollAccumSec += delta;
        if (_pollAccumSec < _pollIntervalSec)
        {
            return;
        }
        _pollAccumSec = 0.0;

        ReloadVecNormIfNeeded();
        ReloadLinearPolicyIfNeeded();
        ReloadOnnxPolicyIfNeeded();
    }

    public bool TryInferAction(double[] observation, double[] outActions)
    {
        if (outActions.Length == 0)
        {
            return false;
        }

        double[] workingObs = new double[observation.Length];
        Array.Copy(observation, workingObs, observation.Length);

        if (_vecNorm != null)
        {
            _vecNorm.NormalizeInPlace(workingObs);
        }

        if (_onnxPolicy != null)
        {
            _onnxPolicy.Infer(workingObs, outActions);
            return true;
        }

        if (_linearPolicy != null)
        {
            _linearPolicy.Infer(workingObs, outActions);
            return true;
        }

        return false;
    }

    private void ReloadVecNormIfNeeded()
    {
        string vecNormPath = ResolveSelectedFile(_vecNormSelector, "*vecnorm*.json");
        if (string.IsNullOrWhiteSpace(vecNormPath))
        {
            return;
        }
        if (!File.Exists(vecNormPath))
        {
            return;
        }

        DateTime writeTime = File.GetLastWriteTimeUtc(vecNormPath);
        if (writeTime <= _lastVecNormWriteUtc)
        {
            return;
        }

        if (VecNormalizeStats.TryLoad(vecNormPath, out VecNormalizeStats stats, out string error))
        {
            _vecNorm = stats;
            _lastVecNormWriteUtc = writeTime;
            GD.Print("Reloaded VecNormalize stats: " + vecNormPath);
        }
        else
        {
            GD.PushWarning("VecNormalize reload failed: " + error);
        }
    }

    private void ReloadLinearPolicyIfNeeded()
    {
        string policyPath = ResolveSelectedFile(_linearSelector, "policy_linear*.json");
        if (string.IsNullOrWhiteSpace(policyPath))
        {
            return;
        }
        if (!File.Exists(policyPath))
        {
            return;
        }

        DateTime writeTime = File.GetLastWriteTimeUtc(policyPath);
        if (writeTime <= _lastPolicyWriteUtc)
        {
            return;
        }

        if (LinearPolicy.TryLoad(policyPath, out LinearPolicy policy, out string error))
        {
            _linearPolicy = policy;
            _lastPolicyWriteUtc = writeTime;
            GD.Print("Reloaded linear policy: " + policyPath);
        }
        else
        {
            GD.PushWarning("Linear policy reload failed: " + error);
        }
    }

    private void ReloadOnnxPolicyIfNeeded()
    {
        string onnxPath = ResolveSelectedFile(_onnxSelector, "*.onnx");
        if (string.IsNullOrWhiteSpace(onnxPath))
        {
            return;
        }
        if (!File.Exists(onnxPath))
        {
            return;
        }

        DateTime writeTime = File.GetLastWriteTimeUtc(onnxPath);
        if (writeTime <= _lastOnnxWriteUtc)
        {
            return;
        }

        _lastOnnxWriteUtc = writeTime;
        LastOnnxPath = onnxPath;

        if (OnnxPolicy.TryLoad(onnxPath, out OnnxPolicy? policy, out string error))
        {
            _onnxPolicy?.Dispose();
            _onnxPolicy = policy;
            GD.Print("Reloaded ONNX policy: " + onnxPath);
        }
        else
        {
            GD.PushWarning("ONNX policy reload failed: " + error);
        }
    }

    public void Dispose()
    {
        _onnxPolicy?.Dispose();
        _onnxPolicy = null;
    }

    private string ResolveSelectedFile(string selector, string fallbackPattern)
    {
        if (!_useLatestFiles)
        {
            return Path.Combine(_exportDirPath, selector);
        }

        string pattern = string.IsNullOrWhiteSpace(selector) ? fallbackPattern : selector;
        string[] matches = Directory.GetFiles(_exportDirPath, pattern, SearchOption.TopDirectoryOnly);
        if (matches.Length == 0)
        {
            return string.Empty;
        }

        Array.Sort(matches, (a, b) => File.GetLastWriteTimeUtc(b).CompareTo(File.GetLastWriteTimeUtc(a)));
        return matches[0];
    }

    private sealed class LinearPolicy
    {
        private readonly double[][] _weights;
        private readonly double[] _bias;
        private readonly double _clipAction;

        private LinearPolicy(double[][] weights, double[] bias, double clipAction)
        {
            _weights = weights;
            _bias = bias;
            _clipAction = clipAction;
        }

        public static bool TryLoad(string filePath, out LinearPolicy policy, out string error)
        {
            policy = new LinearPolicy(Array.Empty<double[]>(), Array.Empty<double>(), 1.0);
            error = string.Empty;

            try
            {
                string json = File.ReadAllText(filePath);
                using JsonDocument doc = JsonDocument.Parse(json);
                JsonElement root = doc.RootElement;

                JsonElement weightsElem = root.GetProperty("weights");
                JsonElement biasElem = root.GetProperty("bias");

                if (weightsElem.ValueKind != JsonValueKind.Array || biasElem.ValueKind != JsonValueKind.Array)
                {
                    error = "weights/bias must be arrays";
                    return false;
                }

                int actionCount = weightsElem.GetArrayLength();
                if (actionCount == 0)
                {
                    error = "weights must contain at least one action row";
                    return false;
                }

                double[][] weights = new double[actionCount][];
                for (int a = 0; a < actionCount; a++)
                {
                    JsonElement row = weightsElem[a];
                    if (row.ValueKind != JsonValueKind.Array)
                    {
                        error = "each weights row must be an array";
                        return false;
                    }
                    weights[a] = new double[row.GetArrayLength()];
                    for (int i = 0; i < weights[a].Length; i++)
                    {
                        weights[a][i] = row[i].GetDouble();
                    }
                }

                double[] bias = new double[biasElem.GetArrayLength()];
                for (int i = 0; i < bias.Length; i++)
                {
                    bias[i] = biasElem[i].GetDouble();
                }

                if (bias.Length != actionCount)
                {
                    error = "bias length must match weights action rows";
                    return false;
                }

                double clipAction = 1.0;
                if (root.TryGetProperty("clip_action", out JsonElement clipElem))
                {
                    clipAction = Math.Max(0.0, clipElem.GetDouble());
                }

                policy = new LinearPolicy(weights, bias, clipAction);
                return true;
            }
            catch (Exception ex)
            {
                error = "policy parse failed: " + ex.Message;
                return false;
            }
        }

        public void Infer(double[] observation, double[] outActions)
        {
            int actionCount = Math.Min(outActions.Length, _weights.Length);
            for (int a = 0; a < actionCount; a++)
            {
                double sum = _bias[a];
                int obsCount = Math.Min(observation.Length, _weights[a].Length);
                for (int i = 0; i < obsCount; i++)
                {
                    sum += _weights[a][i] * observation[i];
                }

                double squashed = Math.Tanh(sum);
                if (squashed > _clipAction)
                {
                    squashed = _clipAction;
                }
                else if (squashed < -_clipAction)
                {
                    squashed = -_clipAction;
                }
                outActions[a] = squashed;
            }
        }
    }
}
