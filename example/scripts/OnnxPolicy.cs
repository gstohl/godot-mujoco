using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.ML.OnnxRuntime;
using Microsoft.ML.OnnxRuntime.Tensors;

public sealed class OnnxPolicy : IDisposable
{
    private readonly InferenceSession _session;
    private readonly string _inputName;
    private readonly string _outputName;
    private readonly float _clipAction;
    private readonly int _expectedObsSize;

    private OnnxPolicy(InferenceSession session, string inputName, string outputName, float clipAction,
                       int expectedObsSize)
    {
        _session = session;
        _inputName = inputName;
        _outputName = outputName;
        _clipAction = clipAction;
        _expectedObsSize = expectedObsSize;
    }

    public static bool TryLoad(string modelPath, out OnnxPolicy? policy, out string error, float clipAction = 1.0f)
    {
        policy = null;
        error = string.Empty;

        try
        {
            var sessionOptions = new SessionOptions();
            var session = new InferenceSession(modelPath, sessionOptions);

            string? input = session.InputMetadata.Keys.FirstOrDefault();
            string? output = session.OutputMetadata.Keys.FirstOrDefault();
            if (string.IsNullOrWhiteSpace(input) || string.IsNullOrWhiteSpace(output))
            {
                session.Dispose();
                error = "ONNX model has no usable input/output metadata";
                return false;
            }

            int expectedObsSize = 0;

            policy = new OnnxPolicy(session, input, output, MathF.Max(0.0f, clipAction), expectedObsSize);
            return true;
        }
        catch (Exception ex)
        {
            error = "Failed to load ONNX model: " + ex.Message;
            return false;
        }
    }

    public void Infer(double[] observation, double[] outActions)
    {
        if (outActions.Length == 0)
        {
            return;
        }

        int obsSize = _expectedObsSize > 0 ? _expectedObsSize : observation.Length;
        float[] obs = new float[obsSize];
        int copyCount = Math.Min(observation.Length, obsSize);
        for (int i = 0; i < copyCount; i++)
        {
            obs[i] = (float)observation[i];
        }

        var tensor = new DenseTensor<float>(obs, new[] { 1, obs.Length });
        var inputs = new List<NamedOnnxValue>
        {
            NamedOnnxValue.CreateFromTensor(_inputName, tensor),
        };

        try
        {
            using IDisposableReadOnlyCollection<DisposableNamedOnnxValue> results = _session.Run(inputs);
            DisposableNamedOnnxValue? named = results.FirstOrDefault(r => r.Name == _outputName) ?? results.FirstOrDefault();
            if (named == null)
            {
                return;
            }

            float[] raw = named.AsEnumerable<float>().ToArray();
            int count = Math.Min(outActions.Length, raw.Length);
            for (int i = 0; i < count; i++)
            {
                double value = raw[i];
                if (value > _clipAction)
                {
                    value = _clipAction;
                }
                else if (value < -_clipAction)
                {
                    value = -_clipAction;
                }
                outActions[i] = value;
            }
        }
        catch
        {
            Array.Clear(outActions, 0, outActions.Length);
        }
    }

    public void Dispose()
    {
        _session.Dispose();
    }
}
