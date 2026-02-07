using System;
using Godot;

public sealed class MjCreatureRuntime : IDisposable
{
    private readonly MjSceneRuntime _scene = new MjSceneRuntime();
    private double[] _actions = Array.Empty<double>();
    private readonly double[] _observationTemplate;
    private double[] _qposRead = Array.Empty<double>();
    private int _trackedBodyId = -1;
    private Vector3 _lastRootPosition = Vector3.Zero;

    public MjCreatureRuntime(int observationSize)
    {
        _observationTemplate = observationSize > 0 ? new double[observationSize] : Array.Empty<double>();
    }

    public bool IsReady => _scene.IsReady && _trackedBodyId >= 0;

    public int ActionSize => _actions.Length;
    public int ObservationSize => _observationTemplate.Length;
    public int BodyCount => _scene.Nbody;

    public bool Initialize(string modelPath, string trackedBodyName)
    {
        if (!_scene.Initialize(modelPath))
        {
            return false;
        }

        _trackedBodyId = _scene.ResolveBodyId(trackedBodyName);
        if (_trackedBodyId < 0)
        {
            GD.PushError("Creature body not found: " + trackedBodyName + " / " + MujocoNative.LastError());
            _scene.Dispose();
            return false;
        }

        int nu = _scene.Nu;
        _actions = nu > 0 ? new double[nu] : Array.Empty<double>();
        int nq = _scene.Nq;
        _qposRead = nq > 0 ? new double[nq] : Array.Empty<double>();

        return true;
    }

    public int Reset()
    {
        if (!IsReady)
        {
            return 1;
        }

        for (int i = 0; i < _actions.Length; i++)
        {
            _actions[i] = 0.0;
        }

        int rc = _scene.Reset();
        if (rc != 0)
        {
            return rc;
        }

        if (TryGetRootPosition(out Vector3 position))
        {
            _lastRootPosition = position;
        }

        return 0;
    }

    public void SetAction(int index, double value)
    {
        if (index < 0 || index >= _actions.Length)
        {
            return;
        }
        _actions[index] = value;
    }

    public int Step(int stepsPerTick)
    {
        if (!IsReady)
        {
            return 1;
        }

        if (_actions.Length > 0)
        {
            int actionRc = _scene.SetCtrlSlice(0, _actions);
            if (actionRc != 0)
            {
                return actionRc;
            }
        }

        return _scene.Step(stepsPerTick);
    }

    public bool TryGetRootPosition(out Vector3 position)
    {
        bool ok = _scene.TryGetBodyWorldPosition(_trackedBodyId, out position);
        if (ok)
        {
            _lastRootPosition = position;
        }
        return ok;
    }

    public Vector3 LastRootPosition => _lastRootPosition;

    public bool TryGetBodyPosition(int bodyIndex, out Vector3 position)
    {
        return _scene.TryGetBodyWorldPosition(bodyIndex, out position);
    }

    public int FillObservation(double[] destination)
    {
        if (!IsReady || destination == null)
        {
            return 1;
        }

        int sampleCount = Math.Min(destination.Length, _qposRead.Length);
        Array.Clear(destination, 0, destination.Length);
        if (sampleCount <= 0)
        {
            return 0;
        }

        int rc = _scene.GetQposSlice(0, _qposRead);
        if (rc != 0)
        {
            return rc;
        }

        Array.Copy(_qposRead, destination, sampleCount);
        return 0;
    }

    public void Dispose()
    {
        _scene.Dispose();
    }
}
