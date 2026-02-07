using System;
using System.Collections.Generic;
using Godot;

public sealed class MjCreatureTrainerBridge : IDisposable
{
    private readonly List<MjCreatureRuntime> _creatures = new List<MjCreatureRuntime>();
    private readonly List<Vector3> _lastPositions = new List<Vector3>();

    public int CreatureCount => _creatures.Count;

    public int GetBodyCount(int creatureIndex)
    {
        if (creatureIndex < 0 || creatureIndex >= _creatures.Count)
        {
            return 0;
        }
        return Math.Max(0, _creatures[creatureIndex].BodyCount);
    }

    public int GetActionSize(int creatureIndex)
    {
        if (creatureIndex < 0 || creatureIndex >= _creatures.Count)
        {
            return 0;
        }
        return _creatures[creatureIndex].ActionSize;
    }

    public bool InitializeCreatures(int count, string modelPath, string trackedBodyName, int observationSize)
    {
        Dispose();

        int safeCount = Math.Max(1, count);
        for (int i = 0; i < safeCount; i++)
        {
            var creature = new MjCreatureRuntime(observationSize);
            if (!creature.Initialize(modelPath, trackedBodyName))
            {
                creature.Dispose();
                Dispose();
                return false;
            }

            int resetRc = creature.Reset();
            if (resetRc != 0)
            {
                GD.PushError("Failed to reset creature " + i + ": " + resetRc + " / " + MujocoNative.LastError());
                creature.Dispose();
                Dispose();
                return false;
            }

            _creatures.Add(creature);
            _lastPositions.Add(creature.LastRootPosition);
        }

        return true;
    }

    public void SetAction(int creatureIndex, int actionIndex, double value)
    {
        if (creatureIndex < 0 || creatureIndex >= _creatures.Count)
        {
            return;
        }
        _creatures[creatureIndex].SetAction(actionIndex, value);
    }

    public int StepCreature(int creatureIndex, int stepsPerTick)
    {
        if (creatureIndex < 0 || creatureIndex >= _creatures.Count)
        {
            return 1;
        }

        int rc = _creatures[creatureIndex].Step(stepsPerTick);
        if (rc != 0)
        {
            return rc;
        }

        if (_creatures[creatureIndex].TryGetRootPosition(out Vector3 position))
        {
            _lastPositions[creatureIndex] = position;
        }
        return 0;
    }

    public int FillObservation(int creatureIndex, double[] destination)
    {
        if (creatureIndex < 0 || creatureIndex >= _creatures.Count)
        {
            return 1;
        }
        return _creatures[creatureIndex].FillObservation(destination);
    }

    public bool TryGetRootPosition(int creatureIndex, out Vector3 position)
    {
        position = Vector3.Zero;
        if (creatureIndex < 0 || creatureIndex >= _creatures.Count)
        {
            return false;
        }
        position = _lastPositions[creatureIndex];
        return true;
    }

    public bool TryGetBodyPosition(int creatureIndex, int bodyIndex, out Vector3 position)
    {
        position = Vector3.Zero;
        if (creatureIndex < 0 || creatureIndex >= _creatures.Count)
        {
            return false;
        }
        return _creatures[creatureIndex].TryGetBodyPosition(bodyIndex, out position);
    }

    public double ComputeRewardForwardX(int creatureIndex)
    {
        if (creatureIndex < 0 || creatureIndex >= _creatures.Count)
        {
            return 0.0;
        }

        Vector3 previous = _lastPositions[creatureIndex];
        if (!_creatures[creatureIndex].TryGetRootPosition(out Vector3 current))
        {
            return 0.0;
        }

        _lastPositions[creatureIndex] = current;
        return current.X - previous.X;
    }

    public bool IsTerminated(int creatureIndex, float minHeight)
    {
        if (creatureIndex < 0 || creatureIndex >= _creatures.Count)
        {
            return true;
        }

        if (!_creatures[creatureIndex].TryGetRootPosition(out Vector3 current))
        {
            return true;
        }

        _lastPositions[creatureIndex] = current;
        return current.Z < minHeight;
    }

    public int ResetCreature(int creatureIndex)
    {
        if (creatureIndex < 0 || creatureIndex >= _creatures.Count)
        {
            return 1;
        }

        int rc = _creatures[creatureIndex].Reset();
        if (rc == 0)
        {
            _lastPositions[creatureIndex] = _creatures[creatureIndex].LastRootPosition;
        }
        return rc;
    }

    public void Dispose()
    {
        foreach (var creature in _creatures)
        {
            creature.Dispose();
        }
        _creatures.Clear();
        _lastPositions.Clear();
    }
}
