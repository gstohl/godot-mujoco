using System;
using System.Collections.Generic;
using System.IO;
using Godot;

public partial class MjCreatureManager : Node3D
{
    [Export]
    public string ModelPath = "/Users/shnidi/claude/robots/AI-orchestrator/creatures/0001/assets/mujoco/octopod.xml";

    [Export]
    public string TrackedBodyName = "torso";

    [Export]
    public int CreatureCount = 1;

    [Export]
    public float CreatureSpacing = 2.0f;

    [Export]
    public int StepsPerTick = 1;

    [Export]
    public float ActionAmplitude = 0.5f;

    [Export]
    public float ActionFrequencyHz = 0.7f;

    [Export]
    public float TerminationMinHeight = 0.2f;

    [Export]
    public string PolicyExportDir = "/Users/shnidi/claude/robots/AI-orchestrator/data/runs/run-1770436362-0001-refine-4b11/artifacts/checkpoints/left";

    [Export]
    public float PolicyPollIntervalSec = 0.5f;

    [Export]
    public bool UseLatestPolicyFiles = true;

    [Export]
    public string OnnxSelector = "*.onnx";

    [Export]
    public string VecNormSelector = "*vecnorm*.json";

    [Export]
    public string LinearSelector = "policy_linear*.json";

    private readonly MjCreatureTrainerBridge _trainer = new MjCreatureTrainerBridge();
    private readonly MjPolicyHotReloader _policyReloader = new MjPolicyHotReloader();
    private readonly List<Node3D> _creatureVisuals = new List<Node3D>();
    private readonly List<List<MeshInstance3D>> _bodyVisuals = new List<List<MeshInstance3D>>();
    private double[] _observationBuffer = new double[1];
    private double[] _actionBuffer = new double[1];
    private double _elapsed;

    public override void _Ready()
    {
        ConfigureView();

        int count = Math.Max(1, CreatureCount);
        string exportDirAbsolutePath = Path.IsPathRooted(PolicyExportDir)
            ? PolicyExportDir
            : ProjectSettings.GlobalizePath(PolicyExportDir);
        Directory.CreateDirectory(exportDirAbsolutePath);

        int observationSize = ResolveObservationSizeFromVecNorm(exportDirAbsolutePath, VecNormSelector);
        _observationBuffer = new double[Math.Max(1, observationSize)];

        bool ok = _trainer.InitializeCreatures(
            count,
            ModelPath,
            TrackedBodyName,
            observationSize: _observationBuffer.Length
        );
        if (!ok)
        {
            GD.PushError("Failed to initialize creature trainer bridge.");
            return;
        }

        _policyReloader.Configure(
            exportDirAbsolutePath,
            PolicyPollIntervalSec,
            UseLatestPolicyFiles,
            OnnxSelector,
            VecNormSelector,
            LinearSelector
        );

        for (int i = 0; i < _trainer.CreatureCount; i++)
        {
            var marker = new Node3D();
            marker.Name = "Creature_" + i;

            var bodyMesh = new MeshInstance3D();
            bodyMesh.Name = "Body";
            bodyMesh.Mesh = new SphereMesh
            {
                Radius = 0.18f,
                Height = 0.36f,
            };

            var material = new StandardMaterial3D();
            material.AlbedoColor = new Color(0.1f, 0.8f, 0.9f, 1.0f);
            material.EmissionEnabled = true;
            material.Emission = new Color(0.02f, 0.15f, 0.2f, 1.0f);
            bodyMesh.MaterialOverride = material;
            marker.AddChild(bodyMesh);

            int bodyCount = _trainer.GetBodyCount(i);
            var bodyMarkers = new List<MeshInstance3D>();
            for (int bodyIndex = 1; bodyIndex < bodyCount; bodyIndex++)
            {
                var jointMarker = new MeshInstance3D();
                jointMarker.Name = "Body_" + bodyIndex;
                jointMarker.Mesh = new SphereMesh
                {
                    Radius = 0.035f,
                    Height = 0.07f,
                };

                var jointMaterial = new StandardMaterial3D();
                jointMaterial.AlbedoColor = new Color(0.95f, 0.92f, 0.35f, 1.0f);
                jointMaterial.EmissionEnabled = true;
                jointMaterial.Emission = new Color(0.12f, 0.1f, 0.02f, 1.0f);
                jointMarker.MaterialOverride = jointMaterial;

                marker.AddChild(jointMarker);
                bodyMarkers.Add(jointMarker);
            }

            AddChild(marker);

            _creatureVisuals.Add(marker);
            _bodyVisuals.Add(bodyMarkers);
        }

        int actionSize = Math.Max(1, _trainer.GetActionSize(0));
        _actionBuffer = new double[actionSize];

        GD.Print("Creatures initialized: " + _trainer.CreatureCount);
    }

    public override void _PhysicsProcess(double delta)
    {
        _elapsed += delta;
        _policyReloader.Update(delta);

        for (int i = 0; i < _trainer.CreatureCount; i++)
        {
            var marker = _creatureVisuals[i];

            if (_trainer.FillObservation(i, _observationBuffer) != 0)
            {
                GD.PushWarning("Observation fetch failed for creature " + i + " / " + MujocoNative.LastError());
            }

            bool hasHotPolicy = _policyReloader.TryInferAction(_observationBuffer, _actionBuffer);
            if (hasHotPolicy)
            {
                for (int actionIndex = 0; actionIndex < _actionBuffer.Length; actionIndex++)
                {
                    _trainer.SetAction(i, actionIndex, _actionBuffer[actionIndex]);
                }
            }
            else
            {
                for (int actionIndex = 0; actionIndex < _actionBuffer.Length; actionIndex++)
                {
                    double phase = (2.0 * Math.PI * ActionFrequencyHz * _elapsed) + (i * 0.8) + (actionIndex * 0.1);
                    _trainer.SetAction(i, actionIndex, ActionAmplitude * Math.Sin(phase));
                }
            }

            int rc = _trainer.StepCreature(i, StepsPerTick);
            if (rc != 0)
            {
                GD.PushWarning("Creature step failed: " + rc + " / " + MujocoNative.LastError());
                continue;
            }

            bool hasRoot = _trainer.TryGetRootPosition(i, out Vector3 rootPosition);
            if (hasRoot)
            {
                marker.Position = new Vector3(rootPosition.X + i * CreatureSpacing, rootPosition.Y, rootPosition.Z);
            }

            List<MeshInstance3D> bodyMarkers = _bodyVisuals[i];
            for (int bodyIndex = 1; bodyIndex <= bodyMarkers.Count; bodyIndex++)
            {
                if (hasRoot && _trainer.TryGetBodyPosition(i, bodyIndex, out Vector3 bodyPosition))
                {
                    bodyMarkers[bodyIndex - 1].Position = bodyPosition - rootPosition;
                }
            }

            double reward = _trainer.ComputeRewardForwardX(i);
            if (_trainer.IsTerminated(i, TerminationMinHeight))
            {
                int resetRc = _trainer.ResetCreature(i);
                if (resetRc != 0)
                {
                    GD.PushWarning("Creature reset failed: " + resetRc + " / " + MujocoNative.LastError());
                }
            }

            if ((i == 0) && (_elapsed % 1.0 < delta))
            {
                GD.Print("Creature 0 reward_x=" + reward + " obs0=" + _observationBuffer[0] +
                         " hot_policy=" + hasHotPolicy + " onnx=" + _policyReloader.LastOnnxPath);
            }
        }
    }

    public override void _ExitTree()
    {
        _policyReloader.Dispose();
        _trainer.Dispose();
        _creatureVisuals.Clear();
        _bodyVisuals.Clear();
    }

    private void ConfigureView()
    {
        Camera3D? camera = GetNodeOrNull<Camera3D>("Camera3D");
        if (camera != null)
        {
            camera.Position = new Vector3(0.0f, -5.2f, 3.6f);
            camera.LookAt(new Vector3(0.0f, 0.0f, 0.6f), Vector3.Up);
            camera.Fov = 65.0f;
        }

        DirectionalLight3D? light = GetNodeOrNull<DirectionalLight3D>("DirectionalLight3D");
        if (light != null)
        {
            light.RotationDegrees = new Vector3(-50.0f, 20.0f, 0.0f);
            light.LightEnergy = 2.0f;
        }
    }

    private static int ResolveObservationSizeFromVecNorm(string exportDirAbsolutePath, string selector)
    {
        string pattern = string.IsNullOrWhiteSpace(selector) ? "*vecnorm*.json" : selector;
        string[] candidates = Directory.GetFiles(exportDirAbsolutePath, pattern, SearchOption.TopDirectoryOnly);
        if (candidates.Length == 0)
        {
            return 1;
        }

        Array.Sort(candidates, (a, b) => File.GetLastWriteTimeUtc(b).CompareTo(File.GetLastWriteTimeUtc(a)));
        if (VecNormalizeStats.TryLoad(candidates[0], out VecNormalizeStats stats, out string _))
        {
            return Math.Max(1, stats.ObsMean.Length);
        }

        return 1;
    }

}
