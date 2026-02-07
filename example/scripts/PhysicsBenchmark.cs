using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Text;
using System.Threading.Tasks;
using Godot;

public partial class PhysicsBenchmark : Node3D
{
    private const int UncappedPhysicsTicksPerSecond = 20000;
    private const double TimeStep = 1.0 / 60.0;
    private const double BenchmarkDurationSec = 8.0;
    private static readonly int[] ObjectCounts = { 100, 1000, 10000 };
    private static readonly List<Shape3D> ShapeKeepAlive = new List<Shape3D>();

    public override async void _Ready()
    {
        GD.Print("=== Physics Benchmark (Uncapped) ===");
        GD.Print("dt=" + TimeStep + ", PhysicsTicksPerSecond=" + UncappedPhysicsTicksPerSecond +
                 ", duration=" + BenchmarkDurationSec + "s");

        var results = new List<BenchmarkResult>();
        foreach (int objectCount in ObjectCounts)
        {
            GD.Print("--- Running " + objectCount + " spheres ---");

            double godotUncapped = await RunGodotPhysicsBenchmark(objectCount, BenchmarkDurationSec, UncappedPhysicsTicksPerSecond);
            double mujocoUncapped = RunMujocoBenchmark(objectCount, BenchmarkDurationSec);
            results.Add(new BenchmarkResult(objectCount, BenchmarkDurationSec, godotUncapped, mujocoUncapped));
        }

        GD.Print("--- Results ---");
        foreach (BenchmarkResult result in results)
        {
            string godotText = result.GodotStepsPerSecond > 0.0 ? result.GodotStepsPerSecond.ToString("F2") : "N/A";
            string ratioText = result.GodotStepsPerSecond > 0.0 ? result.Ratio.ToString("F2") + "x" : "N/A";
            GD.Print(result.ObjectCount + " spheres | Godot=" + godotText +
                     " | MuJoCo=" + result.MujocoStepsPerSecond.ToString("F2") +
                     " | ratio=" + ratioText +
                     " | duration=" + result.DurationSec.ToString("F1") + "s");
        }

        GetTree().Quit();
    }

    private async Task<double> RunGodotPhysicsBenchmark(int objectCount, double durationSec, int physicsTicksPerSecond)
    {
        Engine.PhysicsTicksPerSecond = physicsTicksPerSecond;

        var root = new Node3D { Name = "GodotBenchmarkRoot" };
        AddChild(root);

        var floorBody = new StaticBody3D();
        var floorShapeNode = new CollisionShape3D();
        var floorShape = new BoxShape3D { Size = new Vector3(80.0f, 1.0f, 80.0f) };
        ShapeKeepAlive.Add(floorShape);
        floorShapeNode.Shape = floorShape;
        floorShapeNode.Position = new Vector3(0.0f, 0.0f, -0.5f);
        floorBody.AddChild(floorShapeNode);
        root.AddChild(floorBody);

        var sphereShape = new SphereShape3D { Radius = 0.1f };
        ShapeKeepAlive.Add(sphereShape);

        int side = (int)Math.Ceiling(Math.Pow(objectCount, 1.0 / 3.0));
        for (int i = 0; i < objectCount; i++)
        {
            int x = i % side;
            int y = (i / side) % side;
            int z = i / (side * side);

            var rb = new RigidBody3D();
            rb.Position = new Vector3(
                (x - side / 2) * 0.30f,
                (y - side / 2) * 0.30f,
                1.2f + (z * 0.24f)
            );

            var cs = new CollisionShape3D();
            cs.Shape = sphereShape;
            rb.AddChild(cs);
            root.AddChild(rb);
        }

        await ToSignal(GetTree(), SceneTree.SignalName.PhysicsFrame);
        await ToSignal(GetTree(), SceneTree.SignalName.PhysicsFrame);

        int executedSteps = 0;
        var sw = Stopwatch.StartNew();
        while (sw.Elapsed.TotalSeconds < durationSec)
        {
            await ToSignal(GetTree(), SceneTree.SignalName.PhysicsFrame);
            executedSteps++;
        }
        sw.Stop();

        root.Free();
        return executedSteps / sw.Elapsed.TotalSeconds;
    }

    private double RunMujocoBenchmark(int objectCount, double durationSec)
    {
        string xmlPath = WriteMujocoBenchmarkXml(objectCount);
        byte[] errorBuffer = MujocoNative.CreateErrorBuffer();

        IntPtr model = MujocoNative.gmj_model_load_xml(xmlPath, errorBuffer, (UIntPtr)errorBuffer.Length);
        if (model == IntPtr.Zero)
        {
            GD.PushError("MuJoCo benchmark load failed: " + MujocoNative.LastError());
            return 0.0;
        }

        IntPtr data = MujocoNative.gmj_data_create(model);
        if (data == IntPtr.Zero)
        {
            GD.PushError("MuJoCo benchmark data create failed: " + MujocoNative.LastError());
            MujocoNative.gmj_model_free(model);
            return 0.0;
        }

        int executedSteps = 0;
        var sw = Stopwatch.StartNew();
        while (sw.Elapsed.TotalSeconds < durationSec)
        {
            int rc = MujocoNative.gmj_step(model, data, 1);
            if (rc != 0)
            {
                GD.PushWarning("MuJoCo step error at step " + executedSteps + ": " + rc + " / " + MujocoNative.LastError());
                break;
            }
            executedSteps++;
        }
        sw.Stop();

        MujocoNative.gmj_data_free(data);
        MujocoNative.gmj_model_free(model);
        return executedSteps / sw.Elapsed.TotalSeconds;
    }

    private static string WriteMujocoBenchmarkXml(int objectCount)
    {
        string path = ProjectSettings.GlobalizePath("user://mujoco_benchmark_" + objectCount + ".xml");
        if (!File.Exists(path))
        {
            string xml = BuildMujocoXml(objectCount);
            File.WriteAllText(path, xml);
        }
        return path;
    }

    private static string BuildMujocoXml(int objectCount)
    {
        var sb = new StringBuilder();
        sb.AppendLine("<mujoco model=\"benchmark_" + objectCount + "\">");
        sb.AppendLine("  <option timestep=\"0.0166666667\" gravity=\"0 0 -9.81\"/>");
        sb.AppendLine("  <worldbody>");
        sb.AppendLine("    <geom type=\"plane\" size=\"80 80 0.1\"/>");

        int side = (int)Math.Ceiling(Math.Pow(objectCount, 1.0 / 3.0));
        for (int i = 0; i < objectCount; i++)
        {
            int x = i % side;
            int y = (i / side) % side;
            int z = i / (side * side);
            double px = (x - side / 2) * 0.30;
            double py = (y - side / 2) * 0.30;
            double pz = 1.2 + (z * 0.24);

            sb.AppendLine($"    <body pos=\"{px:F3} {py:F3} {pz:F3}\">");
            sb.AppendLine("      <freejoint/>");
            sb.AppendLine("      <geom type=\"sphere\" size=\"0.1\"/>");
            sb.AppendLine("    </body>");
        }

        sb.AppendLine("  </worldbody>");
        sb.AppendLine("</mujoco>");
        return sb.ToString();
    }

    private readonly struct BenchmarkResult
    {
        public BenchmarkResult(int objectCount, double durationSec, double godotStepsPerSecond, double mujocoStepsPerSecond)
        {
            ObjectCount = objectCount;
            DurationSec = durationSec;
            GodotStepsPerSecond = godotStepsPerSecond;
            MujocoStepsPerSecond = mujocoStepsPerSecond;
        }

        public int ObjectCount { get; }
        public double DurationSec { get; }
        public double GodotStepsPerSecond { get; }
        public double MujocoStepsPerSecond { get; }
        public double Ratio => GodotStepsPerSecond > 0.0 ? MujocoStepsPerSecond / GodotStepsPerSecond : 0.0;
    }
}
