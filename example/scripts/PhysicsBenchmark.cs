using System;
using System.Diagnostics;
using System.IO;
using System.Text;
using System.Threading.Tasks;
using Godot;

public partial class PhysicsBenchmark : Node3D
{
    private const int ObjectCount = 1000;
    private const int BenchmarkSteps = 600;
    private const double TimeStep = 1.0 / 60.0;

    public override async void _Ready()
    {
        GD.Print("=== Physics Benchmark (1000 objects) ===");
        GD.Print("Steps: " + BenchmarkSteps + " at dt=" + TimeStep);

        double godotRealtime = await RunGodotPhysicsBenchmark(60);
        double godotUncapped = await RunGodotPhysicsBenchmark(20000);
        double mujocoUncapped = RunMujocoBenchmark();

        GD.Print("--- Results ---");
        GD.Print("Godot physics steps/sec (60Hz mode): " + godotRealtime.ToString("F2"));
        GD.Print("Godot physics steps/sec (uncapped mode): " + godotUncapped.ToString("F2"));
        GD.Print("MuJoCo steps/sec (uncapped mode): " + mujocoUncapped.ToString("F2"));
        if (godotUncapped > 0.0)
        {
            GD.Print("MuJoCo/Godot ratio (uncapped): " + (mujocoUncapped / godotUncapped).ToString("F2") + "x");
        }

        GetTree().Quit();
    }

    private async Task<double> RunGodotPhysicsBenchmark(int physicsTicksPerSecond)
    {
        Engine.PhysicsTicksPerSecond = physicsTicksPerSecond;

        var root = new Node3D { Name = "GodotBenchmarkRoot" };
        AddChild(root);

        var floorBody = new StaticBody3D();
        var floorShapeNode = new CollisionShape3D();
        floorShapeNode.Shape = new BoxShape3D { Size = new Vector3(60.0f, 1.0f, 60.0f) };
        floorShapeNode.Position = new Vector3(0.0f, -0.5f, 0.0f);
        floorBody.AddChild(floorShapeNode);
        root.AddChild(floorBody);

        var sphereShape = new SphereShape3D { Radius = 0.1f };
        int side = (int)Math.Ceiling(Math.Sqrt(ObjectCount));
        for (int i = 0; i < ObjectCount; i++)
        {
            int x = i % side;
            int y = i / side;

            var rb = new RigidBody3D();
            rb.Position = new Vector3((x - side / 2) * 0.24f, 1.5f + (y * 0.015f), 0.0f);

            var cs = new CollisionShape3D();
            cs.Shape = sphereShape;
            rb.AddChild(cs);
            root.AddChild(rb);
        }

        await ToSignal(GetTree(), SceneTree.SignalName.PhysicsFrame);
        await ToSignal(GetTree(), SceneTree.SignalName.PhysicsFrame);

        var sw = Stopwatch.StartNew();
        for (int i = 0; i < BenchmarkSteps; i++)
        {
            await ToSignal(GetTree(), SceneTree.SignalName.PhysicsFrame);
        }
        sw.Stop();

        root.QueueFree();

        return BenchmarkSteps / sw.Elapsed.TotalSeconds;
    }

    private double RunMujocoBenchmark()
    {
        string xmlPath = WriteMujocoBenchmarkXml();
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

        var sw = Stopwatch.StartNew();
        for (int i = 0; i < BenchmarkSteps; i++)
        {
            int rc = MujocoNative.gmj_step(model, data, 1);
            if (rc != 0)
            {
                GD.PushWarning("MuJoCo step error at step " + i + ": " + rc + " / " + MujocoNative.LastError());
                break;
            }
        }
        sw.Stop();

        MujocoNative.gmj_data_free(data);
        MujocoNative.gmj_model_free(model);

        return BenchmarkSteps / sw.Elapsed.TotalSeconds;
    }

    private static string WriteMujocoBenchmarkXml()
    {
        string xml = BuildMujocoXml();
        string path = ProjectSettings.GlobalizePath("user://mujoco_benchmark_1000.xml");
        File.WriteAllText(path, xml);
        return path;
    }

    private static string BuildMujocoXml()
    {
        var sb = new StringBuilder();
        sb.AppendLine("<mujoco model=\"benchmark_1000\">");
        sb.AppendLine("  <option timestep=\"0.0166666667\" gravity=\"0 0 -9.81\"/>");
        sb.AppendLine("  <worldbody>");
        sb.AppendLine("    <geom name=\"floor\" type=\"plane\" size=\"50 50 0.1\"/>");

        int side = (int)Math.Ceiling(Math.Sqrt(ObjectCount));
        for (int i = 0; i < ObjectCount; i++)
        {
            int x = i % side;
            int y = i / side;
            double px = (x - side / 2) * 0.24;
            double py = 0.0;
            double pz = 1.5 + (y * 0.015);

            sb.AppendLine($"    <body name=\"b{i}\" pos=\"{px:F5} {py:F5} {pz:F5}\">");
            sb.AppendLine("      <freejoint/>");
            sb.AppendLine("      <geom type=\"sphere\" size=\"0.1\" density=\"500\"/>");
            sb.AppendLine("    </body>");
        }

        sb.AppendLine("  </worldbody>");
        sb.AppendLine("</mujoco>");
        return sb.ToString();
    }
}
