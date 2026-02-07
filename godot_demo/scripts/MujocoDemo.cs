using Godot;

public partial class MujocoDemo : Node3D
{
    [Export]
    public string ModelPath = "res://models/pendulum.xml";

    [Export]
    public string BodyName = "pendulum";

    [Export]
    public int StepsPerTick = 1;

    [Export]
    public double MotorControl = 0.0;

    private readonly MjSceneRuntime _runtime = new MjSceneRuntime();
    private int _bodyId = -1;
    private double[] _ctrlValues = System.Array.Empty<double>();

    public override void _Ready()
    {
        if (!_runtime.InitializeFromXmlPath(ModelPath))
        {
            return;
        }

        _bodyId = _runtime.ResolveBodyId(BodyName);
        if (_bodyId < 0)
        {
            GD.PushError("Could not resolve body name: " + BodyName + " / " + MujocoNative.LastError());
            return;
        }

        int nu = _runtime.Nu;
        _ctrlValues = nu > 0 ? new double[nu] : System.Array.Empty<double>();

        GD.Print("MuJoCo bridge initialized.");
    }

    public override void _PhysicsProcess(double delta)
    {
        if (!_runtime.IsReady)
        {
            return;
        }

        if (_ctrlValues.Length > 0)
        {
            _ctrlValues[0] = MotorControl;
            int writeRc = _runtime.SetCtrlSlice(0, _ctrlValues);
            if (writeRc != 0)
            {
                GD.PushWarning("set ctrl slice failed: " + writeRc + " / " + MujocoNative.LastError());
            }
        }

        int rc = _runtime.Step(StepsPerTick);
        if (rc != 0)
        {
            GD.PushWarning("gmj_step failed: " + rc + " / " + MujocoNative.LastError());
            return;
        }

        if (_runtime.TryGetBodyWorldPosition(_bodyId, out Vector3 position))
        {
            Position = position;
        }
    }

    public override void _ExitTree()
    {
        _runtime.Dispose();
    }
}
