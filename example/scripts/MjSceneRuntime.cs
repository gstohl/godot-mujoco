using System;
using System.IO;
using System.Text;
using Godot;

public sealed class MjSceneRuntime : IDisposable
{
    public IntPtr ModelHandle { get; private set; }
    public IntPtr DataHandle { get; private set; }

    public bool IsReady => ModelHandle != IntPtr.Zero && DataHandle != IntPtr.Zero;

    public int Nq => IsReady ? MujocoNative.gmj_nq(ModelHandle) : -1;
    public int Nv => IsReady ? MujocoNative.gmj_nv(ModelHandle) : -1;
    public int Nu => IsReady ? MujocoNative.gmj_nu(ModelHandle) : -1;
    public int Nbody => IsReady ? MujocoNative.gmj_nbody(ModelHandle) : -1;

    public bool Initialize(string modelPath)
    {
        Dispose();

        byte[] errorBuffer = MujocoNative.CreateErrorBuffer();
        string xmlAbsolutePath = Path.IsPathRooted(modelPath)
            ? modelPath
            : ProjectSettings.GlobalizePath(modelPath);

        ModelHandle = MujocoNative.gmj_model_load_xml(xmlAbsolutePath, errorBuffer, (UIntPtr)errorBuffer.Length);
        if (ModelHandle == IntPtr.Zero)
        {
            string fromBuffer = Encoding.UTF8.GetString(errorBuffer).TrimEnd('\0');
            GD.PushError("Failed to load model: " + fromBuffer + " / " + MujocoNative.LastError());
            return false;
        }

        DataHandle = MujocoNative.gmj_data_create(ModelHandle);
        if (DataHandle == IntPtr.Zero)
        {
            GD.PushError("Failed to create simulation data: " + MujocoNative.LastError());
            MujocoNative.gmj_model_free(ModelHandle);
            ModelHandle = IntPtr.Zero;
            return false;
        }

        return true;
    }

    public int ResolveBodyId(string bodyName)
    {
        if (!IsReady)
        {
            return -1;
        }
        return MujocoNative.gmj_body_id(ModelHandle, bodyName);
    }

    public int Step(int steps)
    {
        if (!IsReady)
        {
            return 1;
        }
        return MujocoNative.gmj_step(ModelHandle, DataHandle, Math.Max(1, steps));
    }

    public int Reset()
    {
        if (!IsReady)
        {
            return 1;
        }
        return MujocoNative.gmj_reset_data(ModelHandle, DataHandle);
    }

    public int SetCtrlSlice(int startIndex, double[] values)
    {
        if (!IsReady || values == null)
        {
            return 1;
        }
        return MujocoNative.gmj_set_ctrl_slice(ModelHandle, DataHandle, startIndex, values.Length, values);
    }

    public int GetQposSlice(int startIndex, double[] destination)
    {
        if (!IsReady || destination == null)
        {
            return 1;
        }
        return MujocoNative.gmj_get_qpos_slice(ModelHandle, DataHandle, startIndex, destination.Length, destination);
    }

    public bool TryGetBodyWorldPosition(int bodyId, out Vector3 position)
    {
        position = Vector3.Zero;
        if (!IsReady)
        {
            return false;
        }

        double[] xyz = new double[3];
        int rc = MujocoNative.gmj_body_world_position(ModelHandle, DataHandle, bodyId, xyz);
        if (rc != 0)
        {
            return false;
        }

        position = new Vector3((float)xyz[0], (float)xyz[1], (float)xyz[2]);
        return true;
    }

    public void Dispose()
    {
        if (DataHandle != IntPtr.Zero)
        {
            MujocoNative.gmj_data_free(DataHandle);
            DataHandle = IntPtr.Zero;
        }
        if (ModelHandle != IntPtr.Zero)
        {
            MujocoNative.gmj_model_free(ModelHandle);
            ModelHandle = IntPtr.Zero;
        }
    }
}
