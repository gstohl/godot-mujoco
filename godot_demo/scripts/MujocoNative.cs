using System;
using System.Reflection;
using System.Runtime.InteropServices;
using Godot;

public static class MujocoNative
{
    private const string LibraryName = "godot_mujoco_bridge";
    private const int ErrorBufferBytes = 1024;

    static MujocoNative()
    {
        NativeLibrary.SetDllImportResolver(
            Assembly.GetExecutingAssembly(),
            ResolveLibrary
        );
    }

    private static IntPtr ResolveLibrary(string libraryName, Assembly assembly, DllImportSearchPath? searchPath)
    {
        string extension;
        if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
        {
            extension = ".dll";
        }
        else if (RuntimeInformation.IsOSPlatform(OSPlatform.OSX))
        {
            extension = ".dylib";
        }
        else
        {
            extension = ".so";
        }

        if (libraryName != LibraryName)
        {
            return IntPtr.Zero;
        }

        string fileName = RuntimeInformation.IsOSPlatform(OSPlatform.Windows)
            ? LibraryName + extension
            : "lib" + LibraryName + extension;

        string absolutePath = ProjectSettings.GlobalizePath("res://bin/" + fileName);
        if (NativeLibrary.TryLoad(absolutePath, out IntPtr handle))
        {
            return handle;
        }

        if (NativeLibrary.TryLoad(libraryName, out handle))
        {
            return handle;
        }

        throw new DllNotFoundException("Could not load native bridge: " + absolutePath);
    }

    [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
    public static extern IntPtr gmj_model_load_xml(
        [MarshalAs(UnmanagedType.LPUTF8Str)] string xmlPath,
        byte[] errorBuffer,
        UIntPtr errorBufferSize
    );

    [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void gmj_model_free(IntPtr model);

    [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
    public static extern IntPtr gmj_data_create(IntPtr model);

    [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void gmj_data_free(IntPtr data);

    [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int gmj_step(IntPtr model, IntPtr data, int steps);

    [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int gmj_nq(IntPtr model);

    [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int gmj_nv(IntPtr model);

    [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int gmj_nu(IntPtr model);

    [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int gmj_body_id(IntPtr model, [MarshalAs(UnmanagedType.LPUTF8Str)] string bodyName);

    [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int gmj_joint_id(IntPtr model, [MarshalAs(UnmanagedType.LPUTF8Str)] string jointName);

    [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int gmj_actuator_id(IntPtr model, [MarshalAs(UnmanagedType.LPUTF8Str)] string actuatorName);

    [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int gmj_get_qpos_slice(IntPtr model, IntPtr data, int startIndex, int count, double[] outValues);

    [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int gmj_set_qpos_slice(IntPtr model, IntPtr data, int startIndex, int count, double[] values);

    [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int gmj_get_qvel_slice(IntPtr model, IntPtr data, int startIndex, int count, double[] outValues);

    [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int gmj_set_qvel_slice(IntPtr model, IntPtr data, int startIndex, int count, double[] values);

    [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int gmj_get_ctrl_slice(IntPtr model, IntPtr data, int startIndex, int count, double[] outValues);

    [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int gmj_set_ctrl_slice(IntPtr model, IntPtr data, int startIndex, int count, double[] values);

    [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int gmj_body_world_position(IntPtr model, IntPtr data, int bodyIndex, double[] outXyz3);

    [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
    private static extern IntPtr gmj_last_mujoco_error();

    public static string LastError()
    {
        IntPtr ptr = gmj_last_mujoco_error();
        return ptr == IntPtr.Zero ? string.Empty : Marshal.PtrToStringUTF8(ptr) ?? string.Empty;
    }

    public static byte[] CreateErrorBuffer()
    {
        return new byte[ErrorBufferBytes];
    }
}
