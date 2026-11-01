// Copyright 2026 Eli Aloni
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Hand-written helpers over the generated MujocoLib (MjBindings.cs).

using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;

namespace Mujoco;

public sealed class MujocoException(string message) : Exception(message);

public static unsafe class MjHelpers
{
    const int ErrorSize = 1000;

    public static string Version => Marshal.PtrToStringAnsi(MujocoLib.mj_versionString()) ?? "";

    /// <summary>Compiles an MJCF (or MJB) file from disk. Throws with MuJoCo's error text.</summary>
    public static MujocoLib.mjModel_* LoadXml(string path)
    {
        var error = new StringBuilder(ErrorSize);
        var m = MujocoLib.mj_loadXML(path, null, error, ErrorSize);
        if (m == null)
            throw new MujocoException($"mj_loadXML({path}): {error}");
        return m;
    }

    /// <summary>
    /// Compiles MJCF from memory through a MuJoCo VFS, without temp files. <paramref name="name"/> is
    /// the virtual file name of the XML; <paramref name="assets"/> holds files it references
    /// (meshes, textures, includes) by the name used in the XML.
    /// </summary>
    public static MujocoLib.mjModel_* LoadXml(string name, ReadOnlySpan<byte> xml,
        IReadOnlyDictionary<string, byte[]>? assets = null)
    {
        void* vfs = NativeMemory.AllocZeroed((nuint)sizeof(nint)); // mjVFS = { void* impl_; }
        MujocoLib.mj_defaultVFS(vfs);
        try
        {
            AddBuffer(vfs, name, xml);
            if (assets != null)
                foreach (var (assetName, data) in assets)
                    AddBuffer(vfs, assetName, data);

            var error = new StringBuilder(ErrorSize);
            var m = MujocoLib.mj_loadXML(name, vfs, error, ErrorSize);
            if (m == null)
                throw new MujocoException($"mj_loadXML({name}): {error}");
            return m;
        }
        finally
        {
            MujocoLib.mj_deleteVFS(vfs);
            NativeMemory.Free(vfs);
        }
    }

    static void AddBuffer(void* vfs, string name, ReadOnlySpan<byte> data)
    {
        int rc;
        fixed (byte* p = data)
            rc = MujocoLib.mj_addBufferVFS(vfs, name, p, data.Length);
        if (rc != 0)
            throw new MujocoException($"mj_addBufferVFS({name}) returned {rc}");
    }

    /// <summary>A span over a MuJoCo array, e.g. <c>Span(d->qpos, m->nq)</c>.</summary>
    public static Span<T> Span<T>(T* p, ulong count) where T : unmanaged => new(p, checked((int)count));

    public static string? Name(MujocoLib.mjModel_* m, MujocoLib.mjtObj type, int id) =>
        Marshal.PtrToStringAnsi(MujocoLib.mj_id2name(m, (int)type, id));

    public static int Id(MujocoLib.mjModel_* m, MujocoLib.mjtObj type, string name) =>
        MujocoLib.mj_name2id(m, (int)type, name);
}
