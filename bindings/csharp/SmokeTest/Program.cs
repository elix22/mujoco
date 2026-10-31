// Loads an MJCF model (default: model/humanoid/humanoid.xml), checks that the generated struct
// layouts agree with the native library, steps 1000 times and loads the same model from memory.
using System.Text;
using Mujoco;
using static Mujoco.MujocoLib;

unsafe
{
    string root = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "../../../../../.."));
    string path = args.Length > 0 ? args[0] : Path.Combine(root, "model/humanoid/humanoid.xml");
    int failures = 0;
    void Check(bool ok, string what)
    {
        Console.WriteLine($"{(ok ? "ok  " : "FAIL")} {what}");
        if (!ok) failures++;
    }

    Console.WriteLine($"MuJoCo {MjHelpers.Version} (mj_version {mj_version()}, header {mjVERSION_HEADER})");
    Check(mj_version() == mjVERSION_HEADER, "library version matches the generated header constant");

    mjModel_* m = MjHelpers.LoadXml(path);
    mjData_* d = mj_makeData(m);
    Console.WriteLine($"{Path.GetFileName(path)}: nq={m->nq} nv={m->nv} nu={m->nu} nbody={m->nbody} ngeom={m->ngeom} timestep={m->opt.timestep}");

    // signature is the last field of both mjModel and mjData; mj_makeData copies it, so equal
    // non-zero values mean every field offset before it matches the C layout.
    Check(m->signature != 0 && m->signature == d->signature, $"mjModel/mjData layout (signature 0x{m->signature:x})");
    var names = Enumerable.Range(0, (int)m->nbody).Select(i => MjHelpers.Name(m, mjtObj.mjOBJ_BODY, i)).ToArray();
    var fromStruct = Enumerable.Range(0, (int)m->nbody)
        .Select(i => System.Runtime.InteropServices.Marshal.PtrToStringAnsi((IntPtr)((byte*)m->names + m->name_bodyadr[i]))).ToArray();
    Check(names.SequenceEqual(fromStruct), $"body names via struct pointers == mj_id2name ({string.Join(",", names.Skip(1).Take(4))}...)");

    double z0 = d->qpos[2];
    for (int i = 0; i < 1000; i++)
        mj_step(m, d);
    Console.WriteLine($"after 1000 steps: time={d->time:F4} root z {z0:F3} -> {d->qpos[2]:F3} ncon={d->ncon}");
    Check(Math.Abs(d->time - 1000 * m->opt.timestep) < 1e-9, "time == 1000 * timestep");
    Check(MjHelpers.Span(d->qpos, m->nq).ToArray().All(double.IsFinite), "qpos finite");

    if (args.Length == 0)
    {
        Check(m->nq == 28 && m->nv == 27 && m->nu == 21, "humanoid nq=28 nv=27 nu=21");
        Check(d->qpos[2] < z0 - 0.3, "humanoid fell");
    }

    // FR-M.5: same model from memory through a VFS (the humanoid has no external assets).
    mjModel_* m2 = MjHelpers.LoadXml("model.xml", File.ReadAllBytes(path));
    Check(m2->nq == m->nq && m2->nv == m->nv && m2->ngeom == m->ngeom, "VFS load matches file load");
    try { MjHelpers.LoadXml("bad.xml", Encoding.UTF8.GetBytes("<mujoco><worldbody><geom type='nope'/></worldbody></mujoco>")); Check(false, "bad XML rejected"); }
    catch (MujocoException e) { Check(e.Message.Contains("nope") || e.Message.Length > 20, $"bad XML rejected: {e.Message.Split('\n')[0]}"); }

    mj_deleteModel(m2);
    mj_deleteData(d);
    mj_deleteModel(m);
    Console.WriteLine(failures == 0 ? "PASS" : $"{failures} FAILED");
    return failures == 0 ? 0 : 1;
}
