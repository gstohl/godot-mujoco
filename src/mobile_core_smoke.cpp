// Standalone MuJoCo core smoke — no Godot, no OpenGL.
//
// Proves the from-source simulation core actually steps a model. On Android
// this is a PIE executable you can `adb push` to a device or emulator:
//
//   adb push demo/addons/godot_mujoco/bin/gmj_core_smoke /data/local/tmp/
//   adb push demo/addons/godot_mujoco/bin/libmujoco.so* /data/local/tmp/
//   adb shell "cd /data/local/tmp && LD_LIBRARY_PATH=. ./gmj_core_smoke"
//
// Exit 0 and a "CORE SMOKE: PASS" line means mj_loadXML + mj_step worked.

#include <mujoco/mujoco.h>

#include <cmath>
#include <cstdio>
#include <cstring>

static const char kPendulumXml[] =
        "<mujoco model=\"pendulum\">\n"
        "  <option timestep=\"0.01\" gravity=\"0 0 -9.81\"/>\n"
        "  <worldbody>\n"
        "    <body name=\"pendulum\" pos=\"0 0 1\">\n"
        "      <joint name=\"hinge\" type=\"hinge\" axis=\"0 1 0\"/>\n"
        "      <geom type=\"capsule\" fromto=\"0 0 0 0 0 -0.6\" size=\"0.04\" density=\"600\"/>\n"
        "    </body>\n"
        "  </worldbody>\n"
        "  <actuator>\n"
        "    <motor name=\"hinge_motor\" joint=\"hinge\" gear=\"50\"/>\n"
        "  </actuator>\n"
        "</mujoco>\n";

int main() {
	std::printf("MuJoCo %s\n", mj_versionString());

	mjVFS vfs;
	mj_defaultVFS(&vfs);
	if (mj_addBufferVFS(&vfs, "pendulum.xml", kPendulumXml, (int)std::strlen(kPendulumXml)) != 0) {
		std::fprintf(stderr, "CORE SMOKE: FAIL (VFS add)\n");
		mj_deleteVFS(&vfs);
		return 1;
	}

	char err[1024] = {0};
	mjModel *m = mj_loadXML("pendulum.xml", &vfs, err, sizeof(err));
	mj_deleteVFS(&vfs);
	if (m == nullptr) {
		std::fprintf(stderr, "CORE SMOKE: FAIL (load): %s\n", err);
		return 1;
	}
	mjData *d = mj_makeData(m);
	if (d == nullptr) {
		std::fprintf(stderr, "CORE SMOKE: FAIL (mj_makeData)\n");
		mj_deleteModel(m);
		return 1;
	}

	if (m->nq < 1 || m->nu < 1) {
		std::fprintf(stderr, "CORE SMOKE: FAIL (unexpected sizes nq=%d nu=%d)\n", m->nq, m->nu);
		mj_deleteData(d);
		mj_deleteModel(m);
		return 1;
	}

	d->ctrl[0] = 0.15;
	for (int i = 0; i < 50; ++i) {
		mj_step(m, d);
	}

	const double q = d->qpos[0];
	if (!std::isfinite(q)) {
		std::fprintf(stderr, "CORE SMOKE: FAIL (non-finite qpos)\n");
		mj_deleteData(d);
		mj_deleteModel(m);
		return 1;
	}

	std::printf("nq=%d nu=%d time=%.3f qpos0=%.6f\n", m->nq, m->nu, d->time, q);
	std::printf("CORE SMOKE: PASS\n");

	mj_deleteData(d);
	mj_deleteModel(m);
	return 0;
}
