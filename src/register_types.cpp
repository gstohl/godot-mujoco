#include "register_types.h"

#include "mj_world.h"

#include <gdextension_interface.h>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <mujoco/mujoco.h>

using namespace godot;

// Route MuJoCo's log messages into Godot's console instead of stdout /
// MUJOCO_LOG.TXT so warnings and errors are visible in the editor Output panel.
static void gmj_log_handler(const mjLogMessage *msg) {
	if (msg == nullptr) {
		return;
	}
	String text = String::utf8(msg->subject);
	if (msg->body != nullptr && msg->body[0] != '\0') {
		text += "\n";
		text += String::utf8(msg->body);
	}
	if (msg->level >= mjLOG_ERROR) {
		UtilityFunctions::push_error("MuJoCo: " + text);
	} else if (msg->level == mjLOG_WARNING) {
		UtilityFunctions::push_warning("MuJoCo: " + text);
	} else {
		UtilityFunctions::print("MuJoCo: " + text);
	}
}

void initialize_godot_mujoco_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
	// The log handler is process-global; install once at scene init.
	mju_setLogHandler(gmj_log_handler);
	GDREGISTER_CLASS(MjWorld);
}

void uninitialize_godot_mujoco_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
	mju_setLogHandler(nullptr);
}

extern "C" {
// GDExtension entry point referenced by godot_mujoco.gdextension.
GDExtensionBool GDE_EXPORT godot_mujoco_library_init(GDExtensionInterfaceGetProcAddress p_get_proc_address,
                                                     const GDExtensionClassLibraryPtr p_library,
                                                     GDExtensionInitialization *r_initialization) {
	godot::GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);

	init_obj.register_initializer(initialize_godot_mujoco_module);
	init_obj.register_terminator(uninitialize_godot_mujoco_module);
	init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);

	return init_obj.init();
}
}
