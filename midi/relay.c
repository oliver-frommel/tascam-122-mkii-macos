// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Oliver Frommel

// us122-midirelay — joins the two halves of the US-122MKII MIDI support.
//
// The audio plug-in (running sandboxed as _coreaudiod) owns the USB device but
// may not use CoreMIDI; the CoreMIDI driver (running in the user's MIDIServer)
// can publish ports but cannot reach the device. Both can reach a system-wide
// Mach service, so this launchd daemon relays between them:
//
//   audio -> midi:  {"in": <raw MIDI bytes>}, {"present": bool}
//   midi -> audio:  {"out": <raw MIDI bytes>}
//
// Each side first sends {"role": "audio" | "midi"}. The last known "present"
// value is replayed to the MIDI side when it (re)connects, and reset when the
// audio side goes away.

#include <xpc/xpc.h>
#include <dispatch/dispatch.h>
#include <os/log.h>
#include <libproc.h>
#include <pwd.h>
#include <string.h>

#define SERVICE "local.us122mkii.midirelay"
#define MIDISERVER_PATH "/System/Library/Frameworks/CoreMIDI.framework/MIDIServer"

static os_log_t gLog;
static dispatch_queue_t gQ;
static xpc_connection_t gAudio, gMidi;
static bool gPresent;
static uid_t gCoreAudioUID = (uid_t)-1;

static void sendBool(xpc_connection_t c, const char *key, bool v) {
    if (!c) return;
    xpc_object_t m = xpc_dictionary_create(NULL, NULL, 0);
    xpc_dictionary_set_bool(m, key, v);
    xpc_connection_send_message(c, m);
}

static bool isMIDIServer(xpc_connection_t peer) {
    char path[PROC_PIDPATHINFO_MAXSIZE] = "";
    if (proc_pidpath(xpc_connection_get_pid(peer), path, sizeof path) <= 0) return false;
    return strcmp(path, MIDISERVER_PATH) == 0;
}

static void peerGone(xpc_connection_t peer) {
    if (peer == gAudio) {
        gAudio = NULL;
        if (gPresent) { gPresent = false; sendBool(gMidi, "present", false); }
        os_log(gLog, "audio side disconnected");
    } else if (peer == gMidi) {
        gMidi = NULL;
        os_log(gLog, "midi side disconnected");
    }
}

static void handle(xpc_connection_t peer, xpc_object_t msg) {
    if (xpc_get_type(msg) != XPC_TYPE_DICTIONARY) {
        peerGone(peer); // error: connection invalid / interrupted
        return;
    }
    const char *role = xpc_dictionary_get_string(msg, "role");
    if (role) {
        if (!strcmp(role, "audio") && xpc_connection_get_euid(peer) == gCoreAudioUID) {
            if (gAudio && gAudio != peer) xpc_connection_cancel(gAudio);
            gAudio = peer;
            os_log(gLog, "audio side connected (pid %d)", xpc_connection_get_pid(peer));
        } else if (!strcmp(role, "midi") && isMIDIServer(peer)) {
            if (gMidi && gMidi != peer) xpc_connection_cancel(gMidi); // e.g. fast user switch
            gMidi = peer;
            os_log(gLog, "midi side connected (pid %d, uid %d)", xpc_connection_get_pid(peer),
                   xpc_connection_get_euid(peer));
            sendBool(gMidi, "present", gPresent);
        } else {
            os_log_error(gLog, "rejected role '%{public}s' from pid %d", role, xpc_connection_get_pid(peer));
            xpc_connection_cancel(peer);
        }
        return;
    }
    if (peer == gAudio) {
        if (xpc_dictionary_get_value(msg, "present")) {
            gPresent = xpc_dictionary_get_bool(msg, "present");
            sendBool(gMidi, "present", gPresent);
        }
        if (xpc_dictionary_get_value(msg, "in") && gMidi) xpc_connection_send_message(gMidi, msg);
    } else if (peer == gMidi) {
        if (xpc_dictionary_get_value(msg, "out") && gAudio) xpc_connection_send_message(gAudio, msg);
    }
}

int main(void) {
    gLog = os_log_create("local.us122mkii.driver", "relay");
    gQ = dispatch_queue_create("relay", DISPATCH_QUEUE_SERIAL);
    struct passwd *pw = getpwnam("_coreaudiod");
    if (pw) gCoreAudioUID = pw->pw_uid;
    xpc_connection_t listener = xpc_connection_create_mach_service(SERVICE, gQ, XPC_CONNECTION_MACH_SERVICE_LISTENER);
    xpc_connection_set_event_handler(listener, ^(xpc_object_t peer) {
        if (xpc_get_type(peer) != XPC_TYPE_CONNECTION) return;
        xpc_connection_set_target_queue(peer, gQ);
        xpc_connection_set_event_handler(peer, ^(xpc_object_t msg) { handle(peer, msg); });
        xpc_connection_resume(peer);
    });
    xpc_connection_resume(listener);
    os_log(gLog, "listening");
    dispatch_main();
}
