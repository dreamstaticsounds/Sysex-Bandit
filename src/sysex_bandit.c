/* Sysex Bandit -- MPC OS VST2 port of the Sysex Bandit SysEx tool (a silent
 * instrument: the "audio" is silence, the real I/O is a pair of ALSA sequencer
 * ports, like the kit's poc/midiport.c but for System Exclusive).
 *
 * Ports on one ALSA client, "Sysex Bandit":
 *   "SysEx Out"  CAP_READ|SUBS_READ   -- publishes what the SEND key sends, to every
 *                                       subscriber (snd_seq_ev_set_subs + output_direct);
 *                                       the sweep also subscribes it to every writable
 *                                       port, so SEND reaches a device with no wiring
 *   "SysEx In"   CAP_WRITE|SUBS_WRITE -- receives; while LISTEN is ON a 2 s sweep
 *                                       subscribes it to every readable port on the
 *                                       sequencer (own and SYSTEM clients excluded),
 *                                       the same can_receive() rule as the app.
 *
 * Incoming SysEx is reassembled before it counts: ALSA hands a long message over
 * in several events -- it cuts at 4096 bytes, so a bank dump arrives as hundreds of
 * fragments -- and the reader accumulates bytes and accepts only complete F0..F7
 * messages, the app's extract_messages() rule. One dump = one counted message.
 *
 * Every accepted message is APPENDED to a capture buffer (the app's whole session,
 * not just the last block): a JD-990 bank dump is a run of ~120-byte messages, and
 * 1.0.0 only kept the newest of them, so SAVE RX wrote 123 bytes of a 32 KB bank.
 * The capture holds every message since CLEAR (4 MB / 4096 messages), RX MSG steps
 * through it (n/m, newest first: a new message moves the view to the end), SAVE RX
 * writes the whole run back as a normal multi-message .syx, and the reassembly
 * buffer is 1 MB: a patch is a few hundred bytes, a bank dump is not.
 *
 * Both directions are pickable, because a box with two synths plugged in must not
 * get a dump aimed at the other one, and because MPC OS's own clients talk SysEx
 * too: OUT PORT picks one discovered writable port, addressed with
 * snd_seq_ev_set_dest instead of a broadcast (None is the default and a real
 * choice: SEND is refused rather than sprayed at everything); IN PORT picks one
 * discovered readable port, the only one the sweep subscribes (None: nobody).
 * The Nth port is the Nth in ALSA's own discovery order -- client id,
 * then port id, the order aconnect -l prints -- but the PICK is remembered by
 * identity (client:port, then the "client:port" name): another client coming or
 * going shifts every index after it, and a client that restarts keeps its name
 * under a new id, so the pick follows the device instead of silently changing
 * meaning, and a port that is really gone reads (gone) and refuses.
 *
 * Threading: the wrapper serialises its own engine calls, but a reader thread
 * shares this struct, so everything the reader touches also takes sb->lock (the
 * wrapper's lock does not cover it). Every sequencer call -- send, subscribe,
 * drain -- happens under that lock; render() only memset()s its output and never
 * touches shared state, so audio never waits on it. The mutex is
 * priority-inheriting like the wrapper's, so a sweep can't invert an RT audio
 * thread that polls us from housekeeping().
 *
 * SEND never blocks the host thread: do_send() loads the file and hands it to the
 * tx thread, which paces it out (tx_gap ms per tx_buf-byte block, tx_delay ms after
 * every F7 -- the Midi-OX sysex settings) and then reports the same STATUS line a
 * synchronous send used to. A 32 KB dump at full throttle overruns the receiving
 * synth's input buffer; at the defaults (10 ms after F7, 512 B per 1 ms block) it
 * takes about a second and lands whole.
 *
 * The reader must poll() once and then drain snd_seq_event_input() to -EAGAIN:
 * poll-per-event stalls the moment a burst arrives (proven in
 * test/sysex_read_pattern.c). poll() waits on a snapshot of the sequencer's fd
 * taken at thread start plus a quit pipe, so no ALSA call happens while waiting.
 *
 * Readout params (tx/rx/status/folder/port, display "string") ignore sets -- the
 * host may write "0" to them; momentary keys act only on "1" and always read
 * back "0". "state" is a k=v; chunk including file=<name>, so a preset restores
 * to the same .syx file by name (docs/PORTING.md), not by slot number.
 * display_rev is a hash of every engine-driven numeric value: when it changes,
 * the wrapper diffs the params and automates what moved (rx_total, message after
 * a prev/next tap). */
#include <alsa/asoundlib.h>
#include <dirent.h>
#include <pthread.h>
#include <poll.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <time.h>
#include <unistd.h>
#include "engine.h"          /* mpc_engine_t, the wrapper's interface (wrapper/) */

#define SB_TEXT     64        /* == PARAM_TEXT_MAX (vst.json): room for 63 chars + NUL */
#define SB_NAME     96        /* one .syx file name */
#define MAX_MSGS    64        /* message slots 1..64 */
#define MAX_FILES   256       /* .syx names considered when sorting the folder */
#define MAX_FILE    (4 * 1024 * 1024)  /* app's size cap for one .syx file */
#define CAP_MAX     (4 * 1024 * 1024)  /* capture: every complete message since CLEAR, so
                                        * SAVE RX writes the whole bank dump and not just
                                        * its last block. malloc'd in sb_create. */
#define CAP_MSGS    4096       /* message starts remembered, for RX MSG to step over */
#define ACC_MAX     (1024 * 1024)  /* reassembly buffer -- ALSA cuts SysEx at 4096 bytes
                                    * per event, so a bank dump arrives as hundreds of
                                    * fragments and all of them have to fit before F7 */
#define MARQ_W      13         /* characters of a file name the MESSAGE box shows at once;
                                * longer names scroll through it (its frame is hand-drawn
                                * artwork 140 px wide, so the box cannot grow) */
#define MARQ_GAP    3          /* blanks between the two ends of the scrolling name */
#define MAX_SUBS    64        /* remembered sweep subscriptions (each direction) */
#define MAX_TG      64        /* selectable ports per direction (OUT PORT / IN PORT) */
#define TX_LINES    6         /* TX box lines: layout.conf stacks one readout per line,
                               * because MPC's Label does not wrap */
#define RX_LINES    6
#define TX_BPL      26        /* bytes per line -- sized so a line just fills its box
                               * at layout.conf's tsize (77 chars, inside the host's
                               * PARAM_TEXT_MAX) */
#define RX_BPL      27        /* 80 chars: the widest line that still fits the buffer */
#define SEL_MAX     256       /* head of the selected message kept for the TX box */
#define ECHO_SLOTS  8         /* recent sends recognised as their own echo */
#define ECHO_MS     5000      /* ... but only for this long after the send: the same
                               * bytes arriving minutes later are a real message
                               * (a read-back of the patch you just sent) */
#define SWEEP_MS    2000      /* LISTEN sweep period */
#define POLL_MS     200       /* reader sleep; also the quit reaction time */

/* One discovered port, and the pick remembered by identity. The lists are rebuilt
 * in ALSA's discovery order every sweep, so an index stops meaning anything the
 * moment another client comes or goes: client:port first, the "client:port" name
 * as the fallback (a client that restarts -- MPC reloaded, the sender app bounced
 * -- gets a new id but keeps its name). aim_t: client < 0 = not matched against
 * the list yet (a restored chunk before the first sweep), gone = matched and no
 * longer there. name[0] == 0 means no pick at all. */
typedef struct { int client, port; char name[SB_TEXT]; } tgt_t;
typedef struct { int client, port, gone; char name[SB_TEXT]; } aim_t;
typedef struct { uint32_t h; long long at; } echo_t;   /* a sent message, and when */

typedef struct {
    snd_seq_t *seq;
    int out_port, in_port, self_client;
    int have_thread;
    int quit_pipe[2];
    pthread_t thr;
    pthread_mutex_t lock;

    /* settings the host owns (params.json defaults) */
    int message;              /* 1..MAX_MSGS slot */
    int send_src;             /* 0 FILE, 1 BYTES */
    int nbytes;               /* 1..8 */
    int b[8];
    int listen;               /* 0 OFF, 1 ON */
    int send_to;              /* 0 None, else the Nth discovered writable port */
    int recv_from;            /* 0 None, else the Nth discovered readable port */
    int tx_delay;             /* ms to wait after every F7 before the next message */
    int tx_buf;               /* bytes per paced block of one message */
    int tx_gap;               /* ms to wait between those blocks */

    /* engine-driven readouts */
    char status[SB_TEXT], folder[SB_TEXT], port_str[SB_TEXT];
    int rx_total;             /* complete messages received (wraps 0..999) */
    /* the capture: every complete message since CLEAR, one after the other, with
     * cap_off[] marking where each starts. RX shows rx_msg of them (a new message
     * moves rx_msg to the end), SAVE RX writes all of cap_len. */
    uint8_t *cap;
    int cap_len, cap_msgs;
    int cap_off[CAP_MSGS];
    int rx_msg;               /* 1-based index into the capture, 1 when it is empty */
    uint8_t acc[ACC_MAX];     /* incoming stream, reassembled into F0..F7 here */
    int acc_len;
    uint8_t sel[SEL_MAX];     /* head of the message the TX box shows */
    int sel_len;
    int sel_dirty;            /* the selection changed: re-read it before the next line */
    unsigned marq_ph;         /* file-name scroll phase, advanced by rev_hash so the
                               * wrapper's display_rev change repaints the MESSAGE box */
    unsigned marq_tick;       /* half of those polls advance marq_ph: 1 char / 200 ms */
    char marq_nm[SB_NAME];    /* the name marq_ph belongs to: selecting another file
                               * starts its window at the beginning again */
    int marq_of;              /* and the slot it belongs to (a fast turn A->B->A between
                               * two display reads must restart too) */

    /* message folder */
    char dir[512];
    char names[MAX_MSGS][SB_NAME];
    int nmsgs;

    /* sweep subscriptions (our in-port <- their out-port) */
    struct { int client, port; } subs[MAX_SUBS];
    int nsubs;
    /* ... and (our out-port -> their in-port), so SEND reaches devices */
    struct { int client, port; } osubs[MAX_SUBS];
    int nosubs;
    int need_sweep;

    /* what OUT PORT / IN PORT can pick: every usable port on the sequencer, in
     * discovery order, with the name the readout shows (rebuilt every sweep) */
    tgt_t tg_in[MAX_TG];
    int ntg_in;
    tgt_t tg_out[MAX_TG];
    int ntg_out;
    /* the pick each picker is aimed at, kept across rebuilds by identity: the
     * index follows the port when the list shifts (resolve_pick) */
    aim_t aim_in, aim_out;

    /* the queued SEND, drained by tx_thread under this lock (never on the host
     * thread: a paced 32 KB dump takes about a second) */
    uint8_t *txq;             /* file bytes, or the one BYTES message; owned by the queue */
    size_t txq_len, tx_pos;   /* tx_pos: where the next message starts */
    size_t tx_ms, tx_me;      /* the message open now: [tx_ms, tx_me), empty when equal */
    uint32_t tx_hash;         /* running hash of that message, for the echo filter */
    int tx_active, tx_quit;
    int tx_direct;            /* the queue IS one message (BYTES): no F0..F7 scan */
    int tx_done, tx_total;    /* messages finished / messages in the file */
    size_t tx_bytes, tx_total_bytes;
    int tx_stray;
    snd_seq_addr_t tx_dst;    /* OUT PORT resolved once, when SEND was pressed */
    struct timespec tx_due;   /* when the next block may go out (CLOCK_REALTIME) */
    pthread_t tx_thr;
    int tx_thr_on;
    pthread_cond_t tx_cv;

    /* echo filter: the last ECHO_SLOTS messages we sent, each with the time it
     * went out -- only a fresh one is dropped as "our own echo" */
    echo_t echo[ECHO_SLOTS];
    int echo_n;
} sb_t;

/* ---------------------------------------------------------------- helpers */

static uint32_t fnv1a(const uint8_t *p, int n) {
    uint32_t h = 2166136261u;
    for (int i = 0; i < n; i++) h = (h ^ p[i]) * 16777619u;
    return h;
}

static int clampi(int v, int lo, int hi) { return v < lo ? lo : v > hi ? hi : v; }

static void set_status(sb_t *s, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s->status, sizeof s->status, fmt, ap);
    va_end(ap);
}

/* option params arrive as an index ("0"/"1"); a preset or a state line may use
 * the label instead. */
static int opt_index(const char *v, const char *lo, const char *hi) {
    if (v[0] >= '0' && v[0] <= '9') return clampi(atoi(v), 0, 1);
    return strcasecmp(v, hi) == 0 ? 1 : strcasecmp(v, lo) == 0 ? 0 : 0;
}

/* ------------------------------------------------------- message folder */

/* Sorted list of the folder's .syx names, first MAX_MSGS of them. */
static void scan_dir(sb_t *s) {
    s->nmsgs = 0;
    if (!s->dir[0]) return;
    DIR *d = opendir(s->dir);
    if (!d) return;
    static char all[MAX_FILES][SB_NAME];   /* static: called under sb->lock only */
    int n = 0;
    struct dirent *e;
    while (n < MAX_FILES && (e = readdir(d))) {
        size_t l = strlen(e->d_name);
        if (l < 5 || strcasecmp(e->d_name + l - 4, ".syx") != 0) continue;
        if (l >= SB_NAME) l = SB_NAME - 1;      /* long names truncate; sort still works */
        memcpy(all[n], e->d_name, l);
        all[n][l] = 0;
        n++;
    }
    closedir(d);
    /* insertion sort: n is small and readdir order is arbitrary */
    for (int i = 1; i < n; i++) {
        char tmp[SB_NAME];
        snprintf(tmp, SB_NAME, "%s", all[i]);
        int j = i - 1;
        while (j >= 0 && strcmp(all[j], tmp) > 0) { snprintf(all[j + 1], SB_NAME, "%s", all[j]); j--; }
        snprintf(all[j + 1], SB_NAME, "%s", tmp);
    }
    s->nmsgs = n < MAX_MSGS ? n : MAX_MSGS;
    for (int i = 0; i < s->nmsgs; i++) snprintf(s->names[i], SB_NAME, "%s", all[i]);
}

static const char *slot_name(sb_t *s) {
    return (s->message >= 1 && s->message <= s->nmsgs) ? s->names[s->message - 1] : NULL;
}

/* The scroll window belongs to one file: when a different one is selected the window
 * starts at the beginning, so the box shows the head of the new name right away instead
 * of continuing from wherever the last one stopped (marq_tick = 1 holds the head for one
 * poll before scrolling resumes). Called on every selection change and every display
 * read, always with sb->lock held. */
static void marq_sync(sb_t *s) {
    const char *nm = slot_name(s);
    const char *want = nm ? nm : "";
    if (s->marq_of != s->message || strcmp(s->marq_nm, want) != 0) {
        snprintf(s->marq_nm, sizeof s->marq_nm, "%s", want);
        s->marq_of = s->message;
        s->marq_ph = 0;
        s->marq_tick = 1;
    }
}

/* ------------------------------------------------------- the TX selection */

/* Head of the message the TX box shows: the edited bytes, or the slot's file
 * from its first F0 (the send path's stray-byte rule). Re-read only when the
 * selection changed -- the box is polled ten times a second and must not open a
 * file every time. */
static void load_sel(sb_t *s) {
    if (!s->sel_dirty) return;
    s->sel_dirty = 0;
    s->sel_len = 0;
    if (s->send_src == 1) {                     /* BYTES: exactly what SEND would put out */
        int n = clampi(s->nbytes, 1, 8);
        if (n > SEL_MAX) n = SEL_MAX;
        for (int i = 0; i < n; i++) s->sel[i] = (uint8_t)s->b[i];
        s->sel_len = n;
        return;
    }
    const char *name = slot_name(s);
    if (!name || !s->dir[0]) return;
    char path[sizeof s->dir + SB_NAME + 2];
    snprintf(path, sizeof path, "%s/%s", s->dir, name);
    FILE *f = fopen(path, "rb");
    if (!f) return;
    uint8_t buf[SEL_MAX];
    size_t got = fread(buf, 1, sizeof buf, f);
    fclose(f);
    size_t i = 0;
    while (i < got && buf[i] != 0xF0) i++;       /* stray bytes first: skip them */
    if (got - i > (size_t)SEL_MAX) got = i + (size_t)SEL_MAX;
    s->sel_len = (int)(got - i);
    memmove(s->sel, buf + i, (size_t)s->sel_len);
}

/* "XX XX ..." for bytes [off, off+bpl) of p; "" once past the end. */
static void hex_line(const uint8_t *p, int len, int off, int bpl, char *out, int cap) {
    if (cap <= 0) return;
    out[0] = 0;
    if (!p || off < 0 || off >= len) return;
    int n = len - off;
    if (n > bpl) n = bpl;
    int o = 0;
    for (int i = 0; i < n && o < cap - 4; i++)
        o += snprintf(out + o, (size_t)(cap - o), "%s%02X", i ? " " : "", p[off + i]);
}

/* sb_get has to answer >0 even for an empty line: 0 means "no such key". */
static int put_line(char *buf, int n, const char *s) {
    if (n <= 0) return 0;
    snprintf(buf, (size_t)n, "%s", s);
    return 1;
}

/* The message RX shows: rx_msg of the capture (1-based), 0 when there is none.
 * Clamped here too -- the capture can shrink under a restored preset. */
static int rx_sel(sb_t *s, int *off) {
    *off = 0;
    if (!s->cap || s->cap_msgs <= 0) return 0;
    int i = clampi(s->rx_msg, 1, s->cap_msgs);
    int st = s->cap_off[i - 1];
    int en = (i < s->cap_msgs) ? s->cap_off[i] : s->cap_len;
    *off = st;
    return en - st;
}

/* The MESSAGE box's frame is hand-drawn artwork 140 px wide, so a name longer than
 * MARQ_W characters cannot be shown whole: it scrolls through that window instead.
 * rev_hash advances marq_ph, which moves display_rev, which is what makes the
 * wrapper ask for this text again (a Label clips, it never wraps or shrinks). */
static const char *msg_name(sb_t *s, char *out, int cap) {
    marq_sync(s);                              /* a new file starts its window at the head */
    const char *nm = slot_name(s);
    if (!nm) return s->nmsgs ? "(none)" : "(empty)";
    int len = (int)strlen(nm);
    if (len <= MARQ_W) return nm;
    int o = 0;
    unsigned ph = s->marq_ph % (unsigned)(len + MARQ_GAP);
    for (int i = 0; i < MARQ_W && o < cap - 1; i++) {
        unsigned k = ph + (unsigned)i;
        out[o++] = (k < (unsigned)len) ? nm[k] : ' ';
    }
    out[o] = 0;
    return out;
}

static int tx_line(sb_t *s, int ln, char *buf, int n) {
    if (s->send_to <= 0)                        /* OUT PORT None: nothing selected to send */
        return put_line(buf, n, ln == 1 ? "-" : "");
    load_sel(s);
    hex_line(s->sel, s->sel_len, (ln - 1) * TX_BPL, TX_BPL, buf, n);
    return 1;
}

static int rx_line(sb_t *s, int ln, char *buf, int n) {
    int off = 0, len = rx_sel(s, &off);
    if (ln == 1 && len <= 0) return put_line(buf, n, "-");
    hex_line(s->cap + off, len, (ln - 1) * RX_BPL, RX_BPL, buf, n);
    return 1;
}

/* "tx3" / "rx5" -> the 1-based line number, 0 when this is not a line key. */
static int line_no(const char *key, char letter, int lines) {
    if (key[0] != letter || key[1] != 'x') return 0;
    if (key[2] < '1' || key[2] > '9' || key[3]) return 0;
    int ln = key[2] - '0';
    return ln <= lines ? ln : 0;
}

/* ---------------------------------------------------------- port targets */

/* The pick is remembered by identity, not by its place in the list. aim_clear
 * drops it (the user stepped elsewhere: the next rebuild re-aims from the new
 * index), aim_take reads it off the list at the current index, aim_restore
 * brings back a chunk's saved name. All under s->lock. */
static void aim_clear(sb_t *s, int out) {
    aim_t *a = out ? &s->aim_out : &s->aim_in;
    a->client = a->port = -1;
    a->gone = 0;
    a->name[0] = 0;
}

static void aim_take(sb_t *s, int out) {
    aim_t *a = out ? &s->aim_out : &s->aim_in;
    int idx = out ? s->send_to : s->recv_from;
    int cnt = out ? s->ntg_out : s->ntg_in;
    tgt_t *t = out ? s->tg_out : s->tg_in;
    aim_clear(s, out);
    if (idx >= 1 && idx <= cnt) {
        a->client = t[idx - 1].client;
        a->port = t[idx - 1].port;
        snprintf(a->name, sizeof a->name, "%s", t[idx - 1].name);
    }
}

/* A restored chunk: the saved name wins over the saved index (the list may have
 * moved around since it was written), an index-only chunk (1.0.0 and earlier)
 * re-aims from whatever sits at that index on the next rebuild, exactly as it
 * used to. Either way the name is settled by resolve_pick() on the next sweep. */
static void aim_restore(sb_t *s, int out, const char *name) {
    aim_t *a = out ? &s->aim_out : &s->aim_in;
    int idx = out ? s->send_to : s->recv_from;
    aim_clear(s, out);
    if (idx > 0 && name && name[0]) {
        a->client = a->port = -1;                 /* pending: not matched yet */
        snprintf(a->name, sizeof a->name, "%s", name);
    }
}

/* The pick's identity as the chunk should keep it: the aim when it has one, the
 * name at the current index otherwise (the readout shows that same name). */
static const char *aim_name(sb_t *s, int out) {
    const aim_t *a = out ? &s->aim_out : &s->aim_in;
    int idx = out ? s->send_to : s->recv_from;
    int cnt = out ? s->ntg_out : s->ntg_in;
    if (a->name[0]) return a->name;
    if (idx >= 1 && idx <= cnt) return (out ? s->tg_out : s->tg_in)[idx - 1].name;
    return "";
}

/* Re-aim both picks at the freshly built list: exact client:port first, then the
 * remembered "client:port" name. A pick found somewhere else moves its index
 * along with the port, so the readout, the arrows and the saved chunk all keep
 * agreeing on the same device; a pick that is nowhere is marked gone and left
 * where it was, so the readout says (gone) instead of quietly coming to mean
 * whatever now sits at that index -- and it is re-checked every sweep, so the
 * device coming back (or coming back under a new id) re-finds it by name.
 * Called from rebuild_targets, always under s->lock. */
static void resolve_pick(sb_t *s) {
    for (int out = 0; out < 2; out++) {
        aim_t *a = out ? &s->aim_out : &s->aim_in;
        int *pick = out ? &s->send_to : &s->recv_from;
        tgt_t *t = out ? s->tg_out : s->tg_in;
        int cnt = out ? s->ntg_out : s->ntg_in;
        if (*pick <= 0) { aim_clear(s, out); continue; }        /* None: nobody */
        if (!a->name[0]) aim_take(s, out);                     /* index-only pick */
        if (!a->name[0]) continue;                             /* index past the end: (gone) by index */
        int found = -1;
        for (int i = 0; i < cnt && found < 0; i++)
            if (t[i].client == a->client && t[i].port == a->port) found = i;
        for (int i = 0; i < cnt && found < 0; i++)
            if (!strcmp(t[i].name, a->name)) found = i;
        if (found < 0) { a->gone = 1; continue; }
        a->client = t[found].client;
        a->port = t[found].port;
        a->gone = 0;
        snprintf(a->name, sizeof a->name, "%s", t[found].name);
        *pick = found + 1;                                      /* the index follows the port */
    }
}

/* Who is on the sequencer: fill the OUT PORT / IN PORT candidate lists, names
 * included. Discovery order is ALSA's own (client id, then port id), so "port 3"
 * is the third port aconnect -l would print. Called from the sweep and from a
 * send, always under s->lock, so a device plugged in a second ago is already
 * aimable without waiting out the 2 s sweep. */
static void rebuild_targets(sb_t *s) {
    s->ntg_in = s->ntg_out = 0;
    if (!s->seq) return;
    snd_seq_client_info_t *ci;
    snd_seq_port_info_t *pi;
    snd_seq_client_info_alloca(&ci);
    snd_seq_port_info_alloca(&pi);
    snd_seq_client_info_set_client(ci, -1);
    while (snd_seq_query_next_client(s->seq, ci) >= 0) {
        int cid = snd_seq_client_info_get_client(ci);
        if (cid == s->self_client || cid == SND_SEQ_CLIENT_SYSTEM) continue;
        const char *cname = snd_seq_client_info_get_name(ci);
        if (!cname || !cname[0]) cname = "?";
        snd_seq_port_info_set_client(pi, cid);
        snd_seq_port_info_set_port(pi, -1);
        while (snd_seq_query_next_port(s->seq, pi) >= 0) {
            unsigned caps = snd_seq_port_info_get_capability(pi);
            int pid = snd_seq_port_info_get_port(pi);
            const char *pname = snd_seq_port_info_get_name(pi);
            if (!pname || !pname[0]) pname = "?";
            if ((caps & (SND_SEQ_PORT_CAP_READ | SND_SEQ_PORT_CAP_SUBS_READ)) ==
                (SND_SEQ_PORT_CAP_READ | SND_SEQ_PORT_CAP_SUBS_READ) && s->ntg_in < MAX_TG) {
                s->tg_in[s->ntg_in].client = cid;
                s->tg_in[s->ntg_in].port = pid;
                snprintf(s->tg_in[s->ntg_in].name, SB_TEXT, "%s:%s", cname, pname);
                s->ntg_in++;
            }
            if ((caps & (SND_SEQ_PORT_CAP_WRITE | SND_SEQ_PORT_CAP_SUBS_WRITE)) ==
                (SND_SEQ_PORT_CAP_WRITE | SND_SEQ_PORT_CAP_SUBS_WRITE) && s->ntg_out < MAX_TG) {
                s->tg_out[s->ntg_out].client = cid;
                s->tg_out[s->ntg_out].port = pid;
                snprintf(s->tg_out[s->ntg_out].name, SB_TEXT, "%s:%s", cname, pname);
                s->ntg_out++;
            }
        }
    }
    resolve_pick(s);                        /* and re-aim the picks at what is there now */
}

/* What the picker shows: None (0) is a real choice -- no devices at all -- and
 * the pick's own name wherever it has moved to, with (gone) meaning the chosen
 * port has since unplugged (a send is refused rather than quietly reaching
 * everybody). */
static const char *tg_name(sb_t *s, int out, int n) {
    if (n <= 0) return "None";
    const aim_t *a = out ? &s->aim_out : &s->aim_in;
    if (a->name[0]) return a->gone ? "(gone)" : a->name;
    int cnt = out ? s->ntg_out : s->ntg_in;
    if (n > cnt) return "(gone)";
    return out ? s->tg_out[n - 1].name : s->tg_in[n - 1].name;
}

/* Which readable ports this instance receives from: only the one IN PORT picked,
 * by identity (see resolve_pick). Used to subscribe and to drop what was
 * subscribed before the pick changed. None, (gone) and not-yet-resolved all mean
 * nobody. */
static int in_wanted(sb_t *s, int cid, int pid) {
    const aim_t *a = &s->aim_in;
    if (!a->name[0] || a->gone || a->client < 0) return 0;
    return a->client == cid && a->port == pid;
}

/* Where a send goes: 1 = dst filled with the picked port, -1 = nowhere. None (0)
 * is a destination like any other -- it is refused, not broadcast, so SEND with
 * no port picked does not spray the dump at every device on the box, and a port
 * that has vanished refuses the same way instead of reaching its neighbour. */
static int out_target(sb_t *s, snd_seq_addr_t *dst) {
    const aim_t *a = &s->aim_out;
    if (s->send_to <= 0) { set_status(s, "pick OUT PORT"); return -1; }
    if (!a->name[0] || a->gone || a->client < 0) {
        set_status(s, "out port %d not found", s->send_to);
        return -1;
    }
    dst->client = (unsigned)a->client;
    dst->port = (unsigned)a->port;
    return 1;
}

/* OUT/IN PORT's prev & next arrows: re-read what is actually plugged in, then
 * step one place -- stopping at the last port that exists, so an arrow can never
 * walk the picker onto "(none)". The step is a new pick: the aim is dropped and
 * re-taken from the new index by the next rebuild. */
static void step_target(sb_t *s, int out, int dir) {
    rebuild_targets(s);
    int *pick = out ? &s->send_to : &s->recv_from;
    int cnt = out ? s->ntg_out : s->ntg_in;
    int v = *pick + dir;
    if (v < 0) v = 0;
    if (v > cnt) v = cnt;
    *pick = v;
    aim_clear(s, out);
    s->need_sweep = 1;                            /* the pick may drop a subscription */
}

/* ------------------------------------------------------------- sending */

/* One SysEx event out, addressed at OUT PORT's device (never a broadcast: a dump
 * must not reach the other synth on the box). */
static int emit(sb_t *s, const uint8_t *p, int len, const snd_seq_addr_t *dst) {
    snd_seq_event_t ev;
    snd_seq_ev_clear(&ev);
    snd_seq_ev_set_source(&ev, s->out_port);
    if (dst) snd_seq_ev_set_dest(&ev, dst->client, dst->port);   /* OUT PORT picked one device */
    else snd_seq_ev_set_subs(&ev);                               /* every subscriber of "SysEx Out" */
    snd_seq_ev_set_direct(&ev);
    ev.type = SND_SEQ_EVENT_SYSEX;
    /* ext payloads need LENGTH_VARIABLE: with the cleared flags (LENGTH_FIXED)
     * output_direct rejects the event with -EINVAL (probed on x86). */
    snd_seq_ev_set_variable(&ev, len, (void *)(uintptr_t)p);
    return snd_seq_event_output_direct(s->seq, &ev);
}

/* CLOCK_MONOTONIC in ms: the echo window runs on the clock, not on anything the
 * host, a project or a preset can move. */
static long long mono_ms(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (long long)t.tv_sec * 1000 + t.tv_nsec / 1000000;
}

static void echo_add(sb_t *s, uint32_t h) {
    echo_t e = { h, mono_ms() };
    if (s->echo_n < ECHO_SLOTS) s->echo[s->echo_n++] = e;
    else {
        memmove(s->echo, s->echo + 1, sizeof(s->echo) - sizeof s->echo[0]);
        s->echo[ECHO_SLOTS - 1] = e;
    }
}

/* Our own message coming back -- but only while that is still plausible: an echo
 * round-trips in milliseconds, so a hash last seen ECHO_MS ago is a real message
 * (the same patch read back from the synth minutes later must count, or a
 * round-trip test loses its replies to the filter forever). */
static int echo_seen(sb_t *s, uint32_t h) {
    long long now = mono_ms();
    for (int i = 0; i < s->echo_n; i++)
        if (s->echo[i].h == h && now - s->echo[i].at <= ECHO_MS) return 1;
    return 0;
}

static void ts_add_ms(struct timespec *t, int ms) {
    t->tv_sec += ms / 1000;
    t->tv_nsec += (long)(ms % 1000) * 1000000L;
    if (t->tv_nsec >= 1000000000L) { t->tv_sec++; t->tv_nsec -= 1000000000L; }
}

static int ts_before(const struct timespec *a, const struct timespec *b) {
    return a->tv_sec < b->tv_sec || (a->tv_sec == b->tv_sec && a->tv_nsec < b->tv_nsec);
}

/* STATUS while a paced send runs: messages for a multi-message file, kilobytes for
 * one huge message, nothing for the handful of BYTES. "sending..." is what do_send
 * leaves behind, so anything else means the job is out (or refused). */
static void tx_progress(sb_t *s) {
    if (s->tx_direct) return;
    if (s->tx_total > 1)
        set_status(s, "sending %d/%d msg", s->tx_done + (s->tx_ms < s->tx_me ? 1 : 0), s->tx_total);
    else if (s->tx_total_bytes >= 1024)
        set_status(s, "sending %d/%d KB", s->tx_bytes / 1024,
                   (s->tx_total_bytes + 1023) / 1024);
}

/* The queue is drained: release it and report the same STATUS a synchronous send
 * would have left. */
static void tx_finish(sb_t *s) {
    int n = s->tx_done, stray = s->tx_stray, direct = s->tx_direct;
    size_t bytes = s->tx_bytes;
    free(s->txq);
    s->txq = NULL;
    s->tx_active = 0;
    s->tx_ms = s->tx_me = 0;
    if (direct) set_status(s, "sent %d B", (int)bytes);
    else if (stray) set_status(s, "sent %d msg, %d B stray", n, stray);
    else set_status(s, "sent %d msg, %d B", n, (int)bytes);
}

static void tx_fail(sb_t *s, int r) {
    int n = s->tx_done, direct = s->tx_direct;
    free(s->txq);
    s->txq = NULL;
    s->tx_active = 0;
    s->tx_ms = s->tx_me = 0;
    if (direct) set_status(s, "send failed (%d)", r);
    else set_status(s, "send failed after %d msg (%d)", n, r);
}

/* Hand a loaded buffer to the tx thread. total_msgs/total_bytes only feed the
 * progress line; direct = the buffer is one message already (BYTES), so nothing
 * is scanned for F0..F7. */
static void tx_begin(sb_t *s, uint8_t *q, size_t len, const snd_seq_addr_t *dst,
                     int direct, int total_msgs, size_t total_bytes) {
    s->txq = q;
    s->txq_len = len;
    s->tx_pos = 0;
    s->tx_ms = s->tx_me = 0;
    s->tx_hash = 2166136261u;
    s->tx_direct = direct;
    s->tx_done = 0;
    s->tx_total = total_msgs;
    s->tx_bytes = 0;
    s->tx_total_bytes = total_bytes;
    s->tx_stray = 0;
    s->tx_dst = *dst;
    clock_gettime(CLOCK_REALTIME, &s->tx_due);   /* the first block goes out now */
    s->tx_active = 1;
    set_status(s, "sending...");
    pthread_cond_broadcast(&s->tx_cv);
}

/* One tick of the paced send: the next tx_buf-byte block of the open message, or
 * the first block of the next one, never before tx_due. A block is a slice of the
 * same F0..F7 stream -- no framing is added, the receiver sees the identical byte
 * sequence, just not all at once (Midi-OX's "buffer size + delay between buffers").
 * Called with s->lock held, from tx_worker. */
static void tx_step(sb_t *s, const struct timespec *now) {
    if (s->tx_ms >= s->tx_me) {                       /* open the next message */
        if (s->tx_direct) {
            if (s->tx_pos >= s->txq_len) { tx_finish(s); return; }
            s->tx_ms = 0;
            s->tx_me = s->txq_len;
        } else {
            size_t i = s->tx_pos;
            while (i < s->txq_len && s->txq[i] != 0xF0) i++;      /* stray bytes */
            s->tx_stray += (int)(i - s->tx_pos);
            if (i >= s->txq_len) { s->tx_pos = i; tx_finish(s); return; }
            size_t j = i + 1;
            while (j < s->txq_len && s->txq[j] != 0xF7) j++;
            if (j >= s->txq_len) {                                /* unterminated tail */
                s->tx_stray += (int)(s->txq_len - i);
                s->tx_pos = s->txq_len;
                tx_finish(s);
                return;
            }
            s->tx_pos = s->tx_ms = i;
            s->tx_me = j + 1;
        }
        s->tx_hash = 2166136261u;
    }
    int n = (int)(s->tx_me - s->tx_pos);
    if (n > s->tx_buf) n = s->tx_buf;
    int r = emit(s, s->txq + s->tx_pos, n, &s->tx_dst);
    if (r < 0) { tx_fail(s, r); return; }
    for (int i = 0; i < n; i++) s->tx_hash = (s->tx_hash ^ s->txq[s->tx_pos + i]) * 16777619u;
    s->tx_pos += (size_t)n;
    s->tx_bytes += (size_t)n;
    if (s->tx_pos < s->tx_me) {                     /* more of this message to come */
        s->tx_due = *now;
        ts_add_ms(&s->tx_due, s->tx_gap);
        tx_progress(s);
        return;
    }
    /* the message is out whole: hash it for the echo filter (the receiver
     * reassembles the blocks, so it echoes the whole message, not a slice) */
    echo_add(s, s->tx_hash);
    s->tx_done++;
    tx_progress(s);
    s->tx_ms = s->tx_me = 0;
    if (s->tx_direct) { tx_finish(s); return; }
    size_t k = s->tx_pos;
    while (k < s->txq_len && s->txq[k] != 0xF0) k++;              /* anything left? */
    s->tx_stray += (int)(k - s->tx_pos);
    s->tx_pos = k;
    if (k >= s->txq_len) { tx_finish(s); return; }
    s->tx_due = *now;
    ts_add_ms(&s->tx_due, s->tx_delay);            /* Midi-OX's "delay after F7" */
}

/* The pacing thread: sleeps on tx_cv until tx_due, emits one block, repeats. Not
 * the reader thread -- a 1 ms block gap needs sub-POLL_MS timing and neither loop
 * may stall the other's sweep or drain. */
static void *tx_worker(void *arg) {
    sb_t *s = (sb_t *)arg;
    pthread_mutex_lock(&s->lock);
    while (!s->tx_quit) {
        if (!s->tx_active) { pthread_cond_wait(&s->tx_cv, &s->lock); continue; }
        struct timespec now;
        clock_gettime(CLOCK_REALTIME, &now);
        if (ts_before(&now, &s->tx_due)) {
            pthread_cond_timedwait(&s->tx_cv, &s->lock, &s->tx_due);
            continue;
        }
        if (ts_before(&s->tx_due, &now)) clock_gettime(CLOCK_REALTIME, &now);
        tx_step(s, &now);
    }
    if (s->tx_active) {                      /* quitting mid-send: drop the queue */
        free(s->txq);
        s->txq = NULL;
        s->tx_active = 0;
    }
    pthread_mutex_unlock(&s->lock);
    return NULL;
}

/* SEND: validate, load the file, queue it. Nothing is transmitted here -- the host
 * thread must not sleep out a paced dump -- so STATUS reads "sending..." until the
 * tx thread reports the outcome. BYTES is queued too, for one code path. */
static void do_send(sb_t *s) {
    if (!s->seq) { set_status(s, "no ALSA sequencer"); return; }
    if (s->tx_active) { set_status(s, "already sending"); return; }
    rebuild_targets(s);                        /* the pick may have just appeared */
    snd_seq_addr_t dst;
    if (out_target(s, &dst) < 0) return;       /* out_target already said why */
    if (s->send_src == 1) {
        int n = clampi(s->nbytes, 1, 8);
        uint8_t *q = (uint8_t *)malloc((size_t)n);
        if (!q) { set_status(s, "out of memory"); return; }
        for (int i = 0; i < n; i++) q[i] = (uint8_t)s->b[i];
        tx_begin(s, q, (size_t)n, &dst, 1, 1, (size_t)n);
        return;
    }
    const char *name = slot_name(s);
    if (!name) { set_status(s, "no file in slot %d", s->message); return; }
    char path[sizeof s->dir + SB_NAME + 2];
    snprintf(path, sizeof path, "%s/%s", s->dir, name);
    FILE *f = fopen(path, "rb");
    if (!f) { set_status(s, "can't open %s", name); return; }
    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); set_status(s, "can't read %s", name); return; }
    long sz = ftell(f);
    if (sz < 0 || sz > MAX_FILE) { fclose(f); set_status(s, "%s too large", name); return; }
    rewind(f);
    uint8_t *buf = (uint8_t *)malloc((size_t)sz ? (size_t)sz : 1);
    size_t got = buf ? fread(buf, 1, (size_t)sz, f) : 0;
    fclose(f);
    if (!buf || got != (size_t)sz) { free(buf); set_status(s, "can't read %s", name); return; }

    /* split on complete F0..F7 messages the way the tx thread will, for the
     * progress line; stray bytes count but never go out */
    int msgs = 0;
    size_t bytes = 0, i = 0;
    while (i < got) {
        if (buf[i] == 0xF0) {
            size_t j = i + 1;
            while (j < got && buf[j] != 0xF7) j++;
            if (j >= got) break;                     /* unterminated tail */
            msgs++;
            bytes += j - i + 1;
            i = j + 1;
        } else i++;
    }
    tx_begin(s, buf, (size_t)sz, &dst, 0, msgs, bytes);
}

/* SAVE RX: the whole capture -- every message since CLEAR, one normal
 * multi-message .syx, which is what a bank dump should have been all along -- to
 * messages/rx_NNN.syx, then re-scan so it becomes a slot like any other file. */
static void do_save_rx(sb_t *s) {
    if (!s->cap || s->cap_len <= 0) { set_status(s, "nothing to save"); return; }
    if (!s->dir[0]) { set_status(s, "no folder"); return; }
    char path[sizeof s->dir + 24];
    int idx = 1;
    while (idx < 1000) {
        snprintf(path, sizeof path, "%s/rx_%03d.syx", s->dir, idx);
        if (access(path, F_OK) != 0) break;
        idx++;
    }
    if (idx >= 1000) { set_status(s, "rx_ slots full"); return; }
    FILE *f = fopen(path, "wb");
    if (!f) { set_status(s, "can't write rx_%03d.syx", idx); return; }
    size_t w = fwrite(s->cap, 1, (size_t)s->cap_len, f);
    fclose(f);
    if (w != (size_t)s->cap_len) { set_status(s, "write failed"); return; }
    scan_dir(s);
    set_status(s, "saved rx_%03d.syx (%d msg, %d KB)", idx, s->cap_msgs,
               (s->cap_len + 1023) / 1024);
}

/* ------------------------------------------------------------- sequencer */

static int has_sub(sb_t *s, int client, int port) {
    for (int i = 0; i < s->nsubs; i++)
        if (s->subs[i].client == client && s->subs[i].port == port) return 1;
    return 0;
}

static void forget_sub(sb_t *s, int i) {
    s->subs[i] = s->subs[--s->nsubs];
}

static int has_osub(sb_t *s, int client, int port) {
    for (int i = 0; i < s->nosubs; i++)
        if (s->osubs[i].client == client && s->osubs[i].port == port) return 1;
    return 0;
}

static void forget_osub(sb_t *s, int i) {
    s->osubs[i] = s->osubs[--s->nosubs];
}

/* One subscription, direction chosen by who is sender: dir 0 = their out-port
 * -> our in-port (receiving), dir 1 = our out-port -> their in-port (sending). */
static int make_sub(sb_t *s, int dir, int cid, int pid) {
    snd_seq_port_subscribe_t *sub;
    snd_seq_port_subscribe_alloca(&sub);
    snd_seq_addr_t sender = { .client = (unsigned)(dir ? s->self_client : cid),
                              .port = (unsigned)(dir ? s->out_port : pid) };
    snd_seq_addr_t dest = { .client = (unsigned)(dir ? cid : s->self_client),
                            .port = (unsigned)(dir ? pid : s->in_port) };
    snd_seq_port_subscribe_set_sender(sub, &sender);
    snd_seq_port_subscribe_set_dest(sub, &dest);
    return snd_seq_subscribe_port(s->seq, sub);
}

/* Is this subscription still there? (dir as make_sub) */
static int sub_alive(sb_t *s, int dir, int cid, int pid) {
    snd_seq_port_subscribe_t *sub;
    snd_seq_port_subscribe_alloca(&sub);
    snd_seq_addr_t sender = { .client = (unsigned)(dir ? s->self_client : cid),
                              .port = (unsigned)(dir ? s->out_port : pid) };
    snd_seq_addr_t dest = { .client = (unsigned)(dir ? cid : s->self_client),
                            .port = (unsigned)(dir ? pid : s->in_port) };
    snd_seq_port_subscribe_set_sender(sub, &sender);
    snd_seq_port_subscribe_set_dest(sub, &dest);
    return snd_seq_get_port_subscription(s->seq, sub) >= 0;
}

/* Undo one subscription (dir as make_sub). */
static void drop_sub(sb_t *s, int dir, int cid, int pid) {
    snd_seq_port_subscribe_t *sub;
    snd_seq_port_subscribe_alloca(&sub);
    snd_seq_addr_t sender = { .client = (unsigned)(dir ? s->self_client : cid),
                              .port = (unsigned)(dir ? s->out_port : pid) };
    snd_seq_addr_t dest = { .client = (unsigned)(dir ? cid : s->self_client),
                            .port = (unsigned)(dir ? pid : s->in_port) };
    snd_seq_port_subscribe_set_sender(sub, &sender);
    snd_seq_port_subscribe_set_dest(sub, &dest);
    snd_seq_unsubscribe_port(s->seq, sub);
}

/* The sweep, every 2 s (and whenever a picker moves): refresh the OUT PORT /
 * IN PORT candidate lists, then wire ourselves in -- every readable port feeds
 * our in-port while LISTEN is ON (or only the one IN PORT picked), and our
 * out-port feeds every writable port so pressing SEND reaches a device with no
 * manual aconnect. Subscriptions whose far end went away, or that the IN PORT
 * pick no longer wants, are dropped here too. Own client and SYSTEM are never
 * wired. */
static void sweep(sb_t *s) {
    if (!s->seq) return;
    rebuild_targets(s);
    for (int i = 0; i < s->nsubs; ) {
        if (!sub_alive(s, 0, s->subs[i].client, s->subs[i].port)) forget_sub(s, i);
        else if (!in_wanted(s, s->subs[i].client, s->subs[i].port)) {
            drop_sub(s, 0, s->subs[i].client, s->subs[i].port);
            forget_sub(s, i);
        } else i++;
    }
    for (int i = 0; i < s->nosubs; ) {
        if (!sub_alive(s, 1, s->osubs[i].client, s->osubs[i].port)) forget_osub(s, i);
        else i++;
    }
    if (s->listen)
        for (int i = 0; i < s->ntg_in && s->nsubs < MAX_SUBS; i++) {
            int cid = s->tg_in[i].client, pid = s->tg_in[i].port;
            if (!in_wanted(s, cid, pid) || has_sub(s, cid, pid)) continue;
            int r = make_sub(s, 0, cid, pid);
            if (r == 0 || r == -EEXIST) {         /* EEXIST: from an earlier sweep */
                s->subs[s->nsubs].client = cid;
                s->subs[s->nsubs].port = pid;
                s->nsubs++;
            }
        }
    for (int i = 0; i < s->ntg_out && s->nosubs < MAX_SUBS; i++) {
        int cid = s->tg_out[i].client, pid = s->tg_out[i].port;
        if (has_osub(s, cid, pid)) continue;
        int r = make_sub(s, 1, cid, pid);
        if (r == 0 || r == -EEXIST) {
            s->osubs[s->nosubs].client = cid;
            s->osubs[s->nosubs].port = pid;
            s->nosubs++;
        }
    }
}

/* LISTEN OFF: drop the receiving side only -- sending stays wired. */
static void unsubscribe_in(sb_t *s) {
    if (!s->seq) { s->nsubs = 0; return; }
    while (s->nsubs > 0) {
        int i = --s->nsubs;
        drop_sub(s, 0, s->subs[i].client, s->subs[i].port);
    }
}

static void unsubscribe_all(sb_t *s) {
    unsubscribe_in(s);
    if (!s->seq) { s->nosubs = 0; return; }
    while (s->nosubs > 0) {
        int i = --s->nosubs;
        drop_sub(s, 1, s->osubs[i].client, s->osubs[i].port);
    }
}

/* Accept one complete F0..F7 message: filter our own echo, APPEND it to the
 * capture (so a bank dump's 200-odd blocks all survive, not just the newest),
 * show it and who sent it, count it (RX TOTAL = complete messages, wraps 0..999),
 * and move RX MSG to it. The sender's name is what makes an unexplained RX TOTAL
 * climb diagnosable: STATUS says which client is talking, and how big the capture
 * it is building has grown. */
static void accept_msg(sb_t *s, const uint8_t *p, int len, int src_client) {
    if (echo_seen(s, fnv1a(p, len))) return;      /* our own message coming back */
    if (!s->cap) { set_status(s, "rx %d B: no capture buffer", len); return; }
    if (s->cap_msgs >= CAP_MSGS || (long long)s->cap_len + len > CAP_MAX) {
        set_status(s, "capture full (%d msg/%d KB)", s->cap_msgs, CAP_MAX / 1024);
        return;
    }
    memcpy(s->cap + s->cap_len, p, (size_t)len);
    s->cap_off[s->cap_msgs++] = s->cap_len;
    s->cap_len += len;
    s->rx_msg = s->cap_msgs;                     /* a new message moves RX to the end */
    s->rx_total = (s->rx_total + 1) % 1000;
    char from[40];
    snd_seq_client_info_t *ci;
    snd_seq_client_info_alloca(&ci);
    if (snd_seq_get_any_client_info(s->seq, src_client, ci) >= 0) {
        const char *nm = snd_seq_client_info_get_name(ci);
        snprintf(from, sizeof from, "%s", (nm && nm[0]) ? nm : "?");
    } else snprintf(from, sizeof from, "client %d", src_client);
    set_status(s, "rx %d B from %s (%d msg, %d KB)", len, from, s->cap_msgs,
               (s->cap_len + 1023) / 1024);
}

/* Drain everything queued: one poll() wakes us, this loop takes it all (poll
 * per event would sleep on the rest of a burst). The bytes of a long SysEx
 * arrive split across several events, so they accumulate in s->acc and only
 * complete F0..F7 messages are accepted -- the app's extract_messages() rule;
 * stray bytes before an F0 are dropped, an unterminated tail waits for the
 * rest of the message. An F0 inside that open tail means the open message was
 * aborted (a looped-back fragment -- "Midi Through" echoes everything -- a
 * device reset, two sources interleave): data bytes are 7-bit, so F0 cannot
 * occur inside a SysEx, and a new one restarts the message there instead of
 * letting dead bytes prepend to whatever arrives next. */
static void drain(sb_t *s) {
    snd_seq_event_t *ev;
    int r;
    while ((r = snd_seq_event_input(s->seq, &ev)) >= 0) {
        if (ev->type != SND_SEQ_EVENT_SYSEX) continue;
        const uint8_t *p = (const uint8_t *)ev->data.ext.ptr;
        int len = (int)ev->data.ext.len;
        if (!s->listen || len <= 0) continue;
        if (s->acc_len + len > ACC_MAX) {          /* never completed: drop it */
            s->acc_len = 0;
            set_status(s, "rx message over %d KB", ACC_MAX / 1024);
            continue;
        }
        memcpy(s->acc + s->acc_len, p, (size_t)len);
        s->acc_len += len;
        for (;;) {
            int i = 0;
            while (i < s->acc_len && s->acc[i] != 0xF0) i++;   /* stray bytes */
            if (i) { memmove(s->acc, s->acc + i, (size_t)(s->acc_len - i)); s->acc_len -= i; }
            if (s->acc_len < 2 || s->acc[0] != 0xF0) break;    /* nothing to wait for */
            int j = 1, k = -1;
            while (j < s->acc_len && s->acc[j] != 0xF7) {
                if (s->acc[j] == 0xF0 && k < 0) k = j;         /* a new message started */
                j++;
            }
            if (k >= 0) {                                      /* the open one never will */
                memmove(s->acc, s->acc + k, (size_t)(s->acc_len - k));
                s->acc_len -= k;
                continue;
            }
            if (j >= s->acc_len) break;                        /* tail still missing */
            accept_msg(s, s->acc, j + 1, ev->source.client);
            memmove(s->acc, s->acc + j + 1, (size_t)(s->acc_len - j - 1));
            s->acc_len -= j + 1;
        }
    }
}

static void *reader(void *arg) {
    sb_t *s = (sb_t *)arg;
    struct pollfd pfd[2];
    memset(pfd, 0, sizeof pfd);
    pfd[0].events = POLLIN;
    /* snapshot the sequencer's fd once, so poll() never races an ALSA call */
    if (snd_seq_poll_descriptors(s->seq, pfd, 1, POLLIN) < 1) pfd[0].fd = -1;
    pfd[1].fd = s->quit_pipe[0];
    pfd[1].events = POLLIN;
    struct timespec last = {0, 0};
    for (;;) {
        int pr = poll(pfd, 2, POLL_MS);
        if (pfd[1].revents & (POLLIN | POLLHUP | POLLERR)) break;
        if (pr > 0 && pfd[0].fd >= 0) {
            if (pfd[0].revents & (POLLIN | POLLHUP | POLLERR)) {
                pthread_mutex_lock(&s->lock);
                drain(s);
                pthread_mutex_unlock(&s->lock);
            }
            if (pfd[0].revents & (POLLHUP | POLLERR)) break;   /* sequencer gone */
        }
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        long ms = (now.tv_sec - last.tv_sec) * 1000 + (now.tv_nsec - last.tv_nsec) / 1000000;
        int due;
        pthread_mutex_lock(&s->lock);
        due = s->need_sweep || ms >= SWEEP_MS;
        pthread_mutex_unlock(&s->lock);
        if (due) {
            pthread_mutex_lock(&s->lock);
            s->need_sweep = 0;
            sweep(s);              /* out-side always; in-side only while LISTEN is ON */
            pthread_mutex_unlock(&s->lock);
            last = now;
        }
    }
    return NULL;
}

/* ------------------------------------------------------------ parameters */

/* Every engine-driven numeric in one hash: the wrapper's display_rev diff
 * reports exactly the params whose value moved (rx_total, message after a
 * prev/next tap, a chunk restore). */
static unsigned rev_hash(sb_t *s) {
    uint32_t h = 2166136261u;
    /* A file name too long for the MESSAGE box scrolls through it. Advancing the
     * phase HERE is what makes it move: rev_scan sees display_rev change and asks
     * the host to repaint, which re-reads message_display (a Label never wraps). */
    const char *nm = slot_name(s);
    marq_sync(s);                              /* resets the phase when the selection moved */
    if (nm && (int)strlen(nm) > MARQ_W && (s->marq_tick++ & 1) == 0)
        s->marq_ph = (s->marq_ph + 1) % (unsigned)((int)strlen(nm) + MARQ_GAP);
    else if (!nm || (int)strlen(nm) <= MARQ_W)
        s->marq_ph = 0;                          /* fits: stop where it started */
    h = (h ^ s->marq_ph) * 16777619u;
    int v[19] = { s->message, s->send_src, s->nbytes, s->listen, s->rx_total,
                  s->b[0], s->b[1], s->b[2], s->b[3], s->b[4], s->b[5], s->b[6], s->b[7],
                  s->send_to, s->recv_from, s->rx_msg,
                  s->tx_delay, s->tx_buf, s->tx_gap };
    for (int i = 0; i < 19; i++) {
        h = (h ^ (uint32_t)v[i]) * 16777619u;
        h = (h ^ (uint32_t)(v[i] >> 8)) * 16777619u;
        h = (h ^ (uint32_t)(v[i] >> 16)) * 16777619u;
        h = (h ^ (uint32_t)(v[i] >> 24)) * 16777619u;
    }
    /* and the names the pickers resolve to: a device unplugged under a fixed
     * index renames the entry, which must re-poll the readout too */
    for (int out = 0; out < 2; out++)
        for (const char *p = tg_name(s, out, out ? s->send_to : s->recv_from); *p; p++)
            h = (h ^ (uint32_t)(unsigned char)*p) * 16777619u;
    return h;
}

static int put(char *buf, int n, const char *fmt, ...) {
    if (n <= 0) return 0;
    va_list ap;
    va_start(ap, fmt);
    int r = vsnprintf(buf, (size_t)n, fmt, ap);
    va_end(ap);
    return r > 0 && r < n ? r : (int)strlen(buf);
}

static int state_get(sb_t *s, char *buf, int n) {
    int o = 0;
    int c = put(buf + o, n - o, "message=%d;send_src=%d;nbytes=%d;", s->message, s->send_src, s->nbytes);
    o += c;
    for (int i = 0; i < 8 && o < n; i++) { c = put(buf + o, n - o, "b%d=%d;", i + 1, s->b[i]); o += c; }
    if (o < n) { c = put(buf + o, n - o, "listen=%d;", s->listen); o += c; }
    if (o < n) { c = put(buf + o, n - o, "send_to=%d;recv_from=%d;", s->send_to, s->recv_from); o += c; }
    /* and the pick's identity: an index alone comes back aimed at a different
     * device once the list has moved, exactly as file= does for a message slot */
    {   const char *sn = aim_name(s, 1), *rn = aim_name(s, 0);
        if (sn[0] && o < n) { c = put(buf + o, n - o, "send_name=%s;", sn); o += c; }
        if (rn[0] && o < n) { c = put(buf + o, n - o, "recv_name=%s;", rn); o += c; }
    }
    if (o < n) { c = put(buf + o, n - o, "tx_delay=%d;tx_buf=%d;tx_gap=%d;",
                         s->tx_delay, s->tx_buf, s->tx_gap); o += c; }
    if (o < n) { c = put(buf + o, n - o, "rx_msg=%d;", s->rx_msg); o += c; }
    const char *nm = slot_name(s);
    if (nm && o < n) { c = put(buf + o, n - o, "file=%s;", nm); o += c; }
    if (o >= n && n > 0) { buf[n - 1] = 0; o = n - 1; }
    return o;
}

/* "k=v;..." lines. file= wins over message= when the name still exists, so a
 * preset follows the file even if the folder shifted around it; send_name= /
 * recv_name= win over send_to= / recv_from= the same way, for the same reason. */
static void state_set(sb_t *s, const char *str) {
    char file[SB_NAME] = "";
    char sname[SB_TEXT] = "", rname[SB_TEXT] = "";
    const char *p = str;
    while (*p) {
        const char *eq = strchr(p, '=');
        if (!eq) break;
        const char *semi = strchr(eq + 1, ';');
        int klen = (int)(eq - p), vlen = semi ? (int)(semi - eq - 1) : (int)strlen(eq + 1);
        const char *k = p;
        char v[SB_NAME];                       /* NUL-terminated copy of the value */
        int c = vlen < SB_NAME - 1 ? vlen : SB_NAME - 1;
        memcpy(v, eq + 1, (size_t)c);
        v[c] = 0;
        if (klen == 7 && !memcmp(k, "message", 7)) s->message = clampi(atoi(v), 1, MAX_MSGS);
        else if (klen == 8 && !memcmp(k, "send_src", 8)) s->send_src = opt_index(v, "FILE", "BYTES");
        else if (klen == 6 && !memcmp(k, "nbytes", 6)) s->nbytes = clampi(atoi(v), 1, 8);
        else if (klen == 2 && k[0] == 'b' && k[1] >= '1' && k[1] <= '8') s->b[k[1] - '1'] = clampi(atoi(v), 0, 255);
        else if (klen == 6 && !memcmp(k, "listen", 6)) s->listen = opt_index(v, "OFF", "ON");
        else if (klen == 7 && !memcmp(k, "send_to", 7)) s->send_to = clampi(atoi(v), 0, MAX_TG);
        else if (klen == 9 && !memcmp(k, "recv_from", 9)) s->recv_from = clampi(atoi(v), 0, MAX_TG);
        else if (klen == 9 && !memcmp(k, "send_name", 9)) snprintf(sname, sizeof sname, "%s", v);
        else if (klen == 9 && !memcmp(k, "recv_name", 9)) snprintf(rname, sizeof rname, "%s", v);
        else if (klen == 8 && !memcmp(k, "tx_delay", 8)) s->tx_delay = clampi(atoi(v), 0, 1000);
        else if (klen == 6 && !memcmp(k, "tx_buf", 6)) s->tx_buf = clampi(atoi(v), 64, 4096);
        else if (klen == 6 && !memcmp(k, "tx_gap", 6)) s->tx_gap = clampi(atoi(v), 0, 1000);
        else if (klen == 6 && !memcmp(k, "rx_msg", 6)) s->rx_msg = clampi(atoi(v), 1, CAP_MSGS);
        else if (klen == 4 && !memcmp(k, "file", 4)) snprintf(file, sizeof file, "%s", v);
        p = semi ? semi + 1 : p + strlen(p);
    }
    if (file[0])
        for (int i = 0; i < s->nmsgs; i++)
            if (!strcmp(s->names[i], file)) { s->message = i + 1; break; }
    marq_sync(s);                     /* a restored selection starts its window at the head */
    aim_restore(s, 1, sname);         /* the picks come back by name when they have one */
    aim_restore(s, 0, rname);
    s->need_sweep = 1;                /* settle the names and re-wire at once */
}

static int sb_get(void *inst, const char *key, char *buf, int n) {
    sb_t *s = (sb_t *)inst;
    if (!s || !key || !buf || n <= 0) return 0;
    pthread_mutex_lock(&s->lock);
    int r = 0, ln;
    if (!strcmp(key, "state")) r = state_get(s, buf, n);
    else if (!strcmp(key, "display_rev")) r = put(buf, n, "%08x", rev_hash(s));
    else if (!strcmp(key, "message")) r = put(buf, n, "%d", s->message);
    else if (!strcmp(key, "message_display")) {
        char nm[SB_NAME];
        r = put(buf, n, "%s", msg_name(s, nm, sizeof nm));
    }
    else if (!strcmp(key, "send_src")) r = put(buf, n, "%d", s->send_src);
    else if (!strcmp(key, "nbytes")) r = put(buf, n, "%d", s->nbytes);
    else if (!strcmp(key, "listen")) r = put(buf, n, "%d", s->listen);
    else if (!strcmp(key, "send_to")) r = put(buf, n, "%d", s->send_to);
    else if (!strcmp(key, "recv_from")) r = put(buf, n, "%d", s->recv_from);
    else if (!strcmp(key, "tx_delay")) r = put(buf, n, "%d", s->tx_delay);
    else if (!strcmp(key, "tx_buf")) r = put(buf, n, "%d", s->tx_buf);
    else if (!strcmp(key, "tx_gap")) r = put(buf, n, "%d", s->tx_gap);
    else if (!strcmp(key, "send_to_display")) r = put(buf, n, "%s", tg_name(s, 1, s->send_to));
    else if (!strcmp(key, "recv_from_display")) r = put(buf, n, "%s", tg_name(s, 0, s->recv_from));
    else if (!strcmp(key, "rx_total")) r = put(buf, n, "%d", s->rx_total);
    else if (!strcmp(key, "rx_msg")) r = put(buf, n, "%d", s->rx_msg);
    else if (!strcmp(key, "rx_msg_display"))
        r = s->cap_msgs > 0 ? put(buf, n, "%d/%d", clampi(s->rx_msg, 1, s->cap_msgs), s->cap_msgs)
                            : put(buf, n, "-");
    else if (!strncmp(key, "b", 1) && key[1] >= '1' && key[1] <= '8' && !key[2]) r = put(buf, n, "%d", s->b[key[1] - '1']);
    else if (!strcmp(key, "send") || !strcmp(key, "msg_prev") || !strcmp(key, "msg_next") ||
             !strcmp(key, "save_rx") || !strcmp(key, "rescan") ||
             !strcmp(key, "clear_rx") ||
             !strcmp(key, "rx_msg_prev") || !strcmp(key, "rx_msg_next") ||
             !strcmp(key, "send_to_prev") || !strcmp(key, "send_to_next") ||
             !strcmp(key, "recv_from_prev") || !strcmp(key, "recv_from_next"))
        r = put(buf, n, "0");                    /* momentary springs back */
    else if (!strcmp(key, "tx")) r = tx_line(s, 1, buf, n);
    else if (!strcmp(key, "rx")) r = rx_line(s, 1, buf, n);
    else if ((ln = line_no(key, 't', TX_LINES)) > 0) r = tx_line(s, ln, buf, n);
    else if ((ln = line_no(key, 'r', RX_LINES)) > 0) r = rx_line(s, ln, buf, n);
    else if (!strcmp(key, "status")) r = put(buf, n, "%s", s->status);
    else if (!strcmp(key, "folder")) r = put(buf, n, "%s", s->folder);
    else if (!strcmp(key, "port")) r = put(buf, n, "%s", s->port_str);
    pthread_mutex_unlock(&s->lock);
    return r;   /* 0 = no such key: the wrapper's "<key>_on" probe, readout sets */
}

static void sb_set(void *inst, const char *key, const char *val) {
    sb_t *s = (sb_t *)inst;
    if (!s || !key || !val) return;
    pthread_mutex_lock(&s->lock);
    s->sel_dirty = 1;                       /* any set may have changed what TX shows */
    if (!strcmp(key, "state")) state_set(s, val);
    else if (!strcmp(key, "message")) { s->message = clampi(atoi(val), 1, MAX_MSGS); marq_sync(s); }
    else if (!strcmp(key, "send_src")) s->send_src = opt_index(val, "FILE", "BYTES");
    else if (!strcmp(key, "nbytes")) s->nbytes = clampi(atoi(val), 1, 8);
    else if (!strncmp(key, "b", 1) && key[1] >= '1' && key[1] <= '8' && !key[2])
        s->b[key[1] - '1'] = clampi(atoi(val), 0, 255);
    else if (!strcmp(key, "listen")) {
        int on = opt_index(val, "OFF", "ON");
        if (on != s->listen) {
            s->listen = on;
            if (on) { s->need_sweep = 1; set_status(s, "listening"); }
            else { unsubscribe_in(s); s->acc_len = 0; set_status(s, "listen off"); }
        }
    }
    else if (!strcmp(key, "send")) { if (atoi(val) > 0) do_send(s); }
    else if (!strcmp(key, "tx_delay")) s->tx_delay = clampi(atoi(val), 0, 1000);
    else if (!strcmp(key, "tx_buf")) s->tx_buf = clampi(atoi(val), 64, 4096);
    else if (!strcmp(key, "tx_gap")) s->tx_gap = clampi(atoi(val), 0, 1000);
    else if (!strcmp(key, "rx_msg"))
        s->rx_msg = clampi(atoi(val), 1, s->cap_msgs ? s->cap_msgs : 1);
    else if (!strcmp(key, "rx_msg_prev")) {
        if (atoi(val) > 0 && s->rx_msg > 1) s->rx_msg--;
    }
    else if (!strcmp(key, "rx_msg_next")) {
        if (atoi(val) > 0 && s->rx_msg < s->cap_msgs) s->rx_msg++;
    }
    else if (!strcmp(key, "send_to")) {
        s->send_to = clampi(atoi(val), 0, MAX_TG); aim_clear(s, 1); s->need_sweep = 1;
    }
    else if (!strcmp(key, "recv_from")) {
        s->recv_from = clampi(atoi(val), 0, MAX_TG); aim_clear(s, 0); s->need_sweep = 1;
    }
    else if (!strcmp(key, "send_to_prev")) { if (atoi(val) > 0) step_target(s, 1, -1); }
    else if (!strcmp(key, "send_to_next")) { if (atoi(val) > 0) step_target(s, 1, +1); }
    else if (!strcmp(key, "recv_from_prev")) { if (atoi(val) > 0) step_target(s, 0, -1); }
    else if (!strcmp(key, "recv_from_next")) { if (atoi(val) > 0) step_target(s, 0, +1); }
    else if (!strcmp(key, "save_rx")) { if (atoi(val) > 0) do_save_rx(s); }
    else if (!strcmp(key, "clear_rx")) {
        if (atoi(val) > 0) {
            s->cap_len = 0; s->cap_msgs = 0; s->rx_msg = 1;
            s->acc_len = 0; s->rx_total = 0;
            set_status(s, "cleared");
        }
    }
    else if (!strcmp(key, "rescan")) {
        if (atoi(val) > 0) { scan_dir(s); s->need_sweep = 1; set_status(s, "%d messages", s->nmsgs); }
    }
    /* readouts (tx/rx/status/folder/port/rx_total) and the step_of triggers the
     * wrapper never sends: ignored on purpose. */
    pthread_mutex_unlock(&s->lock);
}

/* ------------------------------------------------------------- lifecycle */

static void set_folder(sb_t *s, const char *dir) {
    if (!dir || !dir[0]) { snprintf(s->folder, sizeof s->folder, "(no folder)"); s->dir[0] = 0; return; }
    snprintf(s->dir, sizeof s->dir, "%s", dir);
    size_t l = strlen(dir);
    if (l < sizeof s->folder) snprintf(s->folder, sizeof s->folder, "%s", dir);
    else snprintf(s->folder, sizeof s->folder, "...%s", dir + l - (sizeof s->folder - 4));
}

static void *sb_create(const char *data_dir) {
    sb_t *s = (sb_t *)calloc(1, sizeof *s);
    if (!s) return NULL;
    pthread_mutexattr_t ma;
    pthread_mutexattr_init(&ma);
    pthread_mutexattr_setprotocol(&ma, PTHREAD_PRIO_INHERIT);   /* like the wrapper's lock */
    pthread_mutex_init(&s->lock, &ma);
    pthread_mutexattr_destroy(&ma);
    s->quit_pipe[0] = s->quit_pipe[1] = -1;
    pthread_cond_init(&s->tx_cv, NULL);
    s->message = 1;
    s->send_src = 0;                        /* FILE */
    s->nbytes = 6;
    { int d[8] = {0xF0, 0x7E, 0x7F, 0x06, 0x01, 0xF7, 0, 0}; memcpy(s->b, d, sizeof d); }
    s->listen = 0;                        /* LISTEN defaults OFF (params.json): opt-in receiving */
    s->send_to = 0;                       /* OUT PORT default None: no device picked */
    s->recv_from = 0;                     /* IN PORT default None: no device picked */
    s->tx_delay = 10;                     /* pacing defaults (params.json): gentle, not full
                                           * throttle -- a 32 KB dump overruns a synth's input
                                           * buffer when it arrives all at once */
    s->tx_buf = 512;
    s->tx_gap = 1;
    s->rx_msg = 1;                        /* RX shows msg 1 of an empty capture: "-" */
    s->sel_dirty = 1;                     /* TX reads the selection on first use */
    s->cap = (uint8_t *)calloc(1, CAP_MAX);
    set_folder(s, data_dir);
    scan_dir(s);
    s->sel_dirty = 1;                     /* scan_dir rebuilt the slot list */

    int alsa = 0;
    if (snd_seq_open(&s->seq, "default", SND_SEQ_OPEN_DUPLEX, SND_SEQ_NONBLOCK) >= 0) {
        snd_seq_set_client_name(s->seq, "Sysex Bandit");
        s->out_port = snd_seq_create_simple_port(s->seq, "SysEx Out",
            SND_SEQ_PORT_CAP_READ | SND_SEQ_PORT_CAP_SUBS_READ,
            SND_SEQ_PORT_TYPE_MIDI_GENERIC | SND_SEQ_PORT_TYPE_APPLICATION);
        s->in_port = snd_seq_create_simple_port(s->seq, "SysEx In",
            SND_SEQ_PORT_CAP_WRITE | SND_SEQ_PORT_CAP_SUBS_WRITE,
            SND_SEQ_PORT_TYPE_MIDI_GENERIC | SND_SEQ_PORT_TYPE_APPLICATION);
        if (s->out_port >= 0 && s->in_port >= 0) {
            s->self_client = snd_seq_client_id(s->seq);
            /* A bank dump arrives as a burst of fragments, and the default input pool
             * only holds about five 4 KB events -- a fast sender outruns the reader and
             * the rest are dropped mid-message. 2000 is the largest this ALSA accepts
             * (anything above is silently ignored); cells are allocated on demand, so
             * it costs nothing until a burst actually uses it. */
            snd_seq_client_pool_t *cp;
            snd_seq_client_pool_alloca(&cp);
            if (snd_seq_get_client_pool(s->seq, cp) >= 0) {
                snd_seq_client_pool_set_input_pool(cp, 2000);
                snd_seq_set_client_pool(s->seq, cp);
            }
            alsa = 1;
            snprintf(s->port_str, sizeof s->port_str, "Sysex Bandit c%d out%d in%d",
                     s->self_client, s->out_port, s->in_port);
            s->need_sweep = 1;
        } else {
            snd_seq_close(s->seq);
            s->seq = NULL;
        }
    }
    if (!alsa) {
        snprintf(s->port_str, sizeof s->port_str, "no ALSA sequencer");
        set_status(s, "ALSA unavailable");
    } else if (s->cap) set_status(s, "ready");
    else set_status(s, "no capture buffer");   /* SAVE RX stays refused */

    if (alsa && pipe(s->quit_pipe) == 0 && pthread_create(&s->thr, NULL, reader, s) == 0)
        s->have_thread = 1;
    if (pthread_create(&s->tx_thr, NULL, tx_worker, s) == 0) s->tx_thr_on = 1;
    return s;
}

static void sb_destroy(void *inst) {
    sb_t *s = (sb_t *)inst;
    if (!s) return;
    if (s->tx_thr_on) {                     /* stop pacing first: it emits on s->seq */
        pthread_mutex_lock(&s->lock);
        s->tx_quit = 1;
        pthread_cond_broadcast(&s->tx_cv);
        pthread_mutex_unlock(&s->lock);
        pthread_join(s->tx_thr, NULL);
        s->tx_thr_on = 0;
    }
    if (s->have_thread) {                    /* wake the reader, then wait for it */
        char c = 1;
        ssize_t w = write(s->quit_pipe[1], &c, 1);
        (void)w;
        pthread_join(s->thr, NULL);
    }
    pthread_mutex_lock(&s->lock);
    unsubscribe_all(s);
    if (s->seq) { snd_seq_close(s->seq); s->seq = NULL; }
    pthread_mutex_unlock(&s->lock);
    if (s->quit_pipe[0] >= 0) close(s->quit_pipe[0]);
    if (s->quit_pipe[1] >= 0) close(s->quit_pipe[1]);
    pthread_cond_destroy(&s->tx_cv);
    pthread_mutex_destroy(&s->lock);
    free(s->cap);
    free(s);
}

static void sb_midi(void *inst, const uint8_t *msg, int len) {
    (void)inst; (void)msg; (void)len;   /* the host's notes mean nothing here */
}

/* Silence, always: this instrument's output is the sequencer, not the audio. */
static void sb_render(void *inst, int16_t *out_lr, int frames) {
    (void)inst;
    memset(out_lr, 0, (size_t)frames * 2 * sizeof(int16_t));
}

static const mpc_engine_t ENGINE = {
    sb_create, sb_destroy, sb_midi, sb_set, sb_get, sb_render, NULL,
};

const mpc_engine_t *mpc_engine(void) { return &ENGINE; }
